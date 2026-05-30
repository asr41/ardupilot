-- iridium_telemetry.lua
-- Bidirectional telemetry over Iridium 9603N (RockBLOCK) via scripting serial port.
--
-- Hardware setup:
--   Wire the 9603N TX/RX/GND to a free UART on the autopilot.
--   Set SERIALx_PROTOCOL = 28  (Scripting)
--   Set SERIALx_BAUD     = 19  (19200 baud, the 9603N default)
--
-- Parameter SCR_USER1 controls TX interval in seconds (min 10, default 60).
-- Update it in flight via a CMD_TX_RATE MT command or ground-side param set.
--
-- ── MO packet (vehicle → ground), 37 bytes, big-endian ───────────────────────
--   [0-1]   magic 0xAD 0x12
--   [2]     type  0x01 (telemetry)
--   [3-4]   sequence number      (u16)
--   [5-8]   latitude             (i32, degrees × 1e7)
--   [9-12]  longitude            (i32, degrees × 1e7)
--   [13-16] altitude MSL         (i32, cm)
--   [17-18] velocity north       (i16, cm/s)
--   [19-20] velocity east        (i16, cm/s)
--   [21-22] velocity down        (i16, cm/s)
--   [23-24] roll                 (i16, centidegrees)
--   [25-26] pitch                (i16, centidegrees)
--   [27-28] yaw                  (i16, centidegrees, 0–36000)
--   [29-30] battery voltage      (u16, mV)
--   [31-32] battery current      (i16, cA)
--   [33]    GPS satellites       (u8)
--   [34]    GPS fix type         (u8, 0=none … 5=RTK)
--   [35]    flight mode          (u8)
--   [36]    armed                (u8, 0/1)
--
-- ── MT commands (ground → vehicle) ───────────────────────────────────────────
--   Header: [0] 0xAD  [1] 0x34  [2] cmd
--   0x01 Waypoint  : lat(i32) lng(i32) alt_cm(i32)          → 15 bytes
--   0x02 Parameter : name(16 B, null-padded) value(f32)      → 23 bytes
--   0x03 Mode      : mode(u8)                                → 4 bytes
--   0x04 Arm       : arm(u8, 1=arm 0=disarm)                 → 4 bytes
--   0x05 TX Rate   : rate_seconds(u16, min 10)               → 5 bytes

-- ─── Configuration ────────────────────────────────────────────────────────────
local SERIAL_INSTANCE = 0       -- Nth SERIALx with PROTOCOL=28 (0-indexed)
local BAUD_RATE       = 19200
local PARAM_TX_RATE   = "SCR_USER1"
local DEFAULT_TX_RATE = 60      -- seconds between transmissions

-- ─── State identifiers ────────────────────────────────────────────────────────
local S_INIT       = 0   -- Disabling modem echo on startup
local S_IDLE       = 1   -- Waiting for next TX interval
local S_WRITE_CMD  = 2   -- Sent AT+SBDWB, waiting for READY
local S_WRITE_DATA = 3   -- Sent binary payload, waiting for status + OK
local S_SESSION    = 4   -- Sent AT+SBDIX, waiting for +SBDIX response
local S_READ       = 5   -- Sent AT+SBDRB, accumulating binary MT response

local uart            = nil
local state           = S_INIT
local rx_buf          = ""
local tx_payload      = nil
local seq_num         = 0
local last_tx_ms      = 0
local state_timeout   = 0
local mt_len_expected = 0
local init_sent       = false

-- State transitions that are not automatic resets get 60 s to complete.
local TIMEOUT_MS = 60000

-- ─── Binary pack / unpack helpers (big-endian) ───────────────────────────────
local function u16(v)
    v = math.floor(v) % 65536
    return string.char(math.floor(v / 256), v % 256)
end

local function i16(v)
    return u16(v)   -- two's-complement wraps correctly mod 65536
end

local function i32(v)
    v = math.floor(v) % 4294967296
    return string.char(
        math.floor(v / 16777216) % 256,
        math.floor(v /    65536) % 256,
        math.floor(v /      256) % 256,
        v % 256)
end

local function f32(f)
    if f ~= f      then return "\x7F\xC0\x00\x00" end  -- NaN
    if f == 0.0    then return "\x00\x00\x00\x00" end
    local sign = 0
    if f < 0 then sign = 1; f = -f end
    if f == math.huge then return string.char(sign * 128 + 127, 128, 0, 0) end
    local exp  = math.floor(math.log(f) / math.log(2))
    exp = math.max(-126, math.min(127, exp))
    local mant = f / (2.0 ^ exp) - 1.0
    if mant < 0 then mant = 0 elseif mant >= 1 then mant = mant - 1; exp = exp + 1 end
    local frac = math.min(8388607, math.floor(mant * 8388608 + 0.5))
    local v = sign * 2147483648 + (exp + 127) * 8388608 + frac
    return string.char(
        math.floor(v / 16777216) % 256,
        math.floor(v /    65536) % 256,
        math.floor(v /      256) % 256,
        v % 256)
end

local function get_u16(s, p)
    if #s < p + 1 then return 0 end
    return s:byte(p) * 256 + s:byte(p + 1)
end

local function get_i32(s, p)
    if #s < p + 3 then return 0 end
    local v = s:byte(p)*16777216 + s:byte(p+1)*65536 + s:byte(p+2)*256 + s:byte(p+3)
    if v >= 2147483648 then v = v - 4294967296 end
    return v
end

local function get_f32(s, p)
    if #s < p + 3 then return 0.0 end
    local v = s:byte(p)*16777216 + s:byte(p+1)*65536 + s:byte(p+2)*256 + s:byte(p+3)
    local sign = 1
    if v >= 2147483648 then sign = -1; v = v - 2147483648 end
    local exp  = math.floor(v / 8388608) % 256
    local frac = v % 8388608
    if exp == 0   then return 0.0 end
    if exp == 255 then return sign * math.huge end
    return sign * (1.0 + frac / 8388608) * (2.0 ^ (exp - 127))
end

-- Iridium SBD binary checksum: sum of all payload bytes, mod 65536, big-endian.
local function sbdwb_checksum(data)
    local sum = 0
    for i = 1, #data do sum = sum + data:byte(i) end
    sum = sum % 65536
    return string.char(math.floor(sum / 256), sum % 256)
end

-- ─── Telemetry packet builder ─────────────────────────────────────────────────
local function build_telemetry()
    seq_num = (seq_num + 1) % 65536

    local lat, lng, alt = 0, 0, 0
    local loc = ahrs:get_position()
    if loc then
        lat = loc:lat()     -- degrees × 1e7
        lng = loc:lng()
        alt = loc:alt()     -- cm
    end

    local vn, ve, vd = 0, 0, 0
    local vel = ahrs:get_velocity_NED()
    if vel then
        vn = math.floor(vel:x() * 100 + 0.5)   -- cm/s
        ve = math.floor(vel:y() * 100 + 0.5)
        vd = math.floor(vel:z() * 100 + 0.5)
    end

    -- ahrs angles are in radians; convert to centidegrees for the packet.
    local roll_cd  = math.floor(math.deg(ahrs:get_roll_rad())  * 100 + 0.5)
    local pitch_cd = math.floor(math.deg(ahrs:get_pitch_rad()) * 100 + 0.5)
    local yaw_cd   = math.floor(math.deg(ahrs:get_yaw_rad())   * 100 + 0.5)
    if yaw_cd < 0 then yaw_cd = yaw_cd + 36000 end     -- normalise to 0–36000

    local bat_mv = math.floor((battery:voltage(0)      or 0) * 1000 + 0.5)
    local bat_ca = math.floor((battery:current_amps(0) or 0) * 100  + 0.5)

    return string.char(0xAD, 0x12, 0x01)
        .. u16(seq_num)
        .. i32(lat) .. i32(lng) .. i32(alt)
        .. i16(vn)  .. i16(ve)  .. i16(vd)
        .. i16(roll_cd) .. i16(pitch_cd) .. i16(yaw_cd)
        .. u16(bat_mv)  .. i16(bat_ca)
        .. string.char(
            gps:num_sats(0) or 0,
            gps:status(0)   or 0,
            vehicle:get_mode() or 0,
            arming:is_armed() and 1 or 0)
end

-- ─── MT command dispatcher ────────────────────────────────────────────────────
local function handle_mt(data)
    if #data < 3 then
        gcs:send_text(4, "IRIDIUM: MT packet too short")
        return
    end
    if data:byte(1) ~= 0xAD or data:byte(2) ~= 0x34 then
        gcs:send_text(4, "IRIDIUM: MT bad magic")
        return
    end

    local cmd = data:byte(3)

    if cmd == 0x01 then
        -- Waypoint: lat(i32) lng(i32) alt_cm(i32)
        if #data < 15 then return end
        local target = Location()
        target:lat(get_i32(data, 4))
        target:lng(get_i32(data, 8))
        target:change_alt_frame(0)
        target:alt(get_i32(data, 12))
        if vehicle:set_target_location(target) then
            gcs:send_text(6, string.format("IRIDIUM: WP %.6f,%.6f alt %.1fm",
                get_i32(data,4)/1e7, get_i32(data,8)/1e7, get_i32(data,12)/100.0))
        else
            gcs:send_text(4, "IRIDIUM: WP set failed (not in GUIDED?)")
        end

    elseif cmd == 0x02 then
        -- Parameter: name(16 B null-padded) value(f32)
        if #data < 23 then return end
        local name = ""
        for i = 4, 19 do
            local c = data:byte(i)
            if c == 0 then break end
            name = name .. string.char(c)
        end
        local val = get_f32(data, 20)
        if param:set_and_save(name, val) then
            gcs:send_text(6, string.format("IRIDIUM: PARAM %s=%.4f", name, val))
        else
            gcs:send_text(4, "IRIDIUM: PARAM failed: " .. name)
        end

    elseif cmd == 0x03 then
        -- Mode change: mode(u8)
        if #data < 4 then return end
        local m = data:byte(4)
        if vehicle:set_mode(m) then
            gcs:send_text(6, string.format("IRIDIUM: mode -> %d", m))
        else
            gcs:send_text(4, string.format("IRIDIUM: mode %d failed", m))
        end

    elseif cmd == 0x04 then
        -- Arm / disarm: arm(u8, 1=arm)
        if #data < 4 then return end
        if data:byte(4) == 1 then
            arming:arm()
            gcs:send_text(6, "IRIDIUM: ARM commanded")
        else
            arming:disarm()
            gcs:send_text(6, "IRIDIUM: DISARM commanded")
        end

    elseif cmd == 0x05 then
        -- TX rate: rate_seconds(u16)
        if #data < 5 then return end
        local r = math.max(10, math.min(3600, get_u16(data, 4)))
        param:set_and_save(PARAM_TX_RATE, r)
        gcs:send_text(6, string.format("IRIDIUM: TX rate -> %ds", r))

    else
        gcs:send_text(4, string.format("IRIDIUM: unknown MT cmd 0x%02X", cmd))
    end
end

-- ─── Helpers ──────────────────────────────────────────────────────────────────
local function set_state(s)
    state = s
    state_timeout = millis() + TIMEOUT_MS
end

local function read_serial()
    local avail = uart:available()
    if avail and avail:toint() > 0 then
        local n = math.min(avail:toint(), 128)
        local chunk = uart:readstring(n)
        if chunk then rx_buf = rx_buf .. chunk end
    end
end

-- ─── Main update loop (runs every 100 ms) ────────────────────────────────────
local function update()

    -- Lazy serial init
    if uart == nil then
        uart = serial:find_serial(SERIAL_INSTANCE)
        if uart == nil then
            gcs:send_text(3, "IRIDIUM: no serial port (set SERIALx_PROTOCOL=28)")
            return update, 5000
        end
        uart:begin(BAUD_RATE)
        uart:set_flow_control(0)
        state_timeout = millis() + 5000
        gcs:send_text(6, "IRIDIUM: serial port ready")
    end

    read_serial()

    -- Timeout guard: if any active state hangs, reset cleanly.
    if state ~= S_IDLE and state ~= S_INIT and millis() > state_timeout then
        gcs:send_text(4, "IRIDIUM: timeout in state " .. tostring(state))
        last_tx_ms = millis()   -- back off before retrying
        state  = S_IDLE
        rx_buf = ""
    end

    -- ── S_INIT: send ATE0 once and wait for OK ────────────────────────────
    if state == S_INIT then
        if not init_sent then
            uart:writestring("ATE0\r")
            init_sent = true
        elseif rx_buf:find("OK") or millis() > state_timeout then
            rx_buf = ""
            state  = S_IDLE
            gcs:send_text(6, "IRIDIUM: modem ready")
        end

    -- ── S_IDLE: wait for TX interval then write MO buffer ────────────────
    elseif state == S_IDLE then
        local rate_s = param:get(PARAM_TX_RATE) or DEFAULT_TX_RATE
        if (millis() - last_tx_ms) >= (rate_s * 1000) then
            tx_payload = build_telemetry()
            uart:writestring(string.format("AT+SBDWB=%d\r", #tx_payload))
            set_state(S_WRITE_CMD)
            gcs:send_text(7, string.format("IRIDIUM: TX start (%d B)", #tx_payload))
        end

    -- ── S_WRITE_CMD: modem said READY, send binary + checksum ────────────
    elseif state == S_WRITE_CMD then
        if rx_buf:find("READY") then
            rx_buf = ""
            uart:writestring(tx_payload .. sbdwb_checksum(tx_payload))
            set_state(S_WRITE_DATA)
        elseif rx_buf:find("ERROR") then
            gcs:send_text(4, "IRIDIUM: AT+SBDWB error")
            state = S_IDLE; rx_buf = ""
        end

    -- ── S_WRITE_DATA: wait for write status digit then OK ────────────────
    elseif state == S_WRITE_DATA then
        if rx_buf:find("OK") then
            local code = tonumber(rx_buf:match("(%d+)")) or 99
            rx_buf = ""
            if code == 0 then
                uart:writestring("AT+SBDIX\r")
                set_state(S_SESSION)
            else
                local errs = {[1]="timeout",[2]="bad checksum",[3]="bad length"}
                gcs:send_text(4, "IRIDIUM: write err: " .. (errs[code] or tostring(code)))
                state = S_IDLE
            end
        end

    -- ── S_SESSION: parse +SBDIX and decide whether to receive MT ─────────
    elseif state == S_SESSION then
        local mo, _, mt_s, _, mt_len, _ =
            rx_buf:match("%+SBDIX:%s*(%d+),%s*(%d+),%s*(%d+),%s*(%d+),%s*(%d+),%s*(%d+)")
        if mo then
            local mo_n  = tonumber(mo)
            local mt_sn = tonumber(mt_s)
            mt_len_expected = tonumber(mt_len)
            rx_buf = ""

            if mo_n <= 4 then
                last_tx_ms = millis()
                gcs:send_text(6, string.format("IRIDIUM: TX ok MO=%d", mo_n))
            elseif mo_n == 32 then
                gcs:send_text(4, "IRIDIUM: no network coverage")
                last_tx_ms = millis()
            else
                gcs:send_text(4, string.format("IRIDIUM: TX failed MO=%d", mo_n))
                last_tx_ms = millis()
            end

            if mt_sn == 1 and mt_len_expected > 0 then
                uart:writestring("AT+SBDRB\r")
                set_state(S_READ)
            else
                state = S_IDLE
            end

        elseif rx_buf:find("ERROR") then
            gcs:send_text(4, "IRIDIUM: session error")
            last_tx_ms = millis()
            state = S_IDLE; rx_buf = ""
        end

    -- ── S_READ: accumulate binary MT response and verify checksum ─────────
    elseif state == S_READ then
        -- Response format on wire: [CRLF] <2-byte length> <data> <2-byte checksum> [CRLF OK CRLF]
        local needed = mt_len_expected + 4  -- length field + data + checksum
        if #rx_buf >= needed then
            -- Skip any leading CR/LF from AT response framing
            local p = 1
            while p <= #rx_buf and (rx_buf:byte(p) == 13 or rx_buf:byte(p) == 10) do
                p = p + 1
            end

            if #rx_buf >= p + mt_len_expected + 3 then
                local dlen = get_u16(rx_buf, p)
                if dlen == mt_len_expected and dlen > 0 then
                    local payload  = rx_buf:sub(p + 2, p + 1 + dlen)
                    local recv_chk = get_u16(rx_buf, p + 2 + dlen)
                    local sum = 0
                    for i = 1, #payload do sum = sum + payload:byte(i) end
                    if (sum % 65536) == recv_chk then
                        handle_mt(payload)
                    else
                        gcs:send_text(4, "IRIDIUM: MT checksum mismatch")
                    end
                elseif dlen == 0 then
                    gcs:send_text(6, "IRIDIUM: empty MT message")
                else
                    gcs:send_text(4, string.format(
                        "IRIDIUM: MT length mismatch got=%d expected=%d", dlen, mt_len_expected))
                end
            end

            state  = S_IDLE
            rx_buf = ""
        end
    end

    return update, 100
end

gcs:send_text(6, "IRIDIUM: script loaded")
return update, 2000

--[[ tweaks pitch fin trims so that when it needs to get range,
    it trims for high glide slope
    when it needs to get down with speed, it trims to less slope

    top fin untouched

    side fins, 1450, 1550 for efficiency
    ..., 1499, 1501 to get down

    starts ranged
    if <1 km to target switch to speed
    if >2 km to target switch to range
    (yes hysteresis)

    if <1km altitude switch to ranged

    only make changes on transitions in case we need to modify in flight
]]--

local last_portside_trim = 1450
local last_starboard_trim = 1550
local last_top_trim = 1500

local portside_ranged_trim  = 1450
local starboard_ranged_trim = 1550

local delta_trim = 20

local portside_speed_trim  = portside_ranged_trim + delta_trim
local starboard_speed_trim = starboard_ranged_trim - delta_trim

local switch_to_speed_m = 2000
local switch_to_range_m = 6000

local alt_switch_to_ranged = 1500

local SERVO1_TRIM = Parameter()
SERVO1_TRIM:init('SERVO1_TRIM')
local SERVO2_TRIM = Parameter()
SERVO2_TRIM:init('SERVO2_TRIM')
local SERVO3_TRIM = Parameter()
SERVO3_TRIM:init('SERVO3_TRIM')

local ranged_latch = false --default ranged mode
local speed_latch = false

GUIDED = 15 --guided mode enum
ROLLDAMP = 27 --roll damper mode enum

TRIFIN1 = 190 --trifin1 servo channel
TRIFIN2 = 191 --trifin2 servo channel
TRIFIN3 = 192 --trifin3 servo channel

--when called transitions by increment, returns true when done
local function trim_to_with_transition(final_trim_port, final_trim_star, increment)
    local current_port_trim = SERVO2_TRIM:get()
    local current_star_trim = SERVO3_TRIM:get()

    local port_done = false
    local star_done = false

    if final_trim_port ~= current_port_trim then
        local direction_port = final_trim_port - current_port_trim / math.abs(final_trim_port - current_port_trim)

        if current_port_trim + direction_port * increment > final_trim_port then
            SERVO2_TRIM:set_and_save(final_trim_port)
            port_done = true
        else
            SERVO2_TRIM:set_and_save(current_port_trim + direction_port * increment)
        end
    else
        port_done = true
    end

    if final_trim_star ~= current_star_trim then
        local direction_star = final_trim_star - current_star_trim / math.abs(final_trim_star - current_star_trim)

        if current_star_trim + direction_star * increment > final_trim_star then
            SERVO3_TRIM:set_and_save(final_trim_star)
            star_done = true
        else
            SERVO3_TRIM:set_and_save(current_star_trim + direction_star * increment)
        end
    else
        star_done = true
    end

    return port_done and star_done
end

local function set_to_ranged_mode()
    if not ranged_latch then
        if trim_to_with_transition(portside_ranged_trim, starboard_ranged_trim, 2) then
            ranged_latch = true
            speed_latch = false
        end
    end
end

local function set_to_speed_mode()
    if not speed_latch then
        if trim_to_with_transition(portside_speed_trim, starboard_speed_trim, 2) then
            ranged_latch = false
            speed_latch = true 
        end
    end
end

local function within_range(current_loc, target_loc, range_m)
    local distance = current_loc:get_distance(target_loc)
    if distance < range_m then
        return true
    end
    return false
end

local function out_of_range(current_loc, target_loc, range_m)
    local distance = current_loc:get_distance(target_loc)
    if distance > range_m then
        return true
    end
    return false
end

local function apply_stable_roll_to_current_trim()
    portside_ranged_trim = last_portside_trim
    starboard_ranged_trim = last_starboard_trim
    
    portside_speed_trim = last_portside_trim + delta_trim
    starboard_speed_trim = last_starboard_trim - delta_trim

    SERVO1_TRIM:set_and_save(last_top_trim)
    SERVO2_TRIM:set_and_save(portside_ranged_trim)
    SERVO3_TRIM:set_and_save(starboard_ranged_trim)
end

last_mode = nil
function update()
    
    local mode = vehicle:get_mode()

    if mode == ROLLDAMP then
        last_top_trim = SRV_Channels:get_output_pwm(TRIFIN1)
        last_portside_trim = SRV_Channels:get_output_pwm(TRIFIN2)
        last_starboard_trim = SRV_Channels:get_output_pwm(TRIFIN3)
        last_mode = mode
        return update, 20
    end

    if mode ~= last_mode and mode == GUIDED then
        apply_stable_roll_to_current_trim()
    end

    local current_loc = ahrs:get_location()
    local target_loc = vehicle:get_target_location()

    if current_loc ~= nil and target_loc ~= nil then
        local altitude_m = current_loc:alt() * 0.01

        if altitude_m < alt_switch_to_ranged then --takes precedence
            set_to_ranged_mode()
        else
            if within_range(current_loc, target_loc, switch_to_speed_m) then
                set_to_speed_mode()
            end
            if out_of_range(current_loc, target_loc, switch_to_range_m) then
                set_to_ranged_mode()
            end
        end
    end

    last_mode = mode

    return update, 1000
end

SERVO1_TRIM:set_and_save(1500)
set_to_ranged_mode()
return update, 1000
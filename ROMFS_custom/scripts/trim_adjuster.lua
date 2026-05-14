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

local portside_ranged_trim  = 1450
local starboard_ranged_trim = 1550

local portside_speed_trim  = 1499
local starboard_speed_trim = 1501

local switch_to_speed_m = 1500
local switch_to_range_m = 4000

local alt_switch_to_ranged = 1500

local SERVO2_TRIM = Parameter()
SERVO2_TRIM:init('SERVO2_TRIM')
local SERVO3_TRIM = Parameter()
SERVO3_TRIM:init('SERVO3_TRIM')

local ranged_latch = false --default ranged mode
local speed_latch = false

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
        if trim_to_with_transition(portside_ranged_trim, starboard_ranged_trim, 5) then
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

function update()
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

    return update, 1000
end

set_to_ranged_mode()
return update, 1000
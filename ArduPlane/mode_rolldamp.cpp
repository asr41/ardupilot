#include "mode.h"
#include "Plane.h"

bool ModeRollDamper::_enter()
{
    // setup on entry, return false to reject the mode switch
    return true;
}

void ModeRollDamper::update()
{
    // Damping gain matched to the Lua roll_damper.lua script (50 PWM units/rad/s → 450 scaled units/rad/s)
    const float ROLL_DAMP_GAIN = 900.0f;

    const float roll_rate_rads = ahrs.get_gyro().x;
    const float aileron_out    = constrain_float(ROLL_DAMP_GAIN * roll_rate_rads, -4500.0f, 4500.0f);

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron_out);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 0.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);

    plane.nav_roll_cd  = ahrs.roll_sensor;
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

/*void ModeManual::run()
{
    reset_controllers();
}*/

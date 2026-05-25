#include "mode.h"
#include "Plane.h"

bool ModeRollDamper::_enter()
{
    roll_integrator = 0.0f;
    return true;
}

void ModeRollDamper::update()
{
    const float ROLL_DAMP_GAIN = -3200.0f;  // scaled units per rad/s
    const float ROLL_PROP_GAIN = -800.0f;   // scaled units per rad
    const float ROLL_INTG_GAIN = -150.0f;   // scaled units per rad·s
    const float INTG_MAX       =  12.0f;    // integrator state limit (1000 scaled / |ROLL_INTG_GAIN|)

    const float roll_rate_rads  = ahrs.get_gyro().x;
    const float roll_angle_rads = radians(ahrs.roll_sensor * 0.01f);

    roll_integrator += roll_angle_rads * plane.G_Dt;
    roll_integrator  = constrain_float(roll_integrator, -INTG_MAX, INTG_MAX);

    const float aileron_out = constrain_float(
        ROLL_DAMP_GAIN * roll_rate_rads +
        ROLL_PROP_GAIN * roll_angle_rads +
        ROLL_INTG_GAIN * roll_integrator,
        -4500.0f, 4500.0f);

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron_out);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));

    plane.nav_roll_cd  = ahrs.roll_sensor;
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

void ModeRollDamper::run()
{
    reset_controllers();
}

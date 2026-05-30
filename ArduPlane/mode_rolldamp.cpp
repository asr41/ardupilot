#include "mode.h"
#include "Plane.h"

const AP_Param::GroupInfo ModeRollDamper::var_info[] = {
    // @Param: DAMP_GAIN
    // @DisplayName: Roll damper rate gain
    // @Description: Gain applied to roll rate (gyro.x) for roll damping. Negative values oppose roll rate.
    // @Range: -5000 0
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("DAMP_GAIN", 1, ModeRollDamper, damp_gain, -3200.0f),

    // @Param: PROP_GAIN
    // @DisplayName: Roll proportional gain
    // @Description: Gain applied to roll angle error for proportional control. Negative values drive ailerons to level the wings.
    // @Range: -3000 0
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("PROP_GAIN", 2, ModeRollDamper, prop_gain, -800.0f),

    // @Param: INTG_GAIN
    // @DisplayName: Roll integrator gain
    // @Description: Gain applied to the roll angle integrator state. Negative values provide steady-state correction.
    // @Range: -500 0
    // @Increment: 5
    // @User: Standard
    AP_GROUPINFO("INTG_GAIN", 3, ModeRollDamper, intg_gain, -200.0f),

    // @Param: INTG_MAX
    // @DisplayName: Roll integrator state limit
    // @Description: Maximum absolute value of the roll angle integrator state in rad*s.
    // @Range: 0 50
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("INTG_MAX",  4, ModeRollDamper, intg_max,  12.0f),

    AP_GROUPEND
};

ModeRollDamper::ModeRollDamper() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeRollDamper::_enter()
{
    roll_integrator = 0.0f;
    return true;
}

void ModeRollDamper::update()
{
    float airspeed_mps;
    const bool airspeed_valid = ahrs.airspeed_TAS(airspeed_mps);
    const bool active = airspeed_valid && (airspeed_mps >= 10.0f);

    const float roll_rate_rads  = ahrs.get_gyro().x;
    const float roll_angle_rads = radians(ahrs.roll_sensor * 0.01f);

    if (active) {
        roll_integrator += roll_angle_rads * plane.G_Dt;
        roll_integrator  = constrain_float(roll_integrator, -(float)intg_max, intg_max);
    } else {
        roll_integrator = 0.0f;
    }

    const float aileron_out = active ? constrain_float(
        damp_gain * roll_rate_rads +
        prop_gain * roll_angle_rads +
        intg_gain * roll_integrator,
        -4500.0f, 4500.0f) : 0.0f;

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron_out + plane.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, plane.rudder_in_expo(false));

    plane.nav_roll_cd  = ahrs.roll_sensor;
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

void ModeRollDamper::run()
{
    reset_controllers();
}

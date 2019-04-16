#include "AP_SpdHgtControl_Heli.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_SpdHgtControl_Heli::var_info[] = {

    // @Param: VEL_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VEL_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VEL_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Velocity (horizontal) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: VEL_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: VEL_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VEL_FF
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel, "VEL_", 6, AP_SpdHgtControl_Heli, AC_PID),
    
    AP_GROUPEND
};

// update speed controller
void AP_SpdHgtControl_Heli::update_speed_controller(void)
{
    
    float speed_forward, accel_target, vel_p, vel_i, vel_d;

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    speed_forward = _groundspeed_vector.x*_ahrs.cos_yaw() + _groundspeed_vector.y*_ahrs.sin_yaw();

    // calculate velocity error
    _vel_error = _vel_target - speed_forward * 100.0f;

    // call pid controller
    _pid_vel.set_input_filter_all(_vel_error);

    // get p
    vel_p = _pid_vel.get_p();

    // update i term if we have not hit the accel or throttle limits OR the i term will reduce
    // TODO: move limit handling into the PI and PID controller
//    if (!_limit.accel_xy && !_motors.limit.throttle_upper) {
        vel_i = _pid_vel.get_i();
//    } else {
//        vel_i = _pid_vel.get_i_shrink();
//    }

    // get d
    vel_d = _pid_vel.get_d();
/*
    // reset accel to current desired acceleration
     if (_flags.reset_accel_to_lean_xy) {
         _accel_target_filter.reset(Vector2f(accel_target.x, accel_target.y));
         _flags.reset_accel_to_lean_xy = false;
     }
*/

    accel_target = (vel_p + vel_i + vel_d);

    // filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(10.0f);
    _accel_target_filter.apply(accel_target, 0.0025);

    // the following section converts desired accelerations provided in lat/lon frame to roll/pitch angles

    // limit acceleration using maximum lean angles
//    float angle_max = MIN(_attitude_control.get_althold_lean_angle_max(), get_lean_angle_max_cd());
    float pitch_angle_max = 4500.0f;
    float accel_max = MIN(GRAVITY_MSS * 100.0f * tanf(ToRad(pitch_angle_max * 0.01f)), _accel_max);
//    _limit.accel_xy = limit_vector_length(_accel_target.x, _accel_target.y, accel_max);

    accel_target = constrain_float(accel_target, -accel_max, accel_max);

    // update angle targets that will be passed to stabilize controller
    _pitch_target = atanf(-accel_target/(GRAVITY_MSS * 100.0f))*(18000.0f/M_PI);

}

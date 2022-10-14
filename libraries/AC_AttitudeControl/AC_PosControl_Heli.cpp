

#include "AC_PosControl_Heli.h"

const AP_Param::GroupInfo AC_PosControl_Heli::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_PosControl, 0),

    // @Param: _ACCZ_P
    // @DisplayName: Acceleration (vertical) controller P gain
    // @Description: Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.200 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCZ_I
    // @DisplayName: Acceleration (vertical) controller I gain
    // @Description: Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCZ_IMAX
    // @DisplayName: Acceleration (vertical) controller I gain maximum
    // @Description: Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCZ_D
    // @DisplayName: Acceleration (vertical) controller D gain
    // @Description: Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCZ_FF
    // @DisplayName: Acceleration (vertical) controller feed forward
    // @Description: Acceleration (vertical) controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: _ACCZ_FLTT
    // @DisplayName: Acceleration (vertical) controller target frequency in Hz
    // @Description: Acceleration (vertical) controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_FLTE
    // @DisplayName: Acceleration (vertical) controller error frequency in Hz
    // @Description: Acceleration (vertical) controller error frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_FLTD
    // @DisplayName: Acceleration (vertical) controller derivative frequency in Hz
    // @Description: Acceleration (vertical) controller derivative frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _ACCZ_SMAX
    // @DisplayName: Accel (vertical) slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_accel_z, "_ACCZ_", 1, AC_PosControl_Heli, AC_PID),

    AP_GROUPEND
};

void AC_PosControl_Heli::set_throttle_out(float accel_z)
{
    // Acceleration Controller

    // Calculate vertical acceleration
    const float z_accel_meas = get_z_accel_cmss();

    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_z.imax()) {
        _pid_accel_z.imax(_motors.get_throttle_hover() * 1000.0f);
    }
    float thr_out;
    if (_vibe_comp_enabled) {
        thr_out = get_throttle_with_vibration_override();
    } else {
        thr_out = _pid_accel_z.update_all(_accel_target.z, z_accel_meas, (_motors.limit.throttle_lower || _motors.limit.throttle_upper)) * 0.001f;
        thr_out += _pid_accel_z.get_ff() * 0.001f;
    }
    thr_out += _motors.get_throttle_hover();

    // Actuator commands
    float throttle_cutoff_freq_hz = POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ;

    if (strcmp(_motors_heli._get_frame(), "HELI_COMPOUND") == 0) {
        if (use_ff_collective) {
            // smoothly set collective to forward flight collective
            thr_out = _motors_heli.get_fwd_flt_coll();
            throttle_cutoff_freq_hz = 0.5f;
        }
    }
    // send throttle to attitude controller with angle boost
    _attitude_control.set_throttle_out(thr_out, true, throttle_cutoff_freq_hz);
}

void AC_PosControl_Heli::input_ned_accel_rate_heading(const Vector3f& ned_accel, float heading_rate_cds, bool slew_yaw)
{

    bool print_gcs = false;
    float pitch_cd = 0.0f;

    if (current_ff_flt_coll != use_ff_collective) {
        current_ff_flt_coll = use_ff_collective;
        print_gcs = true;
    }

    if (strcmp(_motors_heli._get_frame(), "HELI_COMPOUND") == 0) {

        // rotate accelerations into body forward-right frame
        float accel_forward = ned_accel.x * _ahrs.cos_yaw() + ned_accel.y * _ahrs.sin_yaw();
        float accel_right = -ned_accel.x * _ahrs.sin_yaw() + ned_accel.y * _ahrs.cos_yaw();

        // update angle targets that will be passed to stabilize controller
        float pitch_target = accel_to_angle(-accel_forward * 0.01) * 100;
        float cos_pitch_target = cosf(pitch_target * M_PI / 18000.0f);
        float roll_target = accel_to_angle((accel_right * cos_pitch_target)*0.01) * 100;

        float accel_x_target = constrain_float(accel_forward / AC_POSCON_HELI_COMPOUND_ACCEL_X_MAX, -1.0f, 1.0f);
        _motors_heli.set_forward(accel_x_target);
        if (use_ff_collective) {
            pitch_cd = constrain_float(_accel_target.z / 500.0f, -1.0f, 1.0f) * 3000.0f;
            pitch_cd_lpf.reset(pitch_cd);
        } else {
            // smoothly set pitch_cd to zero
            pitch_cd = pitch_cd_lpf.apply(0.0f, _dt);
        }
        _attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(roll_target, pitch_cd, heading_rate_cds);

    } else {
        _attitude_control.input_thrust_vector_rate_heading(ned_accel, heading_rate_cds, slew_yaw);
    }
    if (print_gcs) {
        gcs().send_text(MAV_SEVERITY_NOTICE,"use ff coll; %s pitch_cd: %f", (use_ff_collective)?"true ":"false ", pitch_cd);
        print_gcs = false;
    }



}



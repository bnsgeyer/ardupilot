#pragma once

#include "AC_PosControl.h"
#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsHeli.h>

#define AC_POSCON_HELI_COMPOUND_ACCEL_X_MAX       5.0f

class AC_PosControl_Heli : public AC_PosControl {
public:

    /// Constructor
    AC_PosControl_Heli(AP_AHRS_View& ahrs,
                       const AP_InertialNav& inav,
                       AP_MotorsHeli& motors,
                       AC_AttitudeControl& attitude_control,
                       float dt):
        AC_PosControl(ahrs,inav,motors,attitude_control,dt),
        _motors_heli(motors)
        {
            AP_Param::setup_object_defaults(this, var_info);
            pitch_cd_lpf.set_cutoff_frequency(0.5f);

        }

    // Set use forward flight collective flag
    void set_use_ff_collective(bool ff_collective) { use_ff_collective = ff_collective; };

    void input_ned_accel_rate_heading(const Vector3f& ned_accel, float heading_rate_cds, bool slew_yaw) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // set_throttle_out - send throttle request to motors class
    void set_throttle_out(float accel_z) override;

    bool use_ff_collective;  // true if the forward flight collective is set

    bool current_ff_flt_coll;  // holds current forward flight collective status

    LowPassFilterFloat pitch_cd_lpf;

    AP_MotorsHeli& _motors_heli;

};

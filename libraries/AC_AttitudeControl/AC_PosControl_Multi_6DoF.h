#pragma once

#include "AC_PosControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

class AC_PosControl_Multi_6DoF : public AC_PosControl {
public:

    /// Constructor
    AC_PosControl_Multi_6DoF(AP_AHRS_View& ahrs,
                       const AP_InertialNav& inav,
                       AP_MotorsMulticopter& motors,
                       AC_AttitudeControl& attitude_control,
                       float dt):
        AC_PosControl(ahrs,inav,motors,attitude_control,dt)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // set_throttle_out - send throttle request to motors class
    void set_throttle_out(float accel_z) override;

};

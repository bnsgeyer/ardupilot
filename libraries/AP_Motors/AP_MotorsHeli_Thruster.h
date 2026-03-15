/// @file	AP_MotorsHeli_Thruster.h
/// @brief	Thruster Library for traditional heli
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger_config.h>
#include "AP_MotorsHeli_RSC.h"
#include "AP_Motors_Thrust_Linearization.h"

// thruster types
enum ThrusterType {
    THRUSTER_TYPE_VARIABLE_PITCH = 0,  // Variable Pitch used for tails that have a servo or motor whose ESC is connected to an output with function HeliTailRSC.
    THRUSTER_TYPE_FIXED_PITCH_CW = 1,  // Fixed Pitch rotor turning clockwise
    THRUSTER_TYPE_FIXED_PITCH_CCW = 2,  // Fixed Pitch rotor turning counterclockwise
};

enum RSCType {
    RSC_TYPE_DDFP = 0, // RSC manages spool up and down to initial throttle.
    RSC_TYPE_SETPOINT = 1,  // Used with external governor. RSC sets throttle to value set in parameter and external governor manages spool up and down.
    RSC_TYPE_THROTTLE_CURVE = 2,  // RSC manages spool up and down to initial throttle, then applies a throttle curve based on the collective stick input after spoolup.
    RSC_TYPE_GOVERNOR = 3, // RSC manages spool up and down to initial throttle, then applies a throttle curve based on the collective stick and governor maintains a target rotor speed.
};

class AP_MotorsHeli_Thruster {
public:

    AP_MotorsHeli_Thruster(AP_Motors& _motors);

    // configure - configure the thruster settings for any updated parameters
    void configure();


    float get_output(float commanded_thrust);

    // Thrust Linearization handling
    Thrust_Linearization thr_lin {*&motors};

    // var_info
    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int8  _thruster_type;                   // Thruster Type Setting

    AP_Motors& motors;

};


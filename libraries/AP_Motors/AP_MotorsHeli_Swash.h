/// @file	AP_MotorsHeli_Swash.h
/// @brief	Swashplate Library for traditional heli
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library

// servo position defaults
#define AP_MOTORS_HELI_SWASH_SERVO1_POS                       -60
#define AP_MOTORS_HELI_SWASH_SERVO2_POS                       60
#define AP_MOTORS_HELI_SWASH_SERVO3_POS                       180

// swashplate types
enum SwashPlateType {
    SWASHPLATE_TYPE_H3 = 0,
    SWASHPLATE_TYPE_H1,
    SWASHPLATE_TYPE_H3_140
};

// collective direction
enum CollectiveDirection {
    COLLECTIVE_DIRECTION_NORMAL = 0,
    COLLECTIVE_DIRECTION_REVERSED
};

class AP_MotorsHeli_Swash {
public:
    friend class AP_MotorsHeli_Single;
    friend class AP_MotorsHeli_Dual;

    AP_MotorsHeli_Swash() {};

    // calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
    void calculate_roll_pitch_collective_factors();

    // set_swash_type - sets swashplate type
    void set_swash_type(SwashPlateType swash_type) { _swash_type = swash_type; }

    // set_collective_direction - sets swashplate collective direction
    void set_collective_direction(CollectiveDirection collective_direction) { _collective_direction = collective_direction; }

    // set_phase_angle - sets swashplate phase angle
    void set_phase_angle(int16_t phase_angle) { _phase_angle = phase_angle; }

    // set_phase_angle - sets swashplate phase angle
    float get_servo_out(int8_t servo_num, float pitch, float roll, float collective);

private:
    // internal variables
    SwashPlateType  _swash_type;             // Swashplate type
    CollectiveDirection  _collective_direction;  // Collective control direction, normal or reversed
    int16_t         _phase_angle;           // Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be negative depending on mechanics.
    float           _rollFactor[3];
    float           _pitchFactor[3];
    float           _collectiveFactor[3];


};

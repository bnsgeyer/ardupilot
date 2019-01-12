/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_MotorsHeli_Swash.h"

extern const AP_HAL::HAL& hal;

// CCPM Mixers - calculate mixing scale factors by swashplate type
void AP_MotorsHeli_Swash::calculate_roll_pitch_collective_factors()
{
    if (_swash_type == SWASHPLATE_TYPE_H3) {                  //Three-Servo adjustable CCPM mixer factors
        int16_t servo1_pos = AP_MOTORS_HELI_SWASH_SERVO1_POS;
        int16_t servo2_pos = AP_MOTORS_HELI_SWASH_SERVO2_POS;
        int16_t servo3_pos = AP_MOTORS_HELI_SWASH_SERVO3_POS;
        // aileron factors
        _rollFactor[CH_1] = cosf(radians(servo1_pos + 90 - _phase_angle));
        _rollFactor[CH_2] = cosf(radians(servo2_pos + 90 - _phase_angle));
        _rollFactor[CH_3] = cosf(radians(servo3_pos + 90 - _phase_angle));

        // elevator factors
        _pitchFactor[CH_1] = cosf(radians(servo1_pos - _phase_angle));
        _pitchFactor[CH_2] = cosf(radians(servo2_pos - _phase_angle));
        _pitchFactor[CH_3] = cosf(radians(servo3_pos - _phase_angle));

        // collective factors
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;
    } else if (_swash_type == SWASHPLATE_TYPE_H3_140) {       //Three-Servo H3-140 CCPM mixer factors
        // aileron factors
        _rollFactor[CH_1] = 1;
        _rollFactor[CH_2] = -1;
        _rollFactor[CH_3] = 0;

        // elevator factors
        _pitchFactor[CH_1] = 1;
        _pitchFactor[CH_2] = 1;
        _pitchFactor[CH_3] = -1;

        // collective factors
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;
    } else {                                                              //H1 straight outputs, no mixing
        // aileron factors
        _rollFactor[CH_1] = 1;
        _rollFactor[CH_2] = 0;
        _rollFactor[CH_3] = 0;

        // elevator factors
        _pitchFactor[CH_1] = 0;
        _pitchFactor[CH_2] = 1;
        _pitchFactor[CH_3] = 0;

        // collective factors
        _collectiveFactor[CH_1] = 0;
        _collectiveFactor[CH_2] = 0;
        _collectiveFactor[CH_3] = 1;
    }
//    if (_swash_type == SWASHPLATE_TYPE_H1) {
//        servo1_out += 0.5f;
//        servo2_out += 0.5f;
//    }

}

// get_servo_out - calculates servo output
float AP_MotorsHeli_Swash::get_servo_out(int8_t CH_num, float pitch, float roll, float collective)
{
    // Collective control direction. Swash moves up for negative collective pitch, down for positive collective pitch
    if (_collective_direction == COLLECTIVE_DIRECTION_REVERSED){
        collective = 1 - collective;
    }

    float servo = ((_rollFactor[CH_num] * roll) + (_pitchFactor[CH_num] * pitch))*0.45f + _collectiveFactor[CH_num] * collective;

    // rescale from -1..1, so we can use the pwm calc that includes trim
    servo = 2*servo - 1;

    return servo;
}


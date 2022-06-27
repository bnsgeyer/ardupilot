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
/*
  multicopter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Motor.h"

namespace SITL {

/*
  class to describe a multicopter frame type
 */
class Frame {
public:
    const char *name;
    uint8_t num_motors;
    Motor *motors;

    Frame(const char *_name,
          uint8_t _num_motors,
          Motor *_motors) :
        name(_name),
        num_motors(_num_motors),
        motors(_motors) {}


    // find a frame by name
    static Frame *find_frame(const char *name);
    
    // initialise frame
    void init(float mass, float hover_throttle, float terminal_velocity, float terminal_rotation_rate);

    // calculate rotational and linear accelerations
    void calculate_forces(const Aircraft &aircraft,
                          const struct sitl_input &input,
                          Vector3f &rot_accel, Vector3f &body_accel);

    // calculate rotational and linear accelerations for x8 frame
    void calculate_forces_x8(const Aircraft &aircraft,
                          const struct sitl_input &input,
                          Vector3f &rot_accel, Vector3f &body_accel);
    
    float terminal_velocity;
    float terminal_rotation_rate;
    float thrust_scale;
    uint8_t motor_offset;

    // calculate current and voltage
    void current_and_voltage(const struct sitl_input &input, float &voltage, float &current);

    struct servos_stored {
        uint16_t servo1;
        uint16_t servo2;
        uint16_t servo3;

    };
    uint16_t _servos_delayed[3];
    ObjectBuffer<servos_stored> *servos_stored_buffer;
    void push_to_buffer(const uint16_t servos_input[16]);
    void pull_from_buffer(uint16_t servos_delayed[3]);

    // PED
    struct servo_stored_ped {
        uint16_t servo;
    };
    uint16_t _servo_delayed_ped;
    ObjectBuffer<servo_stored_ped> *servo_stored_ped_buffer;
    void push_to_buffer_ped(const uint16_t servos_input[16]);
    void pull_from_buffer_ped(uint16_t &servo_delayed);

};
}

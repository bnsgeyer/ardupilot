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
  multicopter frame simulator class
*/

#include "SIM_Frame.h"
#include <AP_Motors/AP_Motors.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

using namespace SITL;

static Motor quad_plus_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
};

static Motor quad_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

// motor order to match betaflight conventions
// See: https://fpvfrenzy.com/betaflight-motor-order/
static Motor quad_bf_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,1),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4),
};

// motor order to match betaflight conventions, reversed direction
static Motor quad_bf_x_rev_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
};

// motor order to match DJI conventions
// See: https://forum44.djicdn.com/data/attachment/forum/201711/26/172348bppvtt1ot1nrtp5j.jpg
static Motor quad_dji_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

// motor order so that test order matches motor order ("clockwise X")
static Motor quad_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
};

static Motor tiltquad_h_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, -1, 0, 0, 7, 10, -90),
};

static Motor hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_4,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor hexax_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4)
};

static Motor octa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor octa_quad_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6)
};

static Motor dodeca_hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3),
    Motor(AP_MOTORS_MOT_4,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4),
    Motor(AP_MOTORS_MOT_5,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7),
    Motor(AP_MOTORS_MOT_8, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8),
    Motor(AP_MOTORS_MOT_9,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10),
    Motor(AP_MOTORS_MOT_11, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   11),
    Motor(AP_MOTORS_MOT_12, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  12)
};

static Motor tri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2)
};

static Motor y6_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_6, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3)
};

/*
  FireflyY6 is a Y6 with front motors tiltable using servo on channel 9 (output 8)
 */
static Motor firefly_motors[] =
{
    Motor(AP_MOTORS_MOT_1, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_2,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_6, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6, -1, 0, 0, 6, 0, -90)
};

/*
  table of supported frame types. String order is important for
  partial name matching
 */
static Frame supported_frames[] =
{
    Frame("+",         4, quad_plus_motors),
    Frame("quad",      4, quad_plus_motors),
    Frame("copter",    4, quad_plus_motors),
    Frame("x",         4, quad_x_motors),
    Frame("x8",        4, quad_x_motors),
    Frame("bfxrev",    4, quad_bf_x_rev_motors),
    Frame("bfx",       4, quad_bf_x_motors),
    Frame("djix",      4, quad_dji_x_motors),
    Frame("cwx",       4, quad_cw_x_motors),
    Frame("tilthvec",  4, tiltquad_h_vectored_motors),
    Frame("hexax",     6, hexax_motors),
    Frame("hexa",      6, hexa_motors),
    Frame("octa-quad", 8, octa_quad_motors),
    Frame("octa",      8, octa_motors),
    Frame("dodeca-hexa", 12, dodeca_hexa_motors),
    Frame("tri",       3, tri_motors),
    Frame("tilttrivec",3, tilttri_vectored_motors),
    Frame("tilttri",   3, tilttri_motors),
    Frame("y6",        6, y6_motors),
    Frame("firefly",   6, firefly_motors)
};

void Frame::init(float _mass, float hover_throttle, float _terminal_velocity, float _terminal_rotation_rate)
{
    /*
       scaling from total motor power to Newtons. Allows the copter
       to hover against gravity when each motor is at hover_throttle
    */
    thrust_scale = (_mass * GRAVITY_MSS) / (num_motors * hover_throttle);

    terminal_velocity = _terminal_velocity;
    terminal_rotation_rate = _terminal_rotation_rate;
}

/*
  find a frame by name
 */
Frame *Frame::find_frame(const char *name)
{
    for (uint8_t i=0; i < ARRAY_SIZE(supported_frames); i++) {
        // do partial name matching to allow for frame variants
        if (strncasecmp(name, supported_frames[i].name, strlen(supported_frames[i].name)) == 0) {
            return &supported_frames[i];
        }
    }
    return nullptr;
}

// calculate rotational and linear accelerations
void Frame::calculate_forces(const Aircraft &aircraft,
                             const struct sitl_input &input,
                             Vector3f &rot_accel,
                             Vector3f &body_accel)
{
    Vector3f thrust; // newtons

    for (uint8_t i=0; i<num_motors; i++) {
        Vector3f mraccel, mthrust;
        motors[i].calculate_forces(input, thrust_scale, motor_offset, mraccel, mthrust);
        rot_accel += mraccel;
        thrust += mthrust;
    }

    body_accel = thrust/aircraft.gross_mass();

    if (terminal_rotation_rate > 0) {
        // rotational air resistance
        const Vector3f &gyro = aircraft.get_gyro();
        rot_accel.x -= gyro.x * radians(400.0) / terminal_rotation_rate;
        rot_accel.y -= gyro.y * radians(400.0) / terminal_rotation_rate;
        rot_accel.z -= gyro.z * radians(400.0) / terminal_rotation_rate;
    }

    if (terminal_velocity > 0) {
        // air resistance
        Vector3f air_resistance = -aircraft.get_velocity_air_ef() * (GRAVITY_MSS/terminal_velocity);
        body_accel += aircraft.get_dcm().transposed() * air_resistance;
    }

    // add some noise
    const float gyro_noise = radians(0.1);
    const float accel_noise = 0.3;
    const float noise_scale = thrust.length() / (thrust_scale * num_motors);
    rot_accel += Vector3f(aircraft.rand_normal(0, 1),
                          aircraft.rand_normal(0, 1),
                          aircraft.rand_normal(0, 1)) * gyro_noise * noise_scale;
    body_accel += Vector3f(aircraft.rand_normal(0, 1),
                           aircraft.rand_normal(0, 1),
                           aircraft.rand_normal(0, 1)) * accel_noise * noise_scale;
}


// calculate current and voltage
void Frame::current_and_voltage(const struct sitl_input &input, float &voltage, float &current)
{
    voltage = 0;
    current = 0;
    for (uint8_t i=0; i<num_motors; i++) {
        float c, v;
        motors[i].current_and_voltage(input, v, c, motor_offset);
        current += c;
        voltage += v;
    }
    // use average for voltage, total for current
    voltage /= num_motors;
}
// calculate rotational and linear accelerations for x8 frame
void Frame::calculate_forces_x8(const Aircraft &aircraft,
                             const struct sitl_input &input,
                             Vector3f &rot_accel,
                             Vector3f &body_accel)
{
// calculate rotational and linear accelerations

        const float m2ftd = 1.0f/0.3046f;
	const float m2ftn = 0.3046f;
//        const float lin_conv = 5.6f/7.3f;
/*        const float rot_xx_conv = 87.5f/157.3f;
        const float rot_yy_conv = 91.0f/148.7f;
        const float rot_zz_conv = 12.9f/18.5f;
        float Mu = 1.912f * m2ftd + rot_yy_conv;  //value has been converted from rad/s/s/ft/s to rad/s/s/m/s; value has been converted from x8+ to x8m
        float Lv = -1.644f * m2ftd + rot_xx_conv;  //value has been converted from rad/s/s/ft/s to rad/s/s/m/s; value has been converted from x8+ to x8m
        float Xu = -0.6112f * lin_conv;  // value has been converted from x8+ to x8m
        float Yv = -0.4277f * lin_conv;  // value has been converted from x8+ to x8m
        float Nr = -0.5392f + rot_zz_conv;  // value has been converted from x8+ to x8m
        float Nped = 18.51f + rot_zz_conv;  // value has been converted from x8+ to x8m
        float Zcol = -82.28f * m2ftn * lin_conv;  //value has been converted from ft/s/s to m/s/s
	float Xlon = -17.97f * m2ftn * lin_conv;  //value has been converted from ft/s/s to m/s/s; value has been converted from x8+ to x8m
	float Mlon = 148.7f + rot_yy_conv;  // value has been converted from x8+ to x8m
	float Lag = 23.50f;
	float Lead = 1.32f;
	float Ylat = 18.49f * m2ftn * lin_conv;  //value has been converted from ft/s/s to m/s/s; value has been converted from x8+ to x8m
	float Llat = 157.3f + rot_xx_conv;  // value has been converted from x8+ to x8m

*/
        float Mu = 1.63f * m2ftd;  //value has been converted from rad/s/s/ft/s to rad/s/s/m/s;
        float Lv = -1.758f * m2ftd;  //value has been converted from rad/s/s/ft/s to rad/s/s/m/s; 
        float Xu = -0.474f;
        float Yv = -0.474f;
        float Nr = -0.7521f;
        float Nped = 13.02f;
        float Zcol = -75.0f * m2ftn;  //value has been converted from ft/s/s to m/s/s
	float Xlon = -8.8f * m2ftn;  //value has been converted from ft/s/s to m/s/s;
	float Mlon = 101.02f;
	float Lag = 27.570f;
	float Lead = 1.218f;
	float Ylat = 8.8f * m2ftn;  //value has been converted from ft/s/s to m/s/s;
	float Llat = 108.63f;


        static uint64_t last_calc_us;
        static uint16_t printed = 2000;

        uint64_t now_us = AP_HAL::micros64();
        float dt = 0.0f;
        if (last_calc_us != 0) {
            dt = (now_us - last_calc_us)*1.0e-6;
        }
        last_calc_us = now_us;

	uint16_t _time_delay = 12;

	if (_time_delay == 0 || is_zero(dt)) {
	    for (uint8_t i = 0; i < 3; i++) {
		_servos_delayed[i] = input.servos[i];
            }
	} else if (servos_stored_buffer == nullptr) {
            uint16_t buffer_size = constrain_int16(_time_delay, 1, 100) * 0.001f / dt;
            servos_stored_buffer = new ObjectBuffer<servos_stored>(buffer_size);
            while (servos_stored_buffer->space() != 0) {
                push_to_buffer(input.servos);
            }
            for (uint8_t i = 0; i < 3; i++) {
                _servos_delayed[i] = input.servos[i];
            }
	} else {
            pull_from_buffer(_servos_delayed);
            push_to_buffer(input.servos);
	}

	uint16_t _time_delay_ped = 29;

	if (_time_delay_ped == 0 || is_zero(dt)) {
	    _servo_delayed_ped = input.servos[3];
	} else if (servo_stored_ped_buffer == nullptr) {
            uint16_t buffer_size = constrain_int16(_time_delay_ped, 1, 100) * 0.001f / dt;
            servo_stored_ped_buffer = new ObjectBuffer<servo_stored_ped>(buffer_size);
            while (servo_stored_ped_buffer->space() != 0) {
                push_to_buffer_ped(input.servos);
            }
            _servo_delayed_ped = input.servos[3];
	} else {
            pull_from_buffer_ped(_servo_delayed_ped);
            push_to_buffer_ped(input.servos);
	}

	static float Dlonlag;
	static float Dlatlag;
	static float Dcollag;
	static float Dpedlag;

	float _roll_in = (_servos_delayed[0] / 500.0f) - 3.0f;  // roll
	float _pitch_in = (_servos_delayed[1] / 500.0f) - 3.0f;  //pitch
	float _throttle_in = (_servos_delayed[2] / 1000.0f) - 1.0f;  // throttle
	float _yaw_in  = (_servo_delayed_ped / 500.0f) - 3.0f;  // yaw

	if (is_zero(_throttle_in)) {
	   _pitch_in = 0;
	   _roll_in = 0;
	   _yaw_in = 0;
	}

        Vector3f velocity_air_bf = aircraft.get_dcm().transposed() * aircraft.get_velocity_air_ef();

        const Vector3f &gyro = aircraft.get_gyro();
//        rotational acceleration (in rad/s/s?) in body frame
        rot_accel.x = (Lv)*(velocity_air_bf.y)+(Llat)*(Dlatlag);
        rot_accel.y = (Mu)*(velocity_air_bf.x)+(Mlon)*(Dlonlag);
        rot_accel.z = (Nr)*(gyro.z)+((Nped)-(Lag*Lead))*(Dpedlag)+(Lag*Lead)*(_yaw_in);

        float lateral_y_thrust = (Yv)*(velocity_air_bf.y)+(Ylat)*(Dlatlag);
        float lateral_x_thrust = (Xu)*(velocity_air_bf.x)+(Xlon)*Dlonlag;
	float thrust = Zcol * Dcollag - (0.1 * velocity_air_bf.z) + 0.1f * Zcol;
        body_accel = Vector3f(lateral_x_thrust, lateral_y_thrust, thrust);

	float Dlatlag_dot = (-Lag)*(Dlatlag)+(Lag)*(_roll_in);
	float Dlonlag_dot = (-Lag)*(Dlonlag)+(Lag)*(_pitch_in);
	float Dcollag_dot = (-Lag)*(Dcollag)+(Lag)*(_throttle_in);
	float Dpedlag_dot = (-Lag)*(Dpedlag)+(Lag)*(_yaw_in);

        if (printed > 0) {
            printed -= 1;
        } else {
//            gcs().send_text(MAV_SEVERITY_WARNING, "Running X8-M"); 
//        gcs().send_text(MAV_SEVERITY_WARNING, "_roll_in %f _pitch_in %f _throttle_in %f _yaw_in %f", _roll_in, _pitch_in, _throttle_in, _yaw_in);
//          gcs().send_text(MAV_SEVERITY_WARNING, "input.servos[0] %f 1 %f 2 %f 3 %f", (double)input.servos[0], (double)input.servos[1], (double)input.servos[2], (double)input.servos[3]);
//        gcs().send_text(MAV_SEVERITY_WARNING, "rot_accel.x %f y %f z %f", rot_accel.x, rot_accel.y, rot_accel.z);
//        gcs().send_text(MAV_SEVERITY_WARNING, "Dlatlag %f Dlonlag %f Dcollag %f Dpedlag %f", Dlatlag, Dlonlag, Dcollag, Dpedlag);
//        gcs().send_text(MAV_SEVERITY_WARNING, "body_accel.x %f y %f z %f Dcollag %f", body_accel.x, body_accel.y, body_accel.z, Dcollag);
//        gcs().send_text(MAV_SEVERITY_WARNING, "Servos delayed[0] %f [1] %f [2] %f ped %f", (double)_servos_delayed[0], (double)_servos_delayed[1], (double)_servos_delayed[2], (double)_servo_delayed_ped);
          printed = 2000;
        }

	Dlonlag += Dlonlag_dot * dt;
	Dlatlag += Dlatlag_dot * dt;
	Dcollag += Dcollag_dot * dt;
	Dpedlag += Dpedlag_dot * dt;

}

// push servo input to buffer
void Frame::push_to_buffer(const uint16_t servos_input[16])
{
    servos_stored sample;
    sample.servo1 = servos_input[0];
    sample.servo2 = servos_input[1];
    sample.servo3 = servos_input[2];

    servos_stored_buffer->push(sample);

}

// pull servo delay from buffer
void Frame::pull_from_buffer(uint16_t servos_delayed[3])
{
    servos_stored sample;
    if (!servos_stored_buffer->pop(sample)) {
        // no sample
        return;
    }
    servos_delayed[0] = sample.servo1;
    servos_delayed[1] = sample.servo2;
    servos_delayed[2] = sample.servo3;

}

// push servo input to buffer PED
void Frame::push_to_buffer_ped(const uint16_t servos_input[16])
{
    servo_stored_ped sample;
    sample.servo = servos_input[3];

    servo_stored_ped_buffer->push(sample);

}

// pull servo delay from buffer PED
void Frame::pull_from_buffer_ped(uint16_t &servos_delayed)
{
    servo_stored_ped sample;
    if (!servo_stored_ped_buffer->pop(sample)) {
        // no sample
        return;
    }
    servos_delayed = sample.servo;


}

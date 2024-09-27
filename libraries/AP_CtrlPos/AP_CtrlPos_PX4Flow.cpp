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
  driver for PX4Flow optical flow sensor
 */

#include "AP_CtrlPos_PX4Flow.h"

#if AP_CTRLPOS_PX4FLOW_ENABLED


#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>
#include "AP_CtrlPos.h"
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define PX4FLOW_BASE_I2C_ADDR   0x28
#define PX4FLOW_INIT_RETRIES    20      // attempt to initialise the sensor up to 10 times at startup


// detect the device
AP_CtrlPos_PX4Flow *AP_CtrlPos_PX4Flow::detect(AP_CtrlPos &_frontend)
{
    AP_CtrlPos_PX4Flow *sensor = new AP_CtrlPos_PX4Flow(_frontend);
    if (!sensor) {

        static uint8_t counter = 0;
        counter++;
        if (counter > 50) {
            counter = 0;
            gcs().send_text(MAV_SEVERITY_DEBUG, "Device Found?");
        }
        
        return nullptr;
        
    }
    if (!sensor->setup_sensor()) {
        static uint8_t counter = 0;
        counter++;
        if (counter > 50) {
            counter = 0;
            gcs().send_text(MAV_SEVERITY_DEBUG, "Device Found?");
        }
        delete sensor;
        return nullptr;

        
    }
    return sensor;
}

/*
  look for the sensor on different buses
 */
bool AP_CtrlPos_PX4Flow::scan_buses(void)
{
    bool success = false;
    uint8_t retry_attempt = 0;

    while (!success && retry_attempt < PX4FLOW_INIT_RETRIES) {
        bool all_external = (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK2);
        uint32_t bus_mask = all_external? hal.i2c_mgr->get_bus_mask() : hal.i2c_mgr->get_bus_mask_external();
        FOREACH_I2C_MASK(bus, bus_mask) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "all external %s", all_external? "true":"false");
    #ifdef HAL_OPTFLOW_PX4FLOW_I2C_BUS
            // only one bus from HAL
            if (bus != HAL_OPTFLOW_PX4FLOW_I2C_BUS) {
                continue;
            }
    #endif
            gcs().send_text(MAV_SEVERITY_DEBUG, "determined bus as %f", (double)bus);
            AP_HAL::OwnPtr<AP_HAL::Device> tdev = hal.i2c_mgr->get_device(2, PX4FLOW_BASE_I2C_ADDR);
            if (!tdev) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "no device found");
                continue;
            }
            WITH_SEMAPHORE(tdev->get_semaphore());
            dev = std::move(tdev);
            success = true;
        }
        retry_attempt++;
        if (!success) {
            hal.scheduler->delay(10);
            
        }
    }
    return success;
}

// setup the device
bool AP_CtrlPos_PX4Flow::setup_sensor(void)
{
    if (!scan_buses()) {
        return false;
    }
    // read at 10Hz HOW TO CHANGE TO 25HZ??
    dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_CtrlPos_PX4Flow::timer, void));
    gcs().send_text(MAV_SEVERITY_ALERT, "device found");

    return true;
}


// update - read latest values from sensor and fill in x,y and totals.
void AP_CtrlPos_PX4Flow::update(void)
{
}

#define STATUS_SHIFT 30
#define FIRST_SHIFT 4
#define FIRST_MASK ((1 << 20) - 1)
#define SECOND_SHIFT 8
#define SECOND_MASK ((1 << 20) - 1)
#define THIRD_SHIFT 12
#define THIRD_MASK ((1 << 20) - 1)

// timer to read sensor
void AP_CtrlPos_PX4Flow::timer(void)
{

    uint8_t raw_bytes[4];
    if (!dev->read((uint8_t *)&raw_bytes, sizeof(raw_bytes))) {
        return;
    }
    static uint16_t prnt;
    if (prnt < 1) {
        prnt = 20;
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "in Timer");
    } else {
        prnt--;
    }
    uint32_t data = (raw_bytes[0] << 24) |
                    (raw_bytes[1] << 16) |
                    (raw_bytes[2] << 8)  |
                    raw_bytes[3];

    uint32_t first_raw = (data >> FIRST_SHIFT) & FIRST_MASK;
    uint32_t second_raw = (data >>  SECOND_SHIFT) & SECOND_MASK;
    uint32_t third_raw = (data >> THIRD_SHIFT) & THIRD_MASK;

    float first_number = first_raw * 1.0f;
    float second_number = second_raw * 1.0f;
    float third_number = third_raw * 1.0f;


    struct AP_CtrlPos::CtrlPos_state state {};

    state.first_number = first_number;
    state.second_number = second_number;
    state.third_number = third_number;

    WITH_SEMAPHORE(_sem);
    _update_frontend(state);
}

#endif  // AP_OPTICALFLOW_PX4FLOW_ENABLED

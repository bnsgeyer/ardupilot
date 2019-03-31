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

#include "AP_RPM.h"
#include "RPM_PX4_PWM.h"
#include "RPM_Pin.h"
#include "RPM_SITL.h"
#include "RPM_FFT.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_RPM::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: RPM type
    // @Description: What type of RPM sensor is connected
    // @Values: 0:None,1:PX4-PWM,2:AUXPIN,3:FFT
    // @User: Standard
    AP_GROUPINFO("_TYPE",    0, AP_RPM, _type[0], 0),

    // @Param: _SCALING
    // @DisplayName: RPM scaling
    // @Description: Scaling factor between sensor reading and RPM.
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("_SCALING", 1, AP_RPM, _scaling[0], 1.0f),

    // @Param: _MAX
    // @DisplayName: Maximum RPM
    // @Description: Maximum RPM to report
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MAX", 2, AP_RPM, _maximum[0], 100000),

    // @Param: _MIN
    // @DisplayName: Minimum RPM
    // @Description: Minimum RPM to report
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MIN", 3, AP_RPM, _minimum[0], 10),

    // @Param: _MIN_QUAL
    // @DisplayName: Minimum Quality
    // @Description: Minimum data quality to be used
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_MIN_QUAL", 4, AP_RPM, _quality_min[0], 0.5),

    // @Param: _PIN
    // @DisplayName: Input pin number
    // @Description: Which pin to use
    // @Values: -1:Disabled,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6
    // @User: Standard
    AP_GROUPINFO("_PIN",    5, AP_RPM, _pin[0], 54),
    
#if RPM_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second RPM type
    // @Description: What type of RPM sensor is connected
    // @Values: 0:None,1:PX4-PWM,2:AUXPIN,3:FFT
    // @User: Advanced
    AP_GROUPINFO("2_TYPE",    10, AP_RPM, _type[1], 0),

    // @Param: 2_SCALING
    // @DisplayName: RPM scaling
    // @Description: Scaling factor between sensor reading and RPM.
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("2_SCALING", 11, AP_RPM, _scaling[1], 1.0f),
#endif

    // @Param: 2_PIN
    // @DisplayName: RPM2 input pin number
    // @Description: Which pin to use
    // @Values: -1:Disabled,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6
    // @User: Standard
    AP_GROUPINFO("2_PIN",    12, AP_RPM, _pin[1], -1),
    
    // @Param: 1_SIG_SRCE
    // @DisplayName: RPM1 Signal Source
    // @Description: Signal source for RPM estimate
    // @Values: 1:GyroX,2:GyroY,3:GyroZ,4:AccelX,5:AccelY,6:AccelZ
    // @User: Advanced
    AP_GROUPINFO("1_SIG_SRC", 13, AP_RPM, _signal_source[0], 1),

    // @Param: 1_THRSHOLD
    // @DisplayName: RPM1 FFT Threshold
    // @Description: Sets the threshold for determining first peak
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("1_THRSHLD", 14, AP_RPM, _threshold[0], 400),

    // @Param: 1_THRS_CK
    // @DisplayName: RPM1 Threshold check
    // @Description: Displays first peak amplitude
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("1_THRS_CK", 15, AP_RPM, _threshold_check[0], 0),
    
    // @Param: 2_SIG_SRCE
    // @DisplayName: RPM2 Signal Source
    // @Description: Signal source for RPM estimate
    // @Values: 1:GyroX,2:GyroY,3:GyroZ,4:AccelX,5:AccelY,6:AccelZ
    // @User: Advanced
    AP_GROUPINFO("2_SIG_SRC", 16, AP_RPM, _signal_source[1], 1),

    // @Param: 2_THRSHOLD
    // @DisplayName: RPM2 FFT Threshold
    // @Description: Sets the threshold for determining first peak
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("2_THRSHLD", 17, AP_RPM, _threshold[1], 400),

    // @Param: 2_THRS_CK
    // @DisplayName: RPM2 Threshold check
    // @Description: Displays first peak amplitude
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("2_THRS_CK", 18, AP_RPM, _threshold_check[1], 0),

    AP_GROUPEND
};

AP_RPM::AP_RPM(void) :
    num_instances(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state and drivers
    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));
}

/*
  initialise the AP_RPM class. 
 */
void AP_RPM::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<RPM_MAX_INSTANCES; i++) {
#if (CONFIG_HAL_BOARD == HAL_BOARD_PX4) || ((CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN) && (!defined(CONFIG_ARCH_BOARD_VRBRAIN_V51) && !defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)))
        {
            uint8_t type = _type[num_instances];
            uint8_t instance = num_instances;

            if (type == RPM_TYPE_PX4_PWM) {
                state[instance].instance = instance;
                drivers[instance] = new AP_RPM_PX4_PWM(*this, instance, state[instance]);
            }
        }
#endif
        {
            uint8_t type = _type[num_instances];
            uint8_t instance = num_instances;
            if (type == RPM_TYPE_PIN) {
                state[instance].instance = instance;
                drivers[instance] = new AP_RPM_Pin(*this, instance, state[instance]);
            } else if (type == RPM_TYPE_FFT) {
                state[instance].instance = instance;
                drivers[instance] = new AP_RPM_FFT(*this, instance, state[instance]);
            } else {
//        }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
//        uint8_t instance = num_instances;
//        state[instance].instance = instance;
                drivers[instance] = new AP_RPM_SITL(*this, instance, state[instance]);
#endif
            }
        }
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
    }
}

/*
  update RPM state for all instances. This should be called by main loop
 */
void AP_RPM::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (_type[i] == RPM_TYPE_NONE) {
                // allow user to disable a RPM sensor at runtime
                continue;
            }
            drivers[i]->update();
        }
    }
}
    
/*
  check if an instance is healthy
 */
bool AP_RPM::healthy(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }

    // check that data quality is above minimum required
    if (state[instance].signal_quality < _quality_min[0]) {
        return false;
    }

    return true;
}

/*
  check if an instance is activated
 */
bool AP_RPM::enabled(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }
    // if no sensor type is selected, the sensor is not activated.
    if (_type[instance] == RPM_TYPE_NONE) {
        return false;
    }
    return true;
}

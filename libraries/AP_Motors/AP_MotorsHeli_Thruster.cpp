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
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>

#include "AP_MotorsHeli_Thruster.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Thruster::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Thruster Type
    // @Description: Type of thruster used
    // @Values: 0:Variable Pitch,1:Fixed Pitch Clockwise,2:Fixed Pitch Counterclockwise
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_MotorsHeli_Thruster, _thruster_type, THRUSTER_TYPE_VARIABLE_PITCH),

    // @Param: THST_EXPO
    // @DisplayName: Thrust Curve Expo
    // @Description: Tail rotor DDFP motor thrust curve exponent (0.0 for linear to 1.0 for second order curve)
    // @Range: -1 1
    // @User: Standard

    // @Param: SPIN_MIN
    // @DisplayName: Motor Spin minimum
    // @Description: Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range.
    // @Values: 0.0:Low, 0.15:Default, 0.3:High
    // @Range: 0.0 0.3
    // @User: Standard

    // @Param: SPIN_MAX
    // @DisplayName: Motor Spin maximum
    // @Description: Point at which the thrust saturates expressed as a number from 0 to 1 in the entire output range
    // @Values: 0.9:Low, 0.95:Default, 1.0:High
    // @Range: 0.9 1.0
    // @User: Standard

    // @Param: BAT_IDX
    // @DisplayName: Battery compensation index
    // @Description: Which battery monitor should be used for doing compensation
    // @Values: 0:First battery, 1:Second battery
    // @Range: 0 15
    // @User: Standard

    // @Param: BAT_VMAX
    // @DisplayName: Battery voltage compensation maximum voltage
    // @Description: Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.2 * cell count, 0 = Disabled
    // @Range: 6 53
    // @Units: V
    // @User: Standard

    // @Param: BAT_VMIN
    // @DisplayName: Battery voltage compensation minimum voltage
    // @Description: Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.3 * cell count, 0 = Disabled
    // @Range: 6 42
    // @Units: V
    // @User: Standard
    AP_SUBGROUPINFO(thr_lin, "L_", 2, AP_MotorsHeli_Thruster, Thrust_Linearization),

    AP_GROUPEND
};

AP_MotorsHeli_Thruster::AP_MotorsHeli_Thruster(AP_Motors& _motors) :
    motors(_motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// configure - configure the thruster settings for any updated parameters
void AP_MotorsHeli_Thruster::configure()
{

}


// configure - configure the thruster settings for any updated parameters
float AP_MotorsHeli_Thruster::get_output(float commanded_thrust)
{
    thr_lin.update_lift_max_from_batt_voltage();

    return thr_lin.thrust_to_actuator(commanded_thrust);
}
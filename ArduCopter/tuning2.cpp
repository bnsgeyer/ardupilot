#include "Copter.h"

/*
 * Function to update various parameters in flight using the ch7 tuning knob
 * This should not be confused with the AutoTune feature which can bve found in control_autotune.cpp
 */

// tuning - updates parameters based on the ch7 tuning knob's position
//  should be called at 3.3hz
void Copter::tuning2()
{
    const RC_Channel *rc7 = rc().channel(CH_7);

    // exit immediately when radio failsafe is invoked or transmitter has not been turned on
    if (failsafe.radio || failsafe.radio_counter != 0 || rc7->get_radio_in() == 0) {
        return;
    }

    // exit immediately if a function is assigned to channel 7
    if ((RC_Channel::aux_func_t)rc7->option.get() != RC_Channel::AUX_FUNC::DO_NOTHING) {
        return;
    }

    const uint16_t radio_in = rc7->get_radio_in();
    float tuning_value = linear_interpolate(g2.tuning2_min, g2.tuning2_max, radio_in, rc7->get_radio_min(), rc7->get_radio_max());
    Log_Write_Parameter_Tuning("TUN2", tuning_value, g2.tuning2_min, g2.tuning2_max);

    //TODO set H_RSC_SETPOINT
}

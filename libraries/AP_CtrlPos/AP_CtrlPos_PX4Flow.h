#pragma once

#include "AP_CtrlPos.h"

#ifndef AP_CTRLPOS_PX4FLOW_ENABLED
#define AP_CTRLPOS_PX4FLOW_ENABLED AP_CTRLPOS_ENABLED
#endif

#if AP_CTRLPOS_PX4FLOW_ENABLED

#include <AP_HAL/utility/OwnPtr.h>

class AP_CtrlPos_PX4Flow : public CtrlPos_backend
{
public:
    /// constructor
    using CtrlPos_backend::CtrlPos_backend;

    CLASS_NO_COPY(AP_CtrlPos_PX4Flow);

    // init - initialise the sensor
    void init() override {}

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // detect if the sensor is available
    static AP_CtrlPos_PX4Flow *detect(AP_CtrlPos &_frontend);

private:
    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    
    // scan I2C bus addresses and buses
    bool scan_buses(void);

    // setup sensor
    bool setup_sensor(void);

    void timer(void);
};

#endif  // AP_CTRLPOS_PX4FLOW_ENABLED

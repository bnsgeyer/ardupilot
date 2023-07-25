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
#pragma once

/*
  AP_OpticalFlow backend class for ArduPilot
 */

#include "AP_CtrlPos.h"

class CtrlPos_backend
{
    friend class AP_CtrlPos;

public:
    // constructor
    CtrlPos_backend(AP_CtrlPos &_frontend);
    virtual ~CtrlPos_backend(void);

    CLASS_NO_COPY(CtrlPos_backend);

    // init - initialise sensor
    virtual void init() {}

    // read latest values from sensor and fill in x,y and totals.
    virtual void update() = 0;

    // handle optical flow mavlink messages
    virtual void handle_msg(const mavlink_message_t &msg) {}

protected:
    // access to frontend
    AP_CtrlPos &frontend;

    // update the frontend
    void _update_frontend(const struct AP_CtrlPos::CtrlPos_state &state);

    // get ADDR parameter value
    uint8_t get_address(void) const { return frontend._address; }
    
    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;
};

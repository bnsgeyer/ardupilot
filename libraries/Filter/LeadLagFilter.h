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
  LeadLag filter with settable sample rate, cutoff frequency, and alpha

 */

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <inttypes.h>
#include <AP_Param/AP_Param.h>

template <class T>
class LeadLagFilter {
public:
    // set parameters
    void init(float sample_freq_hz, float cutoff_freq_hz, float alpha);
    T apply(const T &sample);
    void reset();
    float cutoff_freq_hz() const { return _cutoff_freq_hz; }
    float sample_freq_hz() const { return _sample_freq_hz; }

    void disable(void) {
        initialised = false;
    }

protected:

    bool initialised, need_reset;
    float b0, b1, a1;
    float _cutoff_freq_hz, _sample_freq_hz, _alpha;
    T last_sample, signal1;
};

/*
  LeadLag filter enable and filter parameters
 */
class LeadLagFilterParams {
public:
    float cutoff_freq_hz(void) const { return _cutoff_freq_hz; }
    float alpha(void) const { return _alpha; }
    uint8_t enabled(void) const { return _enable; }
    void enable() { _enable.set(true); }
    
protected:
    AP_Int8 _enable;
    AP_Float _cutoff_freq_hz;
    AP_Float _alpha;
};

typedef LeadLagFilter<float> LeadLagFilterFloat;
typedef LeadLagFilter<Vector2f> LeadLagFilterVector2f;
typedef LeadLagFilter<Vector3f> LeadLagFilterVector3f;


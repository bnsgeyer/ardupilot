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

#ifndef HAL_DEBUG_BUILD
#define AP_INLINE_VECTOR_OPS
#pragma GCC optimize("O2")
#endif

#include "LeadLagFilter.h"
#include <AP_Logger/AP_Logger.h>

/*
  initialise filter
 */
template <class T>
void LeadLagFilter<T>::init(float sample_freq_hz, float cutoff_freq_hz, float alpha)
{
    // don't update if no updates required
    if (initialised &&
        is_equal(cutoff_freq_hz, _cutoff_freq_hz) &&
        is_equal(sample_freq_hz, _sample_freq_hz) &&
        is_equal(alpha, _alpha)) {
        return;
    }

    if (is_positive(cutoff_freq_hz) && is_positive(sample_freq_hz) && is_positive(alpha)) {
        float period = 1.0 / sample_freq_hz;
        float cutoff_freq_rps = 2.0 * M_PI * cutoff_freq_hz;
        b0 =  (2.0 + cutoff_freq_rps * period) * sample_freq_hz;
        b1 = (cutoff_freq_rps * period - 2.0) * sample_freq_hz;
        a1 = (2.0 - alpha * cutoff_freq_rps * period) / (2.0 + alpha * cutoff_freq_rps * period);

        _cutoff_freq_hz = cutoff_freq_hz;
        _sample_freq_hz = sample_freq_hz;
        _alpha = alpha;
        initialised = true;
    } else {
        // leave center_freq_hz at last value
        initialised = false;
    }
}

/*
  apply a new input sample, returning new output
 */
template <class T>
T LeadLagFilter<T>::apply(const T &sample)
{
    if (!initialised || need_reset) {
        // if we have not been initialised when return the input
        // sample as output and update delayed samples
        signal1 = sample;
        last_sample = sample;
        need_reset = false;
        return sample;
    }

    T output = sample*b0 + last_sample*b1 - signal1*a1;

    last_sample = sample;
    signal1 = output;

    return output;
}

template <class T>
void LeadLagFilter<T>::reset()
{
    need_reset = true;
}

/*
   instantiate template classes
 */
template class LeadLagFilter<float>;
template class LeadLagFilter<Vector2f>;
template class LeadLagFilter<Vector3f>;

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

#include <AP_AHRS/AP_AHRS.h>
#include "RPM_FFT.h"

extern const AP_HAL::HAL& hal;

/* 
   open the sensor in constructor
*/
AP_RPM_FFT::AP_RPM_FFT(AP_RPM &_ap_rpm, uint8_t _instance, AP_RPM::RPM_State &_state) :
    AP_RPM_Backend(_ap_rpm, _instance, _state)
{
    instance = _instance;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_RPM_FFT::fast_timer_update, void));
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_RPM_FFT::slow_timer_update, void));
    sem = hal.util->new_semaphore();
}


/*
  timer function called at 1kHz
*/
void AP_RPM_FFT::fast_timer_update(void)
{
    if (nsamples < ARRAY_SIZE(fft_buffer)) {
        const AP_InertialSensor &ins = AP::ins();
        uint32_t imu_sample_us = ins.get_last_update_usec();
        if (imu_sample_us == last_imu_sample_us) {
            // no new sample from INS
            return;
        }
        dt = 1.0e-6f * (imu_sample_us - last_imu_sample_us);
        last_imu_sample_us = imu_sample_us;
        const uint8_t &signal = ap_rpm._signal_source[state.instance];
        if (signal > 0 && signal < 4) {
           const Vector3f &gyro = ins.get_gyro(0);
           ins_filter_freq_hz = ins.get_gyro_filter_hz();
           // collecting data. We leave the complex component at zero
           if (signal == 1) {
               fft_buffer[nsamples] = gyro.x;
           } else if (signal == 2) {
               fft_buffer[nsamples] = gyro.y;
           } else if (signal == 3) {
               fft_buffer[nsamples] = gyro.z;
           }
        } else if (signal > 3 && signal < 7) {
           const Vector3f &accel = ins.get_accel(0);
           ins_filter_freq_hz = ins.get_accel_filter_hz();
           if (signal == 4) {
               fft_buffer[nsamples] = accel.x;
           } else if (signal == 5) {
               fft_buffer[nsamples] = accel.y;
        // inject signal to allow testing in sitl
//               fft_buffer[nsamples] = 0.2 * sinf(freq * (float)nsamples * 0.5f * 0.0025f);
           } else if (signal == 6) {
               fft_buffer[nsamples] = accel.z;
           }
        }
        nsamples += 2;

        //get FFT output faster
/*        if (nsamples > 400) {
// used to test in sitl
            freq += 12.0f;
            if (freq > 300) {
                freq = 30;
            } 
            nsamples = ARRAY_SIZE(fft_buffer);
        } */
    }
}

/*
  IO timer function called at low priority to calculate FFT
*/
void AP_RPM_FFT::slow_timer_update(void)
{
    uint16_t max_f = 0;
    float max_value = 0.0f;

    if (nsamples != ARRAY_SIZE(fft_buffer)) {
        // not ready yet
        return;
    } 
    if (arm_cfft_radix4_init_f32(&fft, (uint16_t)RPM_FFT_WIDTH, 0, 1) == ARM_MATH_SUCCESS) {
        arm_cfft_radix4_f32(&fft ,fft_buffer);

        //find first peak above a threshold
        const float &sq_threshold = ap_rpm._threshold[state.instance];
        bool first_max = false;
        bool thrsh = false;

        for (uint16_t i = 0; i < 2*RPM_FFT_WIDTH ; i+=2) { 
            float cfft = sq(fft_buffer[i]) + sq(fft_buffer[i+1]);
            // remove effect of IMU filter on fft
            const float current_freq_hz = (float)i / (2.0f * dt * (float)(RPM_FFT_WIDTH-1));
            cfft *= (1.0f + sq(current_freq_hz / (float)ins_filter_freq_hz));
            if (cfft > sq_threshold) {
                thrsh = true;
            } else {
                thrsh = false;
            }
            if (thrsh && !first_max && i != 0) {
                if (cfft > max_value) {
                    max_value = cfft;
                    max_f = i/2;
                }                
            } else if (!thrsh && max_f != 0) {
                first_max = true;
            }
        }
    }

    if (!sem->take_nonblocking()) {
        return;
    }
    new_rpm = 60.0f * (float)max_f / (dt * ((float)RPM_FFT_WIDTH - 1.0f));
    peak_mag = max_value;
    have_new_rpm = true;
    // reset nsamples
    nsamples = 0;
    memset(fft_buffer, 0, sizeof(fft_buffer));
    sem->give();
}

void AP_RPM_FFT::update(void)
{
    if (have_new_rpm && sem->take_nonblocking()) {
        const float &maximum = ap_rpm._maximum[state.instance];
        const float &minimum = ap_rpm._minimum[state.instance];
        const uint8_t &check_threshold = ap_rpm._threshold_check[state.instance];

        new_rpm *= ap_rpm._scaling[state.instance];
        if (check_threshold == 1) {
            state.rate_rpm = peak_mag;
            state.signal_quality = 0.5f;
            state.last_reading_ms = AP_HAL::millis();
        } else if ((maximum <= 0 || new_rpm <= maximum) && (new_rpm >= minimum)) {
            state.rate_rpm = new_rpm;
            state.signal_quality = 0.5f;
            state.last_reading_ms = AP_HAL::millis();
        } else {
            state.signal_quality = 0.0f;
        }        
        have_new_rpm = false;
        sem->give();
    }
}

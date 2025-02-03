#include "mode.h"
#include "Plane.h"
#include <AP_Math/control.h>

#if HAL_QUADPLANE_ENABLED

/*
 * Init and run calls for systemId, flight mode
 */

const AP_Param::GroupInfo ModeQSystemId::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which axis are being excited.  Set to non-zero to see more parameters
    // @User: Standard
    // @Values: 0:None, 1:Input Roll Angle, 2:Input Pitch Angle, 3:Input Yaw Angle, 4:Recovery Roll Angle, 5:Recovery Pitch Angle, 6:Recovery Yaw Angle, 7:Rate Roll, 8:Rate Pitch, 9:Rate Yaw, 10:Mixer Roll, 11:Mixer Pitch, 12:Mixer Yaw, 13:Mixer Thrust, 14:Measured Lateral Position, 15:Measured Longitudinal Position, 16:Measured Lateral Velocity, 17:Measured Longitudinal Velocity, 18:Input Lateral Velocity, 19:Input Longitudinal Velocity
    AP_GROUPINFO_FLAGS("_AXIS", 1, ModeQSystemId, axis, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: _MAGNITUDE
    // @DisplayName: System identification Chirp Magnitude
    // @Description: Magnitude of sweep in deg, deg/s and 0-1 for mixer outputs.
    // @User: Standard
    AP_GROUPINFO("_MAGNITUDE", 2, ModeQSystemId, waveform_magnitude, 5),

    // @Param: _F_START_HZ
    // @DisplayName: System identification Start Frequency
    // @Description: Frequency at the start of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_START_HZ", 3, ModeQSystemId, frequency_start, 0.25f),

    // @Param: _F_STOP_HZ
    // @DisplayName: System identification Stop Frequency
    // @Description: Frequency at the end of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_STOP_HZ", 4, ModeQSystemId, frequency_stop, 10),

    // @Param: _T_FADE_IN
    // @DisplayName: System identification Fade in time
    // @Description: Time to reach maximum amplitude of sweep
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_IN", 5, ModeQSystemId, time_fade_in, 5),

    // @Param: _T_REC
    // @DisplayName: System identification Total Sweep length
    // @Description: Time taken to complete the sweep
    // @Range: 0 255
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_REC", 6, ModeQSystemId, time_record, 70),

    // @Param: _T_FADE_OUT
    // @DisplayName: System identification Fade out time
    // @Description: Time to reach zero amplitude at the end of the sweep
    // @Range: 0 5
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_OUT", 7, ModeQSystemId, time_fade_out, 1),

    AP_GROUPEND
};

ModeQSystemId::ModeQSystemId(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define SYSTEM_ID_DELAY     1.0f      // time in seconds waited after system id mode change for frequency sweep injection

// systemId_init - initialise systemId controller
bool ModeQSystemId::_enter()
{
    // check if enabled
    if (axis == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No axis selected, SID_AXIS = 0");
        return false;
    }


    // reset inputs
    quadplane.set_sysid_roll_input(0.0);
    quadplane.set_sysid_pitch_input(0.0);
    quadplane.set_sysid_yaw_input(0.0);

    att_bf_feedforward = attitude_control->get_bf_feedforward();
    waveform_time = 0.0f;
    time_const_freq = 2.0f / frequency_start; // Two full cycles at the starting frequency
    systemid_state = SystemIDModeState::SYSTEMID_STATE_TESTING;
    log_subsample = 0;

    chirp_input.init(time_record, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)axis);

    plane.Log_Write_SysID_Setup(axis, waveform_magnitude, frequency_start, frequency_stop, time_fade_in, time_const_freq, time_record, time_fade_out);

    quadplane.throttle_wait = false;

    return true;
}

void ModeQSystemId::update()
{
    // set nav_roll and nav_pitch using sticks
    // Beware that QuadPlane::tailsitter_check_input (called from Plane::read_radio)
    // may alter the control_in values for roll and yaw, but not the corresponding
    // radio_in values. This means that the results for norm_input would not necessarily
    // be correct for tailsitters, so get_control_in() must be used instead.
    // normalize control_input to [-1,1]
    const float roll_input = (float)plane.channel_roll->get_control_in() / plane.channel_roll->get_range();
    const float pitch_input = (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range();

    // then scale to target angles in centidegrees
    if (plane.quadplane.tailsitter.active()) {
        // tailsitters are different
        set_tailsitter_roll_pitch(roll_input, pitch_input);
        return;
    }

    if (!plane.quadplane.option_is_set(QuadPlane::OPTION::INGORE_FW_ANGLE_LIMITS_IN_Q_MODES)) {
        // by default angles are also constrained by forward flight limits
        set_limited_roll_pitch(roll_input, pitch_input);
    } else {
        // use angle max for both roll and pitch
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
        plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
    }

}
// systemId_exit - clean up systemId controller before exiting
void ModeQSystemId::_exit()
{
    // reset the feedforward enabled parameter to the initialized state
    attitude_control->bf_feedforward(att_bf_feedforward);
    quadplane.set_sysid_roll_input(0.0f);
    quadplane.set_sysid_pitch_input(0.0f);
    quadplane.set_sysid_yaw_input(0.0f);

}

// systemId_run - runs the systemId controller
// should be called at 100hz or more
void ModeQSystemId::run()
{

    const uint32_t now = AP_HAL::millis();
//    uint32_t dt = (now - _last_loop_time_ms) * 0.001;
//    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: now=%d", (unsigned)now);
    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();

//    _last_loop_time_ms = now;
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

    plane.quadplane.assign_tilt_to_fwd_thr();

    // special check for ESC calibration in QSTABILIZE
    if (quadplane.esc_calibration != 0) {
        quadplane.run_esc_calibration();
        plane.stabilize_roll();
        plane.stabilize_pitch();
        return;
    }

    float pilot_throttle_scaled = quadplane.get_pilot_throttle();

    waveform_time += last_loop_time_s; 
    _last_loop_time_ms = now;
    waveform_sample = chirp_input.update(waveform_time - SYSTEM_ID_DELAY, waveform_magnitude);
    waveform_freq_rads = chirp_input.get_frequency_rads();

    switch ((AxisType)axis.get()) {
        case AxisType::NONE:
            systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
            gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: axis = 0");
            break;
        case AxisType::INPUT_ROLL:
            quadplane.set_sysid_roll_input(waveform_sample*100.0f);
            break;
        case AxisType::INPUT_PITCH:
            quadplane.set_sysid_pitch_input(waveform_sample*100.0f);
            break;
        case AxisType::INPUT_YAW:
            quadplane.set_sysid_yaw_input(waveform_sample*100.0f);
            break;
        case AxisType::RECOVER_ROLL:
            quadplane.set_sysid_roll_input(waveform_sample*100.0f);
            attitude_control->bf_feedforward(false);
            break;
        case AxisType::RECOVER_PITCH:
            quadplane.set_sysid_pitch_input(waveform_sample*100.0f);
            attitude_control->bf_feedforward(false);
            break;
        case AxisType::RECOVER_YAW:
            quadplane.set_sysid_yaw_input(waveform_sample*100.0f);
            attitude_control->bf_feedforward(false);
            break;
        case AxisType::RATE_ROLL:
            attitude_control->rate_bf_roll_sysid(radians(waveform_sample));
            break;
        case AxisType::RATE_PITCH:
            attitude_control->rate_bf_pitch_sysid(radians(waveform_sample));
            break;
        case AxisType::RATE_YAW:
            attitude_control->rate_bf_yaw_sysid(radians(waveform_sample));
            break;
        case AxisType::MIX_ROLL:
            attitude_control->actuator_roll_sysid(waveform_sample);
            break;
        case AxisType::MIX_PITCH:
            attitude_control->actuator_pitch_sysid(waveform_sample);
            break;
        case AxisType::MIX_YAW:
            attitude_control->actuator_yaw_sysid(waveform_sample);
            break;
        case AxisType::MIX_THROTTLE:
            pilot_throttle_scaled += waveform_sample;
            break;
    }

    quadplane.run_qsystemid(pilot_throttle_scaled);

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();

    if (log_subsample <= 0) {
        log_data();
        if (plane.should_log(MASK_LOG_ATTITUDE_FAST) && plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;
        } else {
            log_subsample = 8;
        }
    }
    log_subsample -= 1;
}

// log system id and attitude
void ModeQSystemId::log_data() const
{
    Vector3f delta_angle;
    float delta_angle_dt;
    plane.ins.get_delta_angle(delta_angle, delta_angle_dt);

    Vector3f delta_velocity;
    float delta_velocity_dt;
    plane.ins.get_delta_velocity(delta_velocity, delta_velocity_dt);

    if (is_positive(delta_angle_dt) && is_positive(delta_velocity_dt)) {
        plane.Log_Write_SysID_Data(waveform_time, waveform_sample, waveform_freq_rads / (2 * M_PI), degrees(delta_angle.x / delta_angle_dt), degrees(delta_angle.y / delta_angle_dt), degrees(delta_angle.z / delta_angle_dt), delta_velocity.x / delta_velocity_dt, delta_velocity.y / delta_velocity_dt, delta_velocity.z / delta_velocity_dt);
    }

    // Full rate logging of attitude, rate and pid loops
    plane.Log_Write_Attitude();
    quadplane.attitude_control->Write_ANG();
    quadplane.Log_Write_Rate();

}

// set the desired roll and pitch for a tailsitter
void ModeQSystemId::set_tailsitter_roll_pitch(const float roll_input, const float pitch_input)
{
    // separate limit for roll, if set
    if (plane.quadplane.tailsitter.max_roll_angle > 0) {
        // roll param is in degrees not centidegrees
        plane.nav_roll_cd = plane.quadplane.tailsitter.max_roll_angle * 100.0f * roll_input;
    } else {
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
    }

    // angle max for tailsitter pitch
    plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;

    plane.quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);
}

// set the desired roll and pitch for normal quadplanes, also limited by forward flight limits
void ModeQSystemId::set_limited_roll_pitch(const float roll_input, const float pitch_input)
{
    plane.nav_roll_cd = roll_input * MIN(plane.roll_limit_cd, plane.quadplane.aparm.angle_max);
    // pitch is further constrained by PTCH_LIM_MIN/MAX which may impose
    // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * MIN(plane.aparm.pitch_limit_max*100, plane.quadplane.aparm.angle_max);
    } else {
        plane.nav_pitch_cd = pitch_input * MIN(-plane.pitch_limit_min*100, plane.quadplane.aparm.angle_max);
    }
}

#endif

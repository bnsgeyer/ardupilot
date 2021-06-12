#include "Copter.h"

const AP_Param::GroupInfo ModeQuadsquad::var_info[] = {

    // @Param: _ALPHA
    // @DisplayName: Agressiveness of trajectory
    // @Description: More aggressive maneuver as alpha gets bigger
    // @Range: 0 3
    // @User: Standard
    AP_GROUPINFO("ALPHA", 1, ModeQuadsquad, alpha, 1.0f),

    AP_GROUPEND
};


ModeQuadsquad::ModeQuadsquad(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
 * Init and run calls for QuadSquad flight mode
 */
// quadsquad_init - initialise quadsquad controller
bool ModeQuadsquad::init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    pos_control->set_alt_target_with_slew(0.0f);
    QS_InnerRateLoop_Obj.initialize();
    engage=0;
    VeSqrSum=0;
    PeSqrSum=0;
    pNcmd=0;
    pEcmd=0;
    pDcmd=0;
    vmax=1;
    pmax=1;
    score = 0;
    trajectorycount=1;
    log_counter_qs=0;

 //   trajGen.setRp(5);
 //   trajGen.setRv(5);
//    trajGen.setManeuver(7);


    // stabilize should never be made to fail
    return true;
}


// QuadSquad_run - runs the main QuadSquad controller
// should be called at 100hz or more
void ModeQuadsquad::run()
{

     float mix_x, mix_y, mix_z, throttle;

     const Vector3f &accel = copter.ins.get_accel();
     const Vector3f &gyro = copter.ins.get_gyro();

    const Vector3f &curr_pos = inertial_nav.get_position_neu_cm();
    const Vector3f &curr_vel = inertial_nav.get_velocity_neu_cms();

    // apply simple mode transform to pilot inputs
    update_simple_mode();

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

        QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.input_lat = channel_roll->get_control_in()/float(4500); //
        QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.input_lon = channel_pitch->get_control_in()/float(4500); //
        QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.input_col = channel_throttle->get_control_in()/float(1000); //
        QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.input_ped = channel_yaw->get_control_in()/float(4500); //

        QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.vN_fpsKF = curr_vel.x / float(100); //convert cm/s to m/s
        QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.vE_fpsKF = curr_vel.y / float(100); // convert cm/s to m/s
        QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.vD_fpsKF = curr_vel.z / float(-100); // convert cm/s to m/s

         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.p_rps = gyro.x; //rad/s
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.q_rps = gyro.y; //rad/s
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.r_rps = gyro.z ; //rad/s

         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.phi_rad = QSahrs.roll_sensor / (57.3f*100.0f); //convert to rad
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.theta_rad = QSahrs.pitch_sensor / (57.3f*100.0f); //convert to rad
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.psi_rad = QSahrs.yaw_sensor / (57.3f*100.0f); //convert to rad

         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.posNorthKF = curr_pos.x / float(100);  //convert cm to m
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.posEastKF = curr_pos.y / float(100);  //convert cm to m
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.posDownKF =  curr_pos.z / float(-100);  //convert cm to m
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.Baro_Alt_m = copter.barometer.get_altitude();  //baro altitude in meters
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.Ax_mpss =  accel.x; //m/s^2
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.Ay_mpss =  accel.y; //m/s^2
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.Az_mpss =  accel.z; //m/s^2
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.mixer_in_x = motors->get_roll();
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.mixer_in_y = motors->get_pitch();
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.mixer_in_z = motors->get_yaw();
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.mixer_in_throttle = attitude_control->get_throttle_in();


     QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.CH8_flag = traj_sw; // this was set to channel 6
     QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.rc_6 = traj_sw;  // this was set to channel 6

     //     Scale Sweep Size
     if(GSInputs.ch1 == 0){
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.sweep_multiplier = 1;
     }
     else if(GSInputs.ch1 < 0){
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.sweep_multiplier = 1/abs(GSInputs.ch1);
     }
     else{
         QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.sweep_multiplier = GSInputs.ch1;
     }


//
 //    Trajectory on or off

     QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.TrajectorySwitch = traj_sw;
     alpha.set(1.0f);
//     alpha = 1.0f; // hard coding alpha
     //Scale RV
         if(alpha <= 0 || alpha > 3 ){
             QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.Rv = 2;
         }
             else {
                 QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.Rv = 1/alpha;
         }
    //Scale Rp (actually don't)
             QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.Rp = 1;



     QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.vel        = 0.0f;
     QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.psi_flight_path = 0.0f;
     QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.gamma      =  0.0f;
     QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.psi_quad   =  0.0f;

     QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.engage = engage;

     QS_InnerRateLoop_Obj.step();

     engage = 1;
	 
	 
	 // Logging
     if (log_counter_qs ++ % 10 == 0){
            AP::logger().Write("QS", "TimeUS,p,q,r,ph,th,psi,lat,lon,col,pd,mx,my,mz,mth" , "Qffffffffffffff",
                                                    AP_HAL::micros64(),
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.p_rps,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.q_rps,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.r_rps,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.phi_rad,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.theta_rad,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.psi_rad,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.input_lat,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.input_lon,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.input_col,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.input_ped,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.mixer_x,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.mixer_y,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.mixer_z,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.mixer_throttle);


            AP::logger().Write("QS2", "TimeUS,swpx,swpy,swpz,swpth,eng,phic,thtc,psic,vzc,CFz,CFvz" , "Qfffffffffff",
                                                    AP_HAL::micros64(),
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.pitch_sweep,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.roll_sweep,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.yaw_sweep,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.coll_sweep,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.engage,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.phi_cmd,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.theta_cmd,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.psi_cmd,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.vz_cmd,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.CF_Alt,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.CF_Vz);

            AP::logger().Write("QS3", "TimeUS,pN,pE,pD,vN,vE,vD,tSW,tON,pNc,pEc,pDc,psc" , "Qffffffffffff",
                                                    AP_HAL::micros64(),
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.posNorthKF,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.posEastKF,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.posDownKF,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.vN_fpsKF,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.vE_fpsKF,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.vD_fpsKF,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_U.TrajectorySwitch,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.TrajectoryON,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.Vnorth_cmd,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.Veast_cmd,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.Vdown_cmd,
                                                    (double)QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.vehheadingcmd);
                                  }



     mix_x = QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.mixer_x*.9;
     mix_y = QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.mixer_y*.9;
     mix_z = QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.mixer_z;
     throttle = QS_InnerRateLoop_Obj.QS_InnerRateLoop_Y.mixer_throttle;


     motors->set_roll(mix_x);
     motors->set_pitch(mix_y);
     motors->set_yaw(mix_z);
     motors->set_throttle(throttle);



}

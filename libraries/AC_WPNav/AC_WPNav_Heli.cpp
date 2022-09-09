#include "AC_WPNav_Heli.h"
#include <AP_Mission/AP_Mission.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_WPNav_Heli::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_WPNav, 0),

    // @Param: USE_L1_NAV
    // @DisplayName: Waypoint mission use L1 navigation with speed/height control
    // @Description: This controls if waypoint missions use L1 navigation
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    AP_GROUPINFO("USE_L1_NAV",   1, AC_WPNav_Heli, _l1_nav_use, 0),

    // @Param: L1_LOIT_RAD
    // @DisplayName: Loiter Radius for L1 Navigation
    // @Description: This sets the circle radius for the L1 loiter mode
    // @Range: 25 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("L1_LTR_RAD",   2, AC_WPNav_Heli, _loiter_radius, 50.0f),

    AP_GROUPEND
};

AC_WPNav_Heli::AC_WPNav_Heli( const AP_InertialNav& inav,
                   const AP_AHRS_View& ahrs,
                   AC_PosControl& pos_control,
                   const AC_AttitudeControl& attitude_control,
                   AP_SpdHgtControl_Heli *helispdhgtctrl,
                   AP_L1_Control_Heli& L1_controller) :
    AC_WPNav(inav,ahrs,pos_control,attitude_control),
    _helispdhgtctrl(helispdhgtctrl),
    _L1_controller(L1_controller)
{
    AP_Param::setup_object_defaults(this, var_info);
}


/// wp_and_spline_init - initialise straight line and spline waypoint controllers
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
void AC_WPNav_Heli::wp_and_spline_init(float speed_cms, Vector3f stopping_point)
{

    AC_WPNav::wp_and_spline_init(speed_cms, stopping_point);

    // if waypoint controller is not active, set origin to current position
    _prev_WP_pos = _inav.get_position_neu_cm();

    _helispdhgtctrl->init_controller();
    _helispdhgtctrl->set_desired_speed(_wp_speed_cms);
    _helispdhgtctrl->set_max_accel(_wp_accel_cmss);

}

/// set_wp_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav_Heli::set_wp_destination_loc(const Location& destination)
{

    return AC_WPNav::set_wp_destination_loc(destination);

}

/// set next destination using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav_Heli::set_wp_destination_next_loc(const Location& destination)
{

    return AC_WPNav::set_wp_destination_next_loc(destination);

}

/// set_wp_destination waypoint using position vector (distance from home in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
bool AC_WPNav_Heli::set_wp_destination(const Vector3f& destination, bool terrain_alt)
{

    // set destination wp location for L1 Navigation
    set_L1_wp_origin_and_destination(destination);

    return AC_WPNav::set_wp_destination(destination, terrain_alt);

}

/// set next destination using position vector (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
///     provide next_destination
bool AC_WPNav_Heli::set_wp_destination_next(const Vector3f& destination, bool terrain_alt)
{

    // set next wp destination
    _next_WP_pos = destination;

    return AC_WPNav::set_wp_destination_next(destination, terrain_alt);

}



/// set the L1 navigation controller origin and destination
void AC_WPNav_Heli::set_L1_wp_origin_and_destination(const Vector3f& destination)
{

    // if waypoint controller is active use the existing position target as the origin
    if ((AP_HAL::millis() - _wp_last_l1_update) < 1000) {
        _prev_WP_pos = _this_WP_pos;
    } else {
        // if waypoint controller is not active, set origin to current position
        _prev_WP_pos = _inav.get_position_neu_cm();
    }
    _this_WP_pos = destination;

//    int32_t temp_alt;
//    destination.get_alt_cm(Location::ALT_FRAME_ABOVE_ORIGIN, temp_alt);
//    _pos_control.set_alt_target(temp_alt);

    gcs().send_text(MAV_SEVERITY_INFO, "L1Nav: prev wp pos x:%f y:%f", double(_prev_WP_pos.x), double(_prev_WP_pos.y));
    gcs().send_text(MAV_SEVERITY_INFO, "L1Nav: this wp pos x:%f y:%f", double(_this_WP_pos.x), double(_this_WP_pos.y));

    _reached_l1_destination = false;

}

/// update_wpnav - run the wp controller - should be called at 100hz or higher
bool AC_WPNav_Heli::update_l1_wpnav()
{
    bool ret = true;
    _desired_speed = _wp_speed_cms;

    AP_Mission *_mission = AP_Mission::get_singleton();
    // get dt from pos controller
    float dt = _pos_control.get_dt();

    // advance the target if necessary
    if (!advance_l1_wp_target_along_track(dt)) {
        // To-Do: handle inability to advance along track (probably because of missing terrain data)
        ret = false;
    }

    // wp_speed_update - update _pos_control.set_max_speed_xy if speed change has been requested
//    wp_speed_update(dt);

    // run plane waypoint controller
    AP_Mission::Mission_Command cmd;
    cmd = _mission->get_current_nav_cmd();
    if (cmd.id == MAV_CMD_NAV_LOITER_UNLIM || cmd.id == MAV_CMD_NAV_LOITER_TIME) {
        _l1_loiter_type = 1;
    } else if (cmd.id == MAV_CMD_NAV_LOITER_TURNS) {
        _l1_loiter_type = 2;
    } else {
        _l1_loiter_type = 0;
    }

    _helispdhgtctrl->set_max_accel(_wp_accel_cmss);
    Vector3f curr_pos = _inav.get_position_neu_cm();
    float speed_forward = _inav.get_velocity_xy_cms().x*_ahrs.cos_yaw() + _inav.get_velocity_xy_cms().y*_ahrs.sin_yaw();
    Vector2f dist_vec = curr_pos.xy() - _this_WP_pos.xy();
    float dist = dist_vec.length();
    float stop_distance = 0.6f * sq(speed_forward) / _wp_accel_cmss;

    if (_l1_loiter_type > 0) {
        _L1_controller.update_loiter(_this_WP_pos.xy(), _loiter_radius, 1);
        _stopping_at_waypoint = false;
    }

    // if last nav command is waypoint then stop at waypoint
    
    if (dist < stop_distance || _stopping_at_waypoint) {
        _desired_speed = 100.0f;
    }


    _helispdhgtctrl->set_desired_speed(_desired_speed);
    _helispdhgtctrl->update_speed_controller();
    _wp_last_l1_update = AP_HAL::millis();

    return ret;
}


bool AC_WPNav_Heli::advance_l1_wp_target_along_track(float dt)
{
    static bool print_text = false;
    static uint16_t pcnt = 2000;
    // set up for altitude calculation
    int32_t temp_alt;
    int32_t wp_alt = int32_t(_this_WP_pos.z);
    int32_t prev_wp_alt = int32_t(_prev_WP_pos.z);
    Vector3f curr_pos = _inav.get_position_neu_cm();
    Vector2f prev_wp_to_curr = curr_pos.xy() - _prev_WP_pos.xy();
    float dist_to_curr_loc = prev_wp_to_curr.length();
    Vector2f prev_wp_to_next_wp = _this_WP_pos.xy() - _prev_WP_pos.xy();
    float dist_btwn_wp = prev_wp_to_next_wp.length();

    if (_l1_loiter_type == 2) {
        if (_L1_controller.reached_loiter_target()) {
            _L1_controller.loiter_angle_update();
        } else {
            _L1_controller.loiter_angle_reset();
        }
    }

    if (_l1_loiter_type > 0) {
        if (_L1_controller.reached_loiter_target()) {
            temp_alt = wp_alt;
            _reached_l1_destination = true;
        } else {
            float radius = _L1_controller.loiter_radius(_loiter_radius);
            temp_alt = (wp_alt - prev_wp_alt) * dist_to_curr_loc / (dist_btwn_wp - radius) + prev_wp_alt;
        }
    }

    Vector3f flex_this_WP_pos = _this_WP_pos;

    _L1_controller.update_waypoint(_prev_WP_pos.xy(), flex_this_WP_pos.xy());
    float acceptance_distance_m = 5.0f; // default to: if overflown - let it fly up to the point

    float speed_forward = _inav.get_velocity_xy_cms().x*_ahrs.cos_yaw() + _inav.get_velocity_xy_cms().y*_ahrs.sin_yaw();
    float stop_distance = 0.6f * sq(speed_forward) / _wp_accel_cmss;
    Vector2f currpos_to_nextwp = _this_WP_pos.xy() - curr_pos.xy();

    if (!_stopping_at_waypoint) {
        float next_ground_course_cd = get_bearing_cd(_this_WP_pos.xy(), _next_WP_pos.xy());
        // get the heading of the current leg
        float ground_course_cd = get_bearing_cd(_prev_WP_pos.xy(), _this_WP_pos.xy());

        // work out the angle we need to turn through
        float next_turn_angle = wrap_180_cd(next_ground_course_cd - ground_course_cd) * 0.01f;
        float angle_max = 45.0f;
            if (print_text) {
                gcs().send_text(MAV_SEVERITY_INFO, "L1Nav: ground_course:%f next course:%f", double(ground_course_cd * 0.01f), double(next_ground_course_cd * 0.01f));
                gcs().send_text(MAV_SEVERITY_INFO, "L1Nav: turn angle:%f", double(180.0f - fabs(next_turn_angle)));
            }

        acceptance_distance_m = _L1_controller.turn_distance(_wp_radius_cm * 0.01f, next_turn_angle);

        if (180.0f - fabs(next_turn_angle) < 90) {
            float next_turn_radius = sq(0.01f * _wp_speed_cms) * cos(angle_max / 57.3f)/(9.81f*safe_sqrt(1 - sq(cos(angle_max / 57.3))));
            float required_turn_radius = 0.01f * _wp_radius_cm * tan(0.5f * (180.0f - fabs(next_turn_angle)) / 57.3);
            if (print_text) {
                gcs().send_text(MAV_SEVERITY_INFO, "L1Nav: turn radius:%f required radius:%f", double(next_turn_radius), double(required_turn_radius));
            }
            if (required_turn_radius < next_turn_radius) {
                float turn_speed = 100.0 * safe_sqrt(required_turn_radius * 9.81f / cos(angle_max / 57.3)) * safe_sqrt(safe_sqrt(1 - sq(cos(angle_max / 57.3))));
                float brake_dist = stop_distance - 0.6f * sq(turn_speed) / _wp_accel_cmss;
                if (currpos_to_nextwp.length() < acceptance_distance_m * 100.0f + 2.0f * brake_dist) {
                    // aircraft must slow down to make turn
                    _desired_speed = turn_speed;
                }
                if (print_text) {
                    gcs().send_text(MAV_SEVERITY_INFO, "L1Nav: brake dist:%f acceptance_dist:%f", double(brake_dist), double(acceptance_distance_m * 100.0f));
                }
            }
        }
    }
    if (pcnt > 0) {
        pcnt -= 1;
        print_text = false;
    } else {
        pcnt = 2000;
        print_text = true;
    }

    if (_in_turn == true) {
        Vector2f currpos_to_turn_wp = _turn_WP_pos.xy() - curr_pos.xy();
        if (currpos_to_turn_wp.length() > _turn_accept_dist) {
            _desired_speed = _wp_speed_cms;
            _in_turn = false;
        }
    }

    if ( currpos_to_nextwp.length() <= acceptance_distance_m * 100.0f) {
        _reached_l1_destination = true;
        if (!_stopping_at_waypoint) {
            _in_turn = true;
            _turn_WP_pos.xy() = _this_WP_pos.xy();
            _turn_accept_dist = acceptance_distance_m * 100.0f;
        }
    }

        


        // have we flown past the waypoint?
//        if (location_passed_point(curr_loc, _prev_WP_loc, flex_next_WP_loc)) {
//            _reached_l1_destination = true;
//        }

    if (wp_alt == prev_wp_alt) {
        temp_alt = wp_alt;
    } else {
        temp_alt = (wp_alt - prev_wp_alt) * dist_to_curr_loc / dist_btwn_wp + prev_wp_alt;
    }
//        break;

//    }

    // don't allow temp_alt to beyond wp altitudes
    if (wp_alt > prev_wp_alt) {
        temp_alt = constrain_float(temp_alt, prev_wp_alt, wp_alt);
    } else if (prev_wp_alt > wp_alt) {
        temp_alt = constrain_float(temp_alt, wp_alt, prev_wp_alt);
    }
    _pos_control.set_alt_target_with_slew(temp_alt);

    return true;
}

// returns true if update_wpnav has been run very recently
bool AC_WPNav_Heli::is_L1_active() const
{
    return (AP_HAL::millis() - _wp_last_l1_update) < 200;
}

/// using_l1_navigation - true when using L1 navigation controller
bool AC_WPNav_Heli::use_l1_navigation()
{
//    float speed_forward = _inav.get_velocity_xy_cms().x*_ahrs.cos_yaw() + _inav.get_velocity_xy_cms().y*_ahrs.sin_yaw();
//    Vector2f curr_vel = _inav.get_velocity_xy_cms();
//    float speed_forward = curr_vel.length();
    if (_l1_nav_use == 1) {
        return true;
    } else {
        return false;
    }
}

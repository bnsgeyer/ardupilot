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


    gcs().send_text(MAV_SEVERITY_INFO, "L1Nav: initialized wp prev pos x:%f y:%f", double(_prev_WP_pos.x), double(_prev_WP_pos.y));
    //
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

    // set next wp location for L1 Navigation
    set_L1_wp_origin_and_destination(destination);

    return AC_WPNav::set_wp_destination(destination, terrain_alt);

}

/// set next destination using position vector (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
///     provide next_destination
bool AC_WPNav_Heli::set_wp_destination_next(const Vector3f& destination, bool terrain_alt)
{

//    set_L1_wp_destination_next(destination);
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

    gcs().send_text(MAV_SEVERITY_INFO, "L1Nav: set origin and dest wp x:%f y:%f", double(_this_WP_pos.x), double(_this_WP_pos.y));

    _reached_l1_destination = false;

}

/// update_wpnav - run the wp controller - should be called at 100hz or higher
bool AC_WPNav_Heli::update_l1_wpnav()
{
    bool ret = true;

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

    _helispdhgtctrl->set_max_accel(_wp_accel_cmss);
    float desired_speed = _wp_speed_cms;
    Vector3f curr_pos = _inav.get_position_neu_cm();
    float speed_forward = _inav.get_velocity_xy_cms().x*_ahrs.cos_yaw() + _inav.get_velocity_xy_cms().y*_ahrs.sin_yaw();
    Vector2f dist_vec = curr_pos.xy() - _this_WP_pos.xy();
    float dist = dist_vec.length();
    float stop_distance = 0.6f * sq(speed_forward) / _wp_accel_cmss;

    switch (cmd.id) {

    case MAV_CMD_NAV_LAND:
        _L1_controller.update_waypoint(_prev_WP_pos.xy(), _this_WP_pos.xy());
        if (dist < stop_distance || _stopping_at_waypoint) {
            desired_speed = 100.0f;
            _stopping_at_waypoint = true;
        }
        break;

    case MAV_CMD_NAV_WAYPOINT:
        _L1_controller.update_waypoint(_prev_WP_pos.xy(), _this_WP_pos.xy());
        AP_Mission::Mission_Command dummy_cmd;
        if (_mission->get_next_nav_cmd(cmd.index+1,dummy_cmd)) {
            _stopping_at_waypoint = false;
        } else {
            if (dist < stop_distance || _stopping_at_waypoint) {
                desired_speed = 100.0f;
                _stopping_at_waypoint = true;
            }
        }
        break;

/*    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
        _L1_controller.update_loiter(_next_WP_loc, _loiter_radius, 1);
        _stopping_at_waypoint = false;
        break;*/
    }

    // if last nav command is waypoint then stop at waypoint
    

    _helispdhgtctrl->set_desired_speed(desired_speed);
    _helispdhgtctrl->update_speed_controller();
    _wp_last_l1_update = AP_HAL::millis();

    return ret;
}


bool AC_WPNav_Heli::advance_l1_wp_target_along_track(float dt)
{

    // set up for altitude calculation
    int32_t temp_alt;
    int32_t wp_alt = int32_t(_this_WP_pos.z);
    int32_t prev_wp_alt = int32_t(_prev_WP_pos.z);
    Vector3f curr_pos = _inav.get_position_neu_cm();
    Vector2f prev_wp_to_curr = curr_pos.xy() - _prev_WP_pos.xy();
    float dist_to_curr_loc = prev_wp_to_curr.length();
    Vector2f prev_wp_to_next_wp = _this_WP_pos.xy() - _prev_WP_pos.xy();
    float dist_btwn_wp = prev_wp_to_next_wp.length();

    // get current waypoint nav command
    AP_Mission *_mission = AP_Mission::get_singleton();
    AP_Mission::Mission_Command cmd;
    cmd = _mission->get_current_nav_cmd();

    switch (cmd.id) {

/*    case MAV_CMD_NAV_LOITER_TURNS:
        if (_L1_controller.reached_loiter_target()) {
            _L1_controller.loiter_angle_update();
        } else {
            _L1_controller.loiter_angle_reset();
        }
        // fallthrough
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_UNLIM:
        if (_L1_controller.reached_loiter_target()) {
            temp_alt = wp_alt;
            _reached_l1_destination = true;
        } else {
            float radius = _L1_controller.loiter_radius(_loiter_radius);
            temp_alt = (wp_alt - prev_wp_alt) * dist_to_curr_loc / (dist_btwn_wp - radius) + prev_wp_alt;
        }
        break;*/

    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_WAYPOINT:
    default:

        Vector3f flex_this_WP_pos = _this_WP_pos;

        _L1_controller.update_waypoint(_prev_WP_pos.xy(), flex_this_WP_pos.xy());
        float acceptance_distance_m = 5.0f; // default to: if overflown - let it fly up to the point

        if (!_stopping_at_waypoint) {
            int32_t next_ground_course_cd = _mission->get_next_ground_course_cd(-1);
            float next_turn_angle;
            if (next_ground_course_cd == -1) {
                // the mission library can't determine a turn angle, assume 90 degrees
                next_turn_angle = 90.0f;
            } else {
                // get the heading of the current leg
                int32_t ground_course_cd = get_bearing_cd(_prev_WP_pos.xy(), _this_WP_pos.xy());

                // work out the angle we need to turn through
                next_turn_angle = wrap_180_cd(next_ground_course_cd - ground_course_cd) * 0.01f;
            }
            acceptance_distance_m = _L1_controller.turn_distance(_wp_radius_cm * 0.01f, next_turn_angle);
        }
        Vector2f currpos_to_nextwp = _this_WP_pos.xy() - curr_pos.xy();
        if ( currpos_to_nextwp.length() <= acceptance_distance_m * 100.0f) {
            _reached_l1_destination = true;
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
        break;

    }

    // don't allow temp_alt to beyond wp altitudes
    if (wp_alt > prev_wp_alt) {
        temp_alt = constrain_float(temp_alt, prev_wp_alt, wp_alt);
    } else if (prev_wp_alt > wp_alt) {
        temp_alt = constrain_float(temp_alt, wp_alt, prev_wp_alt);
    }
    _pos_control.set_alt_target_with_slew(temp_alt);

    return true;
}

/// using_l1_navigation - true when using L1 navigation controller
bool AC_WPNav_Heli::use_l1_navigation()
{
    if (_l1_nav_use == 1) {
        return true;
    } else {
        return false;
    }
}

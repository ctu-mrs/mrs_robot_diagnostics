#pragma once

#include <cstdint>
#include "mrs_robot_diagnostics/enums/uav_state.h"
#include "mrs_robot_diagnostics/enums/tracker_state.h"
#include "mrs_robot_diagnostics/enums/robot_type.h"

#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>


namespace mrs_robot_diagnostics
{
  /* parse_tracker_state() method //{ */
  tracker_state_t parse_tracker_state(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics)
  {
    if (control_manager_diagnostics->active_tracker == "NullTracker")
      return tracker_state_t::INVALID;

    switch (control_manager_diagnostics->tracker_status.state)
    {

      case mrs_msgs::TrackerStatus::STATE_INVALID:    return tracker_state_t::INVALID;
      case mrs_msgs::TrackerStatus::STATE_IDLE:       return tracker_state_t::IDLE;
      case mrs_msgs::TrackerStatus::STATE_TAKEOFF:    return tracker_state_t::TAKEOFF;
      case mrs_msgs::TrackerStatus::STATE_HOVER:      return tracker_state_t::HOVER;
      case mrs_msgs::TrackerStatus::STATE_REFERENCE:  return tracker_state_t::REFERENCE;
      case mrs_msgs::TrackerStatus::STATE_TRAJECTORY: return tracker_state_t::TRAJECTORY;
      case mrs_msgs::TrackerStatus::STATE_LAND:       return tracker_state_t::LAND;
      default:                                        return tracker_state_t::UNKNOWN;
    }
  }
  //}
  
  /* parse_robot_type() method //{ */
  robot_type_t parse_robot_type(int id)
  {

    switch (id)
    {
      case 0:  return robot_type_t::MULTIROTOR;
      case 1:  return robot_type_t::BOAT;
      default: return robot_type_t::UNKNOWN;
    }
  }
  //}

  /* parse_uav_state() method //{ */
  uav_state_t parse_uav_state(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics)
  {

    if (hw_api_status == nullptr || control_manager_diagnostics == nullptr)
      return uav_state_t::UNKNOWN;

    const bool hw_armed = hw_api_status->armed;
    // not armed
    if (!hw_armed)
      return uav_state_t::DISARMED;

    // armed, flying in manual mode
    const bool manual_mode = hw_api_status->mode == "MANUAL";
    if (control_manager_diagnostics->joystick_active)
      return uav_state_t::MANUAL;

    // armed, not flying
    const auto tracker_state = parse_tracker_state(control_manager_diagnostics);
    const bool null_tracker = tracker_state == tracker_state_t::INVALID;
    if (hw_armed && null_tracker){
      const bool offboard = hw_api_status->offboard;
      if (offboard)
        return uav_state_t::OFFBOARD;
      return uav_state_t::ARMED;
    }
    // flying using the MRS system in RC joystick mode
    if (control_manager_diagnostics->joystick_active)
      return uav_state_t::RC_MODE;

    // LandoffTracker goes into idle state when deactivating
    if (control_manager_diagnostics->active_tracker == "LandoffTracker" && tracker_state == tracker_state_t::IDLE)
      return uav_state_t::TAKEOFF;

    // unless the RC mode is active, just parse the tracker state
    switch (tracker_state)
    {
      case tracker_state_t::TAKEOFF:      return uav_state_t::TAKEOFF;
      case tracker_state_t::HOVER:        return uav_state_t::HOVER;
      case tracker_state_t::REFERENCE:    return uav_state_t::GOTO;
      case tracker_state_t::TRAJECTORY:   return uav_state_t::TRAJECTORY;
      case tracker_state_t::LAND:         return uav_state_t::LAND;
      default:                            return uav_state_t::UNKNOWN;
    }
  }
  //}
}

/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>

#include <mrs_msgs/MpcTrackerDiagnostics.h>

#include <mrs_msgs/UavStatus.h>

#include <mrs_msgs/UavDiagnostics.h>
#include <std_msgs/Float64.h>

#include <mrs_robot_diagnostics/GeneralRobotInfo.h>
#include <mrs_robot_diagnostics/StateEstimationInfo.h>
#include <mrs_robot_diagnostics/ControlInfo.h>
#include <mrs_robot_diagnostics/CollisionAvoidanceInfo.h>
#include <mrs_robot_diagnostics/UavInfo.h>

#include "mrs_robot_diagnostics/enums/uav_state.h"
#include "mrs_robot_diagnostics/enums/tracker_state.h"


//}

namespace mrs_robot_diagnostics
{

  /* class StateMonitor //{ */

  class StateMonitor : public nodelet::Nodelet
  {
  private:
    using out_diags_msg_t = mrs_msgs::UavDiagnostics;

  public:
    virtual void onInit();

  private:
    ros::NodeHandle nh_;

    enum_helpers::enum_updater<uav_state_t> uav_state_ = {"UAV STATE", uav_state_t::UNKNOWN};
    enum_helpers::enum_updater<tracker_state_t> tracker_state_ = {"TRACKER STATE", tracker_state_t::NULL_TRACKER};

    std::string _robot_name_;
    std::string _robot_type_;

    // | ---------------------- ROS subscribers --------------------- |
    std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

    // | -------------------- GeneralRobotInfo -------------------- |
    ros::Publisher pub_general_robot_info_;
    mrs_lib::SubscribeHandler<std_msgs::Bool> sh_automatic_start_can_takeoff_;
    mrs_lib::SubscribeHandler<sensor_msgs::BatteryState> sh_battery_state_;

    // | ------------------- StateEstimationInfo ------------------ |
    ros::Publisher pub_state_estimation_info_;
    mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics> sh_estimation_diagnostics_;
    mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_hw_api_gnss_;
    mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped> sh_control_manager_heading_;
    mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped> sh_hw_api_mag_heading_;

    // | ----------------------- ControlInfo ---------------------- |
    ros::Publisher pub_control_info_;
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diagnostics_;
    mrs_lib::SubscribeHandler<std_msgs::Float64> sh_control_manager_thrust_;

    // | ----------------- CollisionAvoidanceInfo ----------------- |
    ros::Publisher pub_collision_avoidance_info_;
    mrs_lib::SubscribeHandler<mrs_msgs::MpcTrackerDiagnostics> sh_mpc_tracker_diagnostics_;

    // | ------------------------- UavInfo ------------------------ |
    ros::Publisher pub_uav_info_;
    mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;
    mrs_lib::SubscribeHandler<mrs_msgs::UavStatus> sh_uav_status_;
    mrs_lib::SubscribeHandler<std_msgs::Float64> sh_mass_nominal_;
    mrs_lib::SubscribeHandler<std_msgs::Float64> sh_mass_estimate_;

    // | ----------------------- main timer ----------------------- |

    ros::Timer timer_main_;
    void timerMain(const ros::TimerEvent& event);

    // | ------------------ Additional functions ------------------ |

    tracker_state_t parse_tracker_state(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics);
    uav_state_t parse_uav_state(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics);
    mrs_robot_diagnostics::GeneralRobotInfo parse_general_robot_info(sensor_msgs::BatteryState::ConstPtr battery_state);
    mrs_robot_diagnostics::StateEstimationInfo parse_state_estimation_info(mrs_msgs::EstimationDiagnostics::ConstPtr estimation_diagnostics, mrs_msgs::Float64Stamped::ConstPtr local_heading, sensor_msgs::NavSatFix::ConstPtr global_position, mrs_msgs::Float64Stamped::ConstPtr global_heading);
    mrs_robot_diagnostics::ControlInfo parse_control_info(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics, std_msgs::Float64::ConstPtr thrust);
    mrs_robot_diagnostics::CollisionAvoidanceInfo parse_collision_avoidance_info(mrs_msgs::MpcTrackerDiagnostics::ConstPtr mpc_tracker_diagnostics);
    mrs_robot_diagnostics::UavInfo parse_uav_info(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::UavStatus::ConstPtr uav_status, std_msgs::Float64::ConstPtr mass_nominal, std_msgs::Float64::ConstPtr mass_estimate, uav_state_t uav_state);
  };
  //}

  /* onInit() //{ */

  void StateMonitor::onInit()
  {

    /* obtain node handle */
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    /* load parameters */
    mrs_lib::ParamLoader param_loader(nh_, "StateMonitor");

    std::string custom_config_path;

    param_loader.loadParam("custom_config", custom_config_path);

    if (custom_config_path != "")
    {
      param_loader.addYamlFile(custom_config_path);
    }

    param_loader.addYamlFileFromParam("config");

    param_loader.loadParam("robot_name", _robot_name_);
    param_loader.loadParam("robot_type", _robot_type_);
    const auto main_timer_rate = param_loader.loadParam2<double>("main_timer_rate");

    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("[StateMonitor]: Could not load all parameters!");
      ros::shutdown();
    }

    // | ----------------------- subscribers ---------------------- |

    tim_mgr_ = std::make_shared<mrs_lib::TimeoutManager>(nh_, ros::Rate(1.0));
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh = nh_;
    shopts.node_name = "StateMonitor";
    shopts.no_message_timeout = ros::Duration(5.0);
    shopts.timeout_manager = tim_mgr_;
    shopts.threadsafe = true;
    shopts.autostart = true;
    shopts.queue_size = 10;
    shopts.transport_hints = ros::TransportHints().tcpNoDelay();

    // | -------------------- GeneralRobotInfo -------------------- |
    pub_general_robot_info_ = nh_.advertise<mrs_robot_diagnostics::GeneralRobotInfo>("out/general_robot_info", 10);
    sh_battery_state_ = mrs_lib::SubscribeHandler<sensor_msgs::BatteryState>(shopts, "in/battery_state");
    sh_automatic_start_can_takeoff_ = mrs_lib::SubscribeHandler<std_msgs::Bool>(shopts, "in/automatic_start_can_takeoff", mrs_lib::no_timeout);

    // | ------------------- StateEstimationInfo ------------------ |
    pub_state_estimation_info_ = nh_.advertise<mrs_robot_diagnostics::StateEstimationInfo>("out/state_estimation_info", 10);
    sh_estimation_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>(shopts, "in/estimation_diagnostics");
    sh_hw_api_gnss_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "in/hw_api_gnss");
    sh_control_manager_heading_ = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "in/control_manager_heading");
    sh_hw_api_mag_heading_ = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "in/hw_api_mag_heading");

    // | ----------------------- ControlInfo ---------------------- |
    pub_control_info_ = nh_.advertise<mrs_robot_diagnostics::ControlInfo>("out/control_info", 10);
    sh_control_manager_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "in/control_manager_diagnostics");
    sh_control_manager_thrust_ = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "in/control_manager_thrust");

    // | ----------------- CollisionAvoidanceInfo ----------------- |
    pub_collision_avoidance_info_ = nh_.advertise<mrs_robot_diagnostics::CollisionAvoidanceInfo>("out/collision_avoidance_info", 10);
    sh_mpc_tracker_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::MpcTrackerDiagnostics>(shopts, "in/mpc_tracker_diagnostics");

    // | ------------------------- UavInfo ------------------------ |
    pub_uav_info_ = nh_.advertise<mrs_robot_diagnostics::UavInfo>("out/uav_info", 10);
    sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, "in/hw_api_status");
    sh_uav_status_ = mrs_lib::SubscribeHandler<mrs_msgs::UavStatus>(shopts, "in/uav_status");
    sh_mass_nominal_ = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "in/mass_nominal");
    sh_mass_estimate_ = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "in/mass_estimate");

    // | ------------------------- timers ------------------------- |

    timer_main_ = nh_.createTimer(ros::Rate(main_timer_rate), &StateMonitor::timerMain, this);

    // | --------------------- finish the init -------------------- |

    ROS_INFO("[StateMonitor]: initialized");
    ROS_INFO("[StateMonitor]: --------------------");
  }

  //}

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  /* timerMain() //{ */

  void StateMonitor::timerMain([[maybe_unused]] const ros::TimerEvent& event)
  {

    // | ---------- Output message paring and publishing ---------- |
    if (sh_battery_state_.newMsg())
      pub_general_robot_info_.publish(parse_general_robot_info(sh_battery_state_.getMsg()));

    if (sh_estimation_diagnostics_.newMsg() && sh_control_manager_heading_.newMsg() && sh_hw_api_gnss_.newMsg() && sh_hw_api_mag_heading_.newMsg())
      pub_state_estimation_info_.publish(parse_state_estimation_info(sh_estimation_diagnostics_.getMsg(), sh_control_manager_heading_.getMsg(), sh_hw_api_gnss_.getMsg(), sh_hw_api_mag_heading_.getMsg()));

    if (sh_control_manager_diagnostics_.newMsg() && sh_control_manager_thrust_.newMsg())
      pub_control_info_.publish(parse_control_info(sh_control_manager_diagnostics_.getMsg(), sh_control_manager_thrust_.getMsg()));

    if (sh_mpc_tracker_diagnostics_.newMsg())
      pub_collision_avoidance_info_.publish(parse_collision_avoidance_info(sh_mpc_tracker_diagnostics_.getMsg()));

    if (sh_hw_api_status_.newMsg() && sh_uav_status_.newMsg() && sh_mass_nominal_.hasMsg() && sh_mass_estimate_.newMsg())
      pub_uav_info_.publish(parse_uav_info(sh_hw_api_status_.getMsg(), sh_uav_status_.getMsg(), sh_mass_nominal_.getMsg(), sh_mass_estimate_.getMsg(), uav_state_.value()));

    // | -------------------- UAV state parsing ------------------- |
    const bool got_all_messages = sh_hw_api_status_.hasMsg() && sh_control_manager_diagnostics_.hasMsg();
    if (got_all_messages)
    {
      // these getMsg() must be after the newMsg() checks above (otherwise, it will never succeed)
      const auto hw_api_status = sh_hw_api_status_.getMsg();
      const auto control_manager_diagnostics = sh_control_manager_diagnostics_.getMsg();
      
      const auto new_state = parse_uav_state(hw_api_status, control_manager_diagnostics);
      uav_state_.set(new_state);
    }

    // to avoid getting timeout warnings on this latched message
    if (sh_mass_nominal_.hasMsg())
      sh_mass_nominal_.setNoMessageTimeout(mrs_lib::no_timeout);
  }

  //}

  // | -------------------- support functions ------------------- |

  /* parse_tracker_state() method //{ */
  tracker_state_t StateMonitor::parse_tracker_state(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics)
  {
    if (control_manager_diagnostics->active_tracker == "NullTracker")
      return tracker_state_t::NULL_TRACKER;
  
    switch (control_manager_diagnostics->tracker_status.state)
    {
      case mrs_msgs::TrackerStatus::STATE_IDLE:       return tracker_state_t::NULL_TRACKER;
      case mrs_msgs::TrackerStatus::STATE_TAKEOFF:    return tracker_state_t::TAKEOFF;
      case mrs_msgs::TrackerStatus::STATE_HOVER:      return tracker_state_t::HOVER;
      case mrs_msgs::TrackerStatus::STATE_REFERENCE:  return tracker_state_t::REFERENCE;
      case mrs_msgs::TrackerStatus::STATE_TRAJECTORY: return tracker_state_t::TRAJECTORY;
      case mrs_msgs::TrackerStatus::STATE_LAND:       return tracker_state_t::LAND;
      default:                                        return tracker_state_t::UNKNOWN;
    }
  }
  //}

  /* parse_uav_state() method //{ */
  uav_state_t StateMonitor::parse_uav_state(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics)
  {
    const bool hw_armed = hw_api_status->armed;
    // not armed
    if (!hw_armed)
      return uav_state_t::DISARMED;
  
    // armed, flying in manual mode
    const bool manual_mode = hw_api_status->mode == "MANUAL";
    if (control_manager_diagnostics->joystick_active)
      return uav_state_t::MANUAL;
  
    // armed, not flying
    const bool hw_loitering = hw_api_status->mode == "AUTO.LOITER";
    const auto tracker_state = parse_tracker_state(control_manager_diagnostics);
    const bool null_tracker = tracker_state == tracker_state_t::NULL_TRACKER;
    if (hw_loitering || null_tracker)
      return uav_state_t::ARMED;
  
    // flying using the MRS system in RC joystick mode
    if (control_manager_diagnostics->joystick_active)
      return uav_state_t::RC_MODE;
  
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

  /* parse_general_robot_info() method //{ */
  mrs_robot_diagnostics::GeneralRobotInfo StateMonitor::parse_general_robot_info(sensor_msgs::BatteryState::ConstPtr battery_state)
  {
    mrs_robot_diagnostics::GeneralRobotInfo msg;
    msg.robot_name = _robot_name_;
    msg.robot_type = _robot_type_;
  
    msg.battery_state.voltage = battery_state->voltage;
    msg.battery_state.percentage = battery_state->percentage;
    msg.battery_state.wh_drained = -1.0;
  
    const bool autostart_running = sh_automatic_start_can_takeoff_.getNumPublishers();
    const bool autostart_ready = sh_automatic_start_can_takeoff_.hasMsg() && sh_automatic_start_can_takeoff_.getMsg()->data;
    const bool state_ok = uav_state_.value() == uav_state_t::DISARMED;
    msg.ready_to_start = state_ok && autostart_running & autostart_ready;
    if (is_flying(uav_state_.value()))
      msg.problem_preventing_start = "UAV is in flight";
    else if (!state_ok)
      msg.problem_preventing_start = "UAV is not in DISARMED state";
    else if (!autostart_running)
      msg.problem_preventing_start = "Automatic start node is not running";
    else if (!autostart_ready)
      msg.problem_preventing_start = "Automatic start reports UAV not ready";
    return msg;
  }
  //}

/* parse_state_estimation_info() //{ */

mrs_robot_diagnostics::StateEstimationInfo StateMonitor::parse_state_estimation_info(mrs_msgs::EstimationDiagnostics::ConstPtr estimation_diagnostics, mrs_msgs::Float64Stamped::ConstPtr local_heading, sensor_msgs::NavSatFix::ConstPtr global_position, mrs_msgs::Float64Stamped::ConstPtr global_heading)
{
  mrs_robot_diagnostics::StateEstimationInfo msg;
  msg.header = estimation_diagnostics->header;

  msg.local_pose.position = estimation_diagnostics->pose.position;
  msg.local_pose.heading = local_heading->value;
  msg.above_ground_level_height = estimation_diagnostics->agl_height;

  msg.global_pose.position.x = global_position->latitude;
  msg.global_pose.position.y = global_position->longitude;
  msg.global_pose.position.z = global_position->altitude;
  msg.global_pose.heading = global_heading->value;

  msg.velocity = estimation_diagnostics->velocity;
  msg.acceleration = estimation_diagnostics->acceleration;

  if (!estimation_diagnostics->running_state_estimators.empty())
    msg.current_estimator = estimation_diagnostics->running_state_estimators.at(0);

  msg.running_estimators = estimation_diagnostics->running_state_estimators;
  msg.switchable_estimators = estimation_diagnostics->switchable_state_estimators;

  return msg;
}

//}

/* parse_control_info() //{ */

mrs_robot_diagnostics::ControlInfo StateMonitor::parse_control_info(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics, std_msgs::Float64::ConstPtr thrust)
{
  mrs_robot_diagnostics::ControlInfo msg;

  msg.active_controller = control_manager_diagnostics->active_controller;
  msg.available_controllers = control_manager_diagnostics->available_controllers;
  msg.active_tracker = control_manager_diagnostics->active_tracker;
  msg.available_trackers = control_manager_diagnostics->available_trackers;
  msg.thrust = thrust->data;

  return msg;
}

//}

/* parse_collision_avoidance_info() //{ */

mrs_robot_diagnostics::CollisionAvoidanceInfo StateMonitor::parse_collision_avoidance_info(mrs_msgs::MpcTrackerDiagnostics::ConstPtr mpc_tracker_diagnostics)
{
  mrs_robot_diagnostics::CollisionAvoidanceInfo msg;

  msg.collision_avoidance_enabled = mpc_tracker_diagnostics->collision_avoidance_active;
  msg.avoiding_collision = mpc_tracker_diagnostics->avoiding_collision;
  msg.other_robots_visible = mpc_tracker_diagnostics->avoidance_active_uavs;

  return msg;
}

//}

/* parse_uav_info() //{ */

mrs_robot_diagnostics::UavInfo StateMonitor::parse_uav_info(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::UavStatus::ConstPtr uav_status, std_msgs::Float64::ConstPtr mass_nominal, std_msgs::Float64::ConstPtr mass_estimate, uav_state_t uav_state)
{
  mrs_robot_diagnostics::UavInfo msg;

  msg.armed = hw_api_status->armed;
  msg.offboard = hw_api_status->offboard;
  msg.flight_duration = uav_status->secs_flown;
  msg.flight_state = to_string(uav_state);
  msg.mass_nominal = mass_nominal->data;
  msg.mass_estimate = mass_estimate->data;

  return msg;
}

//}

}  // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::StateMonitor, nodelet::Nodelet);

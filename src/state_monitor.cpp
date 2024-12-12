/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <Eigen/Dense>

#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <mrs_errorgraph/errorgraph.h>

#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <mrs_msgs/EstimationDiagnostics.h>
#include <sensor_msgs/NavSatFix.h>
#include <mrs_msgs/Float64Stamped.h>

#include <mrs_msgs/MpcTrackerDiagnostics.h>

#include <mrs_msgs/UavStatus.h>

#include <mrs_msgs/UavDiagnostics.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/MagneticField.h>

#include <mrs_robot_diagnostics/GeneralRobotInfo.h>
#include <mrs_robot_diagnostics/StateEstimationInfo.h>
#include <mrs_robot_diagnostics/ControlInfo.h>
#include <mrs_robot_diagnostics/CollisionAvoidanceInfo.h>
#include <mrs_robot_diagnostics/UavInfo.h>
#include <mrs_robot_diagnostics/SystemHealthInfo.h>

#include "mrs_robot_diagnostics/enums/uav_state.h"
#include "mrs_robot_diagnostics/enums/robot_type.h"
#include "mrs_robot_diagnostics/enums/tracker_state.h"
#include "mrs_robot_diagnostics/parsing_functions.h"


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

    std::mutex errorgraph_mtx_;
    mrs_errorgraph::Errorgraph errorgraph_;
    const mrs_errorgraph::node_id_t autostart_node_id_ = {"AutomaticStart", "main"};

    std::string _robot_name_;
    std::string _robot_type_;
    int _robot_type_id_;

    // Robot type mapping
    std::map<std::string, int> robot_type_id_map_ = {
        {"multirotor", 0},
        {"boat", 1},
    };

    // | ---------------------- ROS subscribers --------------------- |
    std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

    mrs_lib::SubscribeHandler<mrs_errorgraph::ErrorgraphElement> sh_errorgraph_error_msg_;

    // | -------------------- GeneralRobotInfo -------------------- |
    ros::Publisher pub_general_robot_info_;
    mrs_robot_diagnostics::GeneralRobotInfo last_general_robot_info_;
    mrs_lib::SubscribeHandler<std_msgs::Bool> sh_automatic_start_can_takeoff_;
    mrs_lib::SubscribeHandler<sensor_msgs::BatteryState> sh_battery_state_;

    // | ------------------- StateEstimationInfo ------------------ |
    ros::Publisher pub_state_estimation_info_;
    mrs_robot_diagnostics::StateEstimationInfo last_state_estimation_info_;
    mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics> sh_estimation_diagnostics_;
    mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_hw_api_gnss_;
    mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped> sh_control_manager_heading_;
    mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped> sh_hw_api_mag_heading_;

    // | ----------------------- ControlInfo ---------------------- |
    ros::Publisher pub_control_info_;
    mrs_robot_diagnostics::ControlInfo last_control_info_;
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diagnostics_;
    mrs_lib::SubscribeHandler<std_msgs::Float64> sh_control_manager_thrust_;

    // | ----------------- CollisionAvoidanceInfo ----------------- |
    ros::Publisher pub_collision_avoidance_info_;
    mrs_robot_diagnostics::CollisionAvoidanceInfo last_collision_avoidance_info_;
    mrs_lib::SubscribeHandler<mrs_msgs::MpcTrackerDiagnostics> sh_mpc_tracker_diagnostics_;

    // | ------------------------- UavInfo ------------------------ |
    ros::Publisher pub_uav_info_;
    mrs_robot_diagnostics::UavInfo last_uav_info_;
    mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;
    mrs_lib::SubscribeHandler<mrs_msgs::UavStatus> sh_uav_status_;
    mrs_lib::SubscribeHandler<std_msgs::Float64> sh_mass_nominal_;
    mrs_lib::SubscribeHandler<std_msgs::Float64> sh_mass_estimate_;

    // | -------------------- SystemHealthInfo -------------------- |
    ros::Publisher pub_system_health_info_;
    mrs_robot_diagnostics::SystemHealthInfo last_system_health_info_;
    mrs_lib::SubscribeHandler<sensor_msgs::MagneticField> sh_hw_api_magnetic_field_;

    // | ------------------------ UAV state ----------------------- |
    ros::Publisher pub_uav_state_;

    // | ----------------------- main timer ----------------------- |

    ros::Timer timer_main_;
    void timerMain(const ros::TimerEvent& event);

    // | ------------------------ Callbacks ----------------------- |
    void cbk_errorgraph_element(const mrs_errorgraph::ErrorgraphElement::ConstPtr element_msg);

    // | ------------------ Additional functions ------------------ |
    mrs_robot_diagnostics::GeneralRobotInfo parse_general_robot_info(sensor_msgs::BatteryState::ConstPtr battery_state);
    mrs_robot_diagnostics::StateEstimationInfo parse_state_estimation_info(mrs_msgs::EstimationDiagnostics::ConstPtr estimation_diagnostics,
                                                                           mrs_msgs::Float64Stamped::ConstPtr local_heading,
                                                                           sensor_msgs::NavSatFix::ConstPtr global_position,
                                                                           mrs_msgs::Float64Stamped::ConstPtr global_heading);
    mrs_robot_diagnostics::ControlInfo parse_control_info(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics,
                                                          std_msgs::Float64::ConstPtr thrust);
    mrs_robot_diagnostics::CollisionAvoidanceInfo parse_collision_avoidance_info(mrs_msgs::MpcTrackerDiagnostics::ConstPtr mpc_tracker_diagnostics);
    mrs_robot_diagnostics::UavInfo parse_uav_info(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::UavStatus::ConstPtr uav_status,
                                                  std_msgs::Float64::ConstPtr mass_nominal, std_msgs::Float64::ConstPtr mass_estimate, uav_state_t uav_state);
    mrs_robot_diagnostics::SystemHealthInfo parse_system_health_info(mrs_msgs::UavStatus::ConstPtr uav_status, sensor_msgs::NavSatFix::ConstPtr gnss,
                                                                     sensor_msgs::MagneticField::ConstPtr magnetic_field);

    mrs_robot_diagnostics::GeneralRobotInfo init_general_robot_info();
    mrs_robot_diagnostics::StateEstimationInfo init_state_estimation_info();
    mrs_robot_diagnostics::ControlInfo init_control_info();
    mrs_robot_diagnostics::CollisionAvoidanceInfo init_collision_avoidance_info();
    mrs_robot_diagnostics::UavInfo init_uav_info();
    mrs_robot_diagnostics::SystemHealthInfo init_system_health_info();
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

    // Maping the received robot type
    auto it = robot_type_id_map_.find(_robot_type_);
    if (it != robot_type_id_map_.end())
    {
      _robot_type_id_ = it->second;
    } else
    {
      ROS_ERROR_STREAM("[IROCBridge]: Unknown robot_type: " << _robot_type_);
    }

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

    sh_errorgraph_error_msg_ = mrs_lib::SubscribeHandler<mrs_errorgraph::ErrorgraphElement>(shopts, "in/errors", &StateMonitor::cbk_errorgraph_element, this);

    // | -------------------- GeneralRobotInfo -------------------- |
    pub_general_robot_info_ = nh_.advertise<mrs_robot_diagnostics::GeneralRobotInfo>("out/general_robot_info", 10);
    last_general_robot_info_ = init_general_robot_info();
    sh_battery_state_ = mrs_lib::SubscribeHandler<sensor_msgs::BatteryState>(shopts, "in/battery_state");
    sh_automatic_start_can_takeoff_ = mrs_lib::SubscribeHandler<std_msgs::Bool>(shopts, "in/automatic_start_can_takeoff", mrs_lib::no_timeout);

    // | ------------------- StateEstimationInfo ------------------ |
    pub_state_estimation_info_ = nh_.advertise<mrs_robot_diagnostics::StateEstimationInfo>("out/state_estimation_info", 10);
    last_state_estimation_info_ = init_state_estimation_info();
    sh_estimation_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>(shopts, "in/estimation_diagnostics");
    sh_hw_api_gnss_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "in/hw_api_gnss");
    sh_control_manager_heading_ = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "in/control_manager_heading");
    sh_hw_api_mag_heading_ = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "in/hw_api_mag_heading");

    // | ----------------------- ControlInfo ---------------------- |
    pub_control_info_ = nh_.advertise<mrs_robot_diagnostics::ControlInfo>("out/control_info", 10);
    last_control_info_ = init_control_info();
    sh_control_manager_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "in/control_manager_diagnostics");
    sh_control_manager_thrust_ = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "in/control_manager_thrust");

    // | ----------------- CollisionAvoidanceInfo ----------------- |
    pub_collision_avoidance_info_ = nh_.advertise<mrs_robot_diagnostics::CollisionAvoidanceInfo>("out/collision_avoidance_info", 10);
    last_collision_avoidance_info_ = init_collision_avoidance_info();
    sh_mpc_tracker_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::MpcTrackerDiagnostics>(shopts, "in/mpc_tracker_diagnostics");

    // | ------------------------- UavInfo ------------------------ |
    pub_uav_info_ = nh_.advertise<mrs_robot_diagnostics::UavInfo>("out/uav_info", 10);
    last_uav_info_ = init_uav_info();
    sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, "in/hw_api_status");
    sh_uav_status_ = mrs_lib::SubscribeHandler<mrs_msgs::UavStatus>(shopts, "in/uav_status");
    sh_mass_nominal_ = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "in/mass_nominal");
    sh_mass_estimate_ = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "in/mass_estimate");

    // | -------------------- SystemHealthInfo -------------------- |
    pub_system_health_info_ = nh_.advertise<mrs_robot_diagnostics::SystemHealthInfo>("out/system_health_info", 10);
    last_system_health_info_ = init_system_health_info();
    sh_hw_api_magnetic_field_ = mrs_lib::SubscribeHandler<sensor_msgs::MagneticField>(shopts, "in/hw_api_magnetic_field", mrs_lib::no_timeout);

    // | ------------------------ UAV state ----------------------- |
    pub_uav_state_ = nh_.advertise<mrs_robot_diagnostics::UavState>("out/uav_state", 10);

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
    const auto now = ros::Time::now();
    const bool new_uav_status = sh_uav_status_.newMsg();
    const auto uav_status = new_uav_status ? sh_uav_status_.getMsg() : nullptr;

    const bool new_hw_api_gnss = sh_hw_api_gnss_.newMsg();
    const auto hw_api_gnss = new_hw_api_gnss ? sh_hw_api_gnss_.getMsg() : nullptr;

    const bool new_battery_state = sh_hw_api_gnss_.newMsg();
    const auto battery_state = new_battery_state ? sh_battery_state_.getMsg() : nullptr;

    // | ---------- Output message parsing and publishing ---------- |
    last_general_robot_info_ = parse_general_robot_info(battery_state);

    if (sh_estimation_diagnostics_.newMsg() && sh_control_manager_heading_.newMsg() && new_hw_api_gnss && sh_hw_api_mag_heading_.newMsg())
      last_state_estimation_info_ =
          parse_state_estimation_info(sh_estimation_diagnostics_.getMsg(), sh_control_manager_heading_.getMsg(), hw_api_gnss, sh_hw_api_mag_heading_.getMsg());

    if (sh_control_manager_diagnostics_.newMsg() && sh_control_manager_thrust_.newMsg())
      last_control_info_ = parse_control_info(sh_control_manager_diagnostics_.getMsg(), sh_control_manager_thrust_.getMsg());

    if (sh_mpc_tracker_diagnostics_.newMsg())
      last_collision_avoidance_info_ = parse_collision_avoidance_info(sh_mpc_tracker_diagnostics_.getMsg());

    if (sh_hw_api_status_.newMsg() && new_uav_status && sh_mass_nominal_.hasMsg() && sh_mass_estimate_.newMsg())
      last_uav_info_ = parse_uav_info(sh_hw_api_status_.getMsg(), uav_status, sh_mass_nominal_.getMsg(), sh_mass_estimate_.getMsg(), uav_state_.value());

    const auto hw_api_magnetic_field = sh_hw_api_magnetic_field_.hasMsg() ? sh_hw_api_magnetic_field_.getMsg() : nullptr;
    if (new_uav_status && new_hw_api_gnss)
      last_system_health_info_ = parse_system_health_info(uav_status, hw_api_gnss, hw_api_magnetic_field);

    pub_general_robot_info_.publish(last_general_robot_info_);
    pub_state_estimation_info_.publish(last_state_estimation_info_);
    pub_control_info_.publish(last_control_info_);
    pub_collision_avoidance_info_.publish(last_collision_avoidance_info_);
    pub_uav_info_.publish(last_uav_info_);
    pub_system_health_info_.publish(last_system_health_info_);

    mrs_robot_diagnostics::UavState uav_state_msg;
    uav_state_msg.stamp = now;
    uav_state_msg.state = to_ros(uav_state_.value());
    pub_uav_state_.publish(uav_state_msg);

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

  // | ------------------------ callbacks ----------------------- |
  void StateMonitor::cbk_errorgraph_element(const mrs_errorgraph::ErrorgraphElement::ConstPtr element_msg)
  {
    std::scoped_lock lck(errorgraph_mtx_);
    errorgraph_.add_element_from_msg(*element_msg);
  }

  // | -------------------- support functions ------------------- |

  /* cov2eigen() //{ */
  Eigen::Matrix3d cov2eigen(const boost::array<double, 9>& msg_cov)
  {
    Eigen::Matrix3d cov;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        cov(r, c) = msg_cov.at(r + 3 * c);
    return cov;
  }

  // | --------------------- Parsing methods -------------------- |

  /* parse_general_robot_info() method //{ */
  mrs_robot_diagnostics::GeneralRobotInfo StateMonitor::parse_general_robot_info(sensor_msgs::BatteryState::ConstPtr battery_state)
  {
    mrs_robot_diagnostics::GeneralRobotInfo msg;
    msg.stamp = ros::Time::now();
    msg.robot_name = _robot_name_;
    msg.robot_type = _robot_type_id_;

    if (battery_state == nullptr)
    {
      msg.battery_state= last_general_robot_info_.battery_state;
    }else{
      msg.battery_state.voltage = battery_state->voltage;
      msg.battery_state.percentage = battery_state->percentage;
      msg.battery_state.wh_drained = -1.0;
    }


    const bool autostart_running = sh_automatic_start_can_takeoff_.getNumPublishers();
    const bool autostart_ready = sh_automatic_start_can_takeoff_.hasMsg() && sh_automatic_start_can_takeoff_.getMsg()->data;
    const bool state_ok = uav_state_.value() == uav_state_t::DISARMED;
    msg.ready_to_start = state_ok && autostart_running && autostart_ready;

    if (is_flying(uav_state_.value()))
    {
      msg.problems_preventing_start.emplace_back("UAV is in flight");
    }
    else if (!state_ok)
    {
      msg.problems_preventing_start.emplace_back("UAV is not in DISARMED state");
    }
    else if (!autostart_running)
    {
      msg.problems_preventing_start.emplace_back("Automatic start node is not running");
    }
    else if (!autostart_ready)
    {
      // if autostart reports that it is not ready, try to find the root cause
      std::scoped_lock lck(errorgraph_mtx_);
      const auto dependency_roots = errorgraph_.find_dependency_roots(autostart_node_id_);
      if (dependency_roots.empty())
      {
        msg.problems_preventing_start.emplace_back("Automatic start reports UAV not ready");
      }
      else
      {
        for (const auto& root : dependency_roots)
          for (const auto& error : root->errors)
            msg.problems_preventing_start.push_back(error.type);
      }
    }

    { // find all errors
      std::scoped_lock lck(errorgraph_mtx_);
      const auto error_roots = errorgraph_.find_roots();
      if (!error_roots.empty())
      {
        for (const auto& root : error_roots){
          for (const auto& error : root->errors){
            msg.errors.push_back(error.type);
          }
        }
      } 
    }
    return msg;
  }
  //}

  /* parse_state_estimation_info() //{ */

  mrs_robot_diagnostics::StateEstimationInfo StateMonitor::parse_state_estimation_info(mrs_msgs::EstimationDiagnostics::ConstPtr estimation_diagnostics,
                                                                                       mrs_msgs::Float64Stamped::ConstPtr local_heading,
                                                                                       sensor_msgs::NavSatFix::ConstPtr global_position,
                                                                                       mrs_msgs::Float64Stamped::ConstPtr global_heading)
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

  mrs_robot_diagnostics::ControlInfo StateMonitor::parse_control_info(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics,
                                                                      std_msgs::Float64::ConstPtr thrust)
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

  mrs_robot_diagnostics::UavInfo StateMonitor::parse_uav_info(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::UavStatus::ConstPtr uav_status,
                                                              std_msgs::Float64::ConstPtr mass_nominal, std_msgs::Float64::ConstPtr mass_estimate,
                                                              uav_state_t uav_state)
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

  /* parse_system_health_info() method //{ */
  mrs_robot_diagnostics::SystemHealthInfo StateMonitor::parse_system_health_info(mrs_msgs::UavStatus::ConstPtr uav_status,
                                                                                 sensor_msgs::NavSatFix::ConstPtr gnss,
                                                                                 sensor_msgs::MagneticField::ConstPtr magnetic_field)
  {
    mrs_robot_diagnostics::SystemHealthInfo msg;

    msg.cpu_load = uav_status->cpu_load;
    msg.free_ram = uav_status->free_ram;
    msg.total_ram = uav_status->total_ram;
    msg.free_hdd = uav_status->free_hdd;
    const size_t n = std::min(uav_status->node_cpu_loads.cpu_loads.size(), uav_status->node_cpu_loads.node_names.size());
    for (int it = 0; it < n; it++)
    {
      mrs_robot_diagnostics::NodeCpuLoad node_cpu_load;
      node_cpu_load.node_name = uav_status->node_cpu_loads.node_names.at(it);
      node_cpu_load.cpu_load = uav_status->node_cpu_loads.cpu_loads.at(it);
      msg.node_cpu_loads.push_back(node_cpu_load);
    }

    msg.hw_api_rate = uav_status->hw_api_hz;
    msg.control_manager_rate = uav_status->control_manager_diag_hz;
    msg.state_estimation_rate = uav_status->odom_hz;

    {
      const Eigen::Matrix3d cov = cov2eigen(gnss->position_covariance);
      msg.gnss_uncertainty = std::cbrt(cov.determinant());
    }

    if (magnetic_field == nullptr)
    {
      msg.mag_strength = -1;
      msg.mag_uncertainty = -1;
    } else
    {
      const Eigen::Vector3d field(magnetic_field->magnetic_field.x, magnetic_field->magnetic_field.y, magnetic_field->magnetic_field.z);
      msg.mag_strength = field.norm();
      const Eigen::Matrix3d cov = cov2eigen(magnetic_field->magnetic_field_covariance);
      msg.mag_uncertainty = std::cbrt(cov.determinant());
    }

    // TODO: required sensors from estimation manager
    mrs_robot_diagnostics::SensorStatus ss_msg;
    ss_msg.name = "NOT_IMPLEMENTED";
    ss_msg.ready = false;
    ss_msg.rate = -1;
    ss_msg.status = "NOT_IMPLEMENTED";

    msg.required_sensors.push_back(ss_msg);

    return msg;
  }
  //}

  // | -------------------- Msg init methods -------------------- |

  /* init_general_robot_info() method //{ */
  mrs_robot_diagnostics::GeneralRobotInfo StateMonitor::init_general_robot_info()
  {
    mrs_robot_diagnostics::GeneralRobotInfo msg;

    msg.stamp = ros::Time(0);
    msg.robot_name = "";
    msg.robot_type = 0;

    msg.battery_state.voltage = -1;
    msg.battery_state.percentage = -1;
    msg.battery_state.wh_drained = -1;

    msg.ready_to_start = false;
    msg.problems_preventing_start.emplace_back("Diagnostics not initialized.");

    return msg;
  }
  //}

  /* init_state_estimation_info() method //{ */
  mrs_robot_diagnostics::StateEstimationInfo StateMonitor::init_state_estimation_info()
  {
    mrs_robot_diagnostics::StateEstimationInfo msg;

    msg.header.stamp = ros::Time(0);
    msg.header.frame_id = "";

    msg.local_pose.position.x = std::numeric_limits<double>::quiet_NaN();
    msg.local_pose.position.y = std::numeric_limits<double>::quiet_NaN();
    msg.local_pose.position.z = std::numeric_limits<double>::quiet_NaN();
    msg.local_pose.heading = std::numeric_limits<double>::quiet_NaN();

    msg.velocity.linear.x = std::numeric_limits<double>::quiet_NaN();
    msg.velocity.linear.y = std::numeric_limits<double>::quiet_NaN();
    msg.velocity.linear.z = std::numeric_limits<double>::quiet_NaN();
    msg.velocity.angular.x = std::numeric_limits<double>::quiet_NaN();
    msg.velocity.angular.y = std::numeric_limits<double>::quiet_NaN();
    msg.velocity.angular.z = std::numeric_limits<double>::quiet_NaN();

    msg.acceleration.linear.x = std::numeric_limits<double>::quiet_NaN();
    msg.acceleration.linear.y = std::numeric_limits<double>::quiet_NaN();
    msg.acceleration.linear.z = std::numeric_limits<double>::quiet_NaN();
    msg.acceleration.angular.x = std::numeric_limits<double>::quiet_NaN();
    msg.acceleration.angular.y = std::numeric_limits<double>::quiet_NaN();
    msg.acceleration.angular.z = std::numeric_limits<double>::quiet_NaN();

    msg.above_ground_level_height = std::numeric_limits<double>::quiet_NaN();

    msg.global_pose.position.x = std::numeric_limits<double>::quiet_NaN();
    msg.global_pose.position.y = std::numeric_limits<double>::quiet_NaN();
    msg.global_pose.position.z = std::numeric_limits<double>::quiet_NaN();
    msg.global_pose.heading = std::numeric_limits<double>::quiet_NaN();

    msg.current_estimator = "unknown";

    return msg;
  }
  //}

  /* init_control_info() method //{ */
  mrs_robot_diagnostics::ControlInfo StateMonitor::init_control_info()
  {
    mrs_robot_diagnostics::ControlInfo msg;

    msg.active_controller = "unknown";
    msg.active_tracker = "unknown";
    msg.thrust = std::numeric_limits<double>::quiet_NaN();

    return msg;
  }
  //}

  /* init_collision_avoidance_info() method //{ */
  mrs_robot_diagnostics::CollisionAvoidanceInfo StateMonitor::init_collision_avoidance_info()
  {
    mrs_robot_diagnostics::CollisionAvoidanceInfo msg;

    msg.collision_avoidance_enabled = false;
    msg.avoiding_collision = false;

    return msg;
  }
  //}

  /* init_uav_info() method //{ */
  mrs_robot_diagnostics::UavInfo StateMonitor::init_uav_info()
  {
    mrs_robot_diagnostics::UavInfo msg;

    msg.flight_state = "unknown";
    msg.flight_duration = std::numeric_limits<double>::quiet_NaN();
    msg.armed = false;
    msg.offboard = false;
    msg.mass_nominal = std::numeric_limits<double>::quiet_NaN();
    msg.mass_estimate = std::numeric_limits<double>::quiet_NaN();

    return msg;
  }
  //}

  /* init_system_health_info() method //{ */
  mrs_robot_diagnostics::SystemHealthInfo StateMonitor::init_system_health_info()
  {
    mrs_robot_diagnostics::SystemHealthInfo msg;

    msg.cpu_load = std::numeric_limits<float>::quiet_NaN();
    msg.free_ram = std::numeric_limits<float>::quiet_NaN();
    msg.total_ram = std::numeric_limits<float>::quiet_NaN();
    msg.free_hdd = -1;

    msg.hw_api_rate = std::numeric_limits<float>::quiet_NaN();
    msg.control_manager_rate = std::numeric_limits<float>::quiet_NaN();
    msg.state_estimation_rate = std::numeric_limits<float>::quiet_NaN();

    msg.gnss_uncertainty = std::numeric_limits<float>::quiet_NaN();
    msg.mag_strength = std::numeric_limits<float>::quiet_NaN();
    msg.mag_uncertainty = std::numeric_limits<float>::quiet_NaN();

    return msg;
  }
  //}

}  // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::StateMonitor, nodelet::Nodelet);

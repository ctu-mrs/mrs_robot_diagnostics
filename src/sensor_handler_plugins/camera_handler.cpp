#include <mrs_robot_diagnostics/sensor_plugins/camera_handler.h>

namespace mrs_robot_diagnostics
{

namespace sensor_handlers
{

namespace camera_handler
{

bool CameraHandler::initialize(ros::NodeHandle &nh, const std::string &name, const std::string &name_space, const std::string &topic) {
  ros::NodeHandle nh_(nh, name_space);
  _name_  = name;
  _topic_ = topic;
  ros::Time::waitForValid();


  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "StateMonitor";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  std::string image_topic_name;
  nh.getParam("image_topic", image_topic_name);
  ROS_INFO(" [CameraHandler] Subscribing to image topic: %s", image_topic_name.c_str());

  std::string camera_info_topic_name;
  nh.getParam("camera_info_topic", camera_info_topic_name);
  ROS_INFO(" [CameraHandler] Subscribing to camera info topic: %s", camera_info_topic_name.c_str());

  std::string camera_orientation_topic_name;
  nh.getParam("camera_orientation_topic", camera_orientation_topic_name);
  ROS_INFO("[CameraHandler] Subscribing to camera orientation topic: %s", camera_orientation_topic_name.c_str());

  if (!nh.getParam("camera_frame", _camera_frame_)) {
    ROS_WARN("Parameter 'camera_frame' not found, using default");
    return false;
  }

  if (!nh.getParam("optical_frame", _optical_frame_)) {
    ROS_WARN("Parameter 'optical_frame' not found, using default");
    return false;
  }

  if (!nh.getParam("fcu_frame", _fcu_frame_)) {
    ROS_WARN("Parameter 'fcu_frame' not found, using default");
    return false;
  }

  // Subscribers
  ROS_INFO(" [CameraHandler] Subscribing to image topic: %s", image_topic_name.c_str());
  sh_image_              = createSubscriber<sensor_msgs::Image>(nh_, image_topic_name);
  sh_camera_info_        = mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo>(shopts, camera_info_topic_name);
  sh_camera_orientation_ = mrs_lib::SubscribeHandler<std_msgs::Float32MultiArray>(shopts, camera_orientation_topic_name);

  // Publisher
  ph_camera_details_ = mrs_lib::PublisherHandler<mrs_robot_diagnostics::SensorInfo>(nh, "out/sensor_info", 1, true);

  ROS_INFO("Camera handler '%s' initialized in namespace '%s'", name.c_str(), name_space.c_str());
  is_initialized_ = true;
  return true;
}

mrs_robot_diagnostics::SensorStatus CameraHandler::updateStatus() {
  mrs_robot_diagnostics::SensorStatus ss_msg;
  ss_msg.name = _name_;
  ss_msg.type = mrs_robot_diagnostics::SensorStatus::TYPE_CAMERA;

  if (!is_initialized_) {
    ss_msg.ready  = false;
    ss_msg.rate   = -1;
    ss_msg.status = "NOT_INITIALIZED";
    return ss_msg;
  }

  json camera_info_json;
  if (sh_camera_info_.hasMsg()) {

    ss_msg.rate = current_rate_;

    bool has_image = sh_image_.hasMsg();
    ss_msg.ready   = has_image;
    if (has_image) {
      ss_msg.status = "OK";
    } else {
      ss_msg.status = "NO_IMAGE_DATA";
    }

    auto msg            = sh_camera_info_.getMsg();
    const double height = msg->height;
    const double width  = msg->width;
    const double fx     = msg->K[0];
    const double fy     = msg->K[4];

    const double fov_x = 2 * atan(width / (2 * fx));
    const double fov_y = 2 * atan(height / (2 * fy));

    camera_info_json = {
        {"height", height},
        {"width", width},
        {"fov_x_rad", fov_x},
        {"fov_y_rad", fov_y},
    };

  } else {
    ss_msg.ready  = false;
    ss_msg.rate   = -1;
    ss_msg.status = "NO_CAMERA_INFO";
  }

  geometry_msgs::TransformStamped transform;
  json camera_tf_json;
  try {
    transform = tf_buffer_.lookupTransform(_fcu_frame_, _camera_frame_, ros::Time(0));
    double x  = transform.transform.translation.x;
    double y  = transform.transform.translation.y;
    double z  = transform.transform.translation.z;

    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;

    tf2::Quaternion q(qx, qy, qz, qw);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    camera_tf_json = {
        {"translation", {{"x", x}, {"y", y}, {"z", z}}},
        {"rotation_rpy", {{"roll", roll}, {"pitch", pitch}, {"yaw", yaw}}},
    };
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }


  json optical_tf_json;
  try {
    transform = tf_buffer_.lookupTransform(_fcu_frame_, _optical_frame_, ros::Time(0));
    double x  = transform.transform.translation.x;
    double y  = transform.transform.translation.y;
    double z  = transform.transform.translation.z;

    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;

    tf2::Quaternion q(qx, qy, qz, qw);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    optical_tf_json = {
        {"translation", {{"x", x}, {"y", y}, {"z", z}}},
        {"rotation_rpy", {{"roll", roll}, {"pitch", pitch}, {"yaw", yaw}}},
    };
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }


  json camera_orientation_json;
  if (sh_camera_orientation_.hasMsg()) {
    auto orientation_msg    = sh_camera_orientation_.getMsg();
    camera_orientation_json = {
        {"orientation_rpy",
         {
             {"roll", orientation_msg->data[0]},
             {"pitch", orientation_msg->data[1]},
             {"yaw", orientation_msg->data[2]},
         }},
    };
  }

  json json_msg = {
      {"camera_frame_tf", camera_tf_json},
      {"optical_frame_tf", optical_tf_json},
      {"camera_info", camera_info_json},
      {"camera_orientation", camera_orientation_json},
  };

  std::string json_str = json_msg.dump();

  mrs_robot_diagnostics::SensorInfo sensor_info_msg;
  sensor_info_msg.type    = mrs_robot_diagnostics::SensorStatus::TYPE_CAMERA;
  sensor_info_msg.details = json_str;
  ph_camera_details_.publish(sensor_info_msg);
  // Return the camera status
  return ss_msg;
}


} // namespace camera_handler
} // namespace sensor_handlers
} // namespace mrs_robot_diagnostics
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::sensor_handlers::camera_handler::CameraHandler, mrs_robot_diagnostics::sensor_handlers::SensorHandler)

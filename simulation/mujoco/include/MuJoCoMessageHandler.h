#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "communication/msg/actuator_cmds.hpp"
#include "communication/msg/touch_sensor.hpp"
#include "communication/srv/simulation_reset.hpp"
#include "communication/msg/mcl_actuator.hpp" 
#include "communication/msg/mcl_imu.hpp" 

#include "array_safety.h"
#include "simulate.h"

using namespace rclcpp;

using namespace std::chrono_literals;

namespace deepbreak {
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

class MuJoCoMessageHandler : public rclcpp::Node {
public:
  struct ActuatorCmds {
    double time = 0.0;
    std::vector<std::string> actuators_name;
    std::vector<float> kp;
    std::vector<float> pos;
    std::vector<float> kd;
    std::vector<float> vel;
    std::vector<float> torque;
  };

  MuJoCoMessageHandler(mj::Simulate *sim);
  ~MuJoCoMessageHandler();

  std::shared_ptr<ActuatorCmds> get_actuator_cmds_ptr();

private:
  void reset_callback(
      const std::shared_ptr<communication::srv::SimulationReset::Request> request,
      std::shared_ptr<communication::srv::SimulationReset::Response> response);

  void imu_callback();

  void joint_callback();

  void actuator_cmd_callback(
      const communication::msg::MclActuator::SharedPtr msg) const; // 보증을 의미함.

  void parameter_callback(const rclcpp::Parameter &);

  void drop_old_message();

  // void throw_box();

  mj::Simulate *sim_; // fixed pointer 
  std::string name_prefix, model_param_name;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  rclcpp::Publisher<communication::msg::MclImu>::SharedPtr
      imu_publisher_;
  rclcpp::Publisher<communication::msg::MclActuator>::SharedPtr
      joint_state_publisher_;

  rclcpp::Subscription<communication::msg::MclActuator>::SharedPtr
      actuator_cmd_subscription_;


  rclcpp::Service<communication::srv::SimulationReset>::SharedPtr reset_service_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

  std::shared_ptr<ActuatorCmds> actuator_cmds_ptr_;
  // std::thread spin_thread;
};

} // namespace deepbreak

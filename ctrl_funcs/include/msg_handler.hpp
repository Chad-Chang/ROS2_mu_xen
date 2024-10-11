#pragma once

#include <thread>
#include <iostream>
// #include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "communication/msg/actuator_cmds.hpp"
#include "communication/msg/touch_sensor.hpp"
#include "communication/srv/simulation_reset.hpp"

#include "communication/msg/mcl_imu.hpp"
#include "communication/msg/mcl_actuator.hpp"


#define RNUMOFSLAVES 14 // real motor number
#define SNUMOFSLAVES 12 // simulation motor number
using namespace rclcpp;

using namespace std::chrono_literals;

class MsgHandler : public rclcpp::Node{
public:
    struct RealRobotState // 실제 로봇에서 통신 받아올 데이터: 나중에 공유 포인터로 데이터 접근할 계획
    {
        double time = 0;
        std::vector<std::string> actuators_name;
        std::vector<double> jpos;
        std::vector<double> jvel;
        std::vector<double> jcurrent;
        std::vector<std::string> imu_name;
        std::vector<double> imu; // 아직 어떻게 데이터를 받을지 모르겠네?
        
    };
    struct SimRobotState // 시뮬레이션에서 받을 데이터 : 나중에 공유 포인터로 데이터 접근할 계획
    {
        double time = 0;
        std::vector<std::string> actuators_name;
        std::vector<double> jpos;
        std::vector<double> jvel;
        std::vector<std::string> imu_name;
        std::vector<double> imu;

        std::vector<double> kp_rw;
        std::vector<double> ki_rw;
        std::vector<double> kd_rw;
        std::vector<double> qd_rw;
        std::vector<double> kp_j;
        std::vector<double> ki_j;
        std::vector<double> kd_j;
        std::vector<double> qd_j;
        std::vector<double> q_jDob;
        std::vector<double> q_oriDob;
        std::vector<double> q_rwDob;
    };
    

    MsgHandler();
    ~MsgHandler();
    
    // std::shared_ptr<SimRobotState> get_sim_cmd_ptr(); // sim pos, vel torq .. 구조체 접근 : 구독한데이터 저장
    // std::shared_ptr<RealRobotState> get_real_cmd_ptr(); // real pos -> 구독한 데이터 저장

private:
    std::string sim_name_prefix, real_name_prefix;
    void ctrl_callback(); // publish 뭔가 주기마다 쏴주면 안될꺼같은 느낌 -> 계산되고 쏴줘야할꺼같은데 주기적으로 하면 안될꺼같은 느낌임.
    void sensor_jnt_callback(const communication::msg::MclActuator::SharedPtr msg) const; // sensor subscrib
    void sensor_imu_callback(const communication::msg::MclImu::SharedPtr msg) const; //sensor subscrib

    // std::string name_prefix, model_param_name;
    
    // rclcpp::Subscription<communication::msg::MclImu>::SharedPtr
    //   imu_real_subsc_;
    // // state 
    rclcpp::Subscription<communication::msg::MclActuator>::SharedPtr
      jnt_sim_subsc_;
    rclcpp::Subscription<communication::msg::MclImu>::SharedPtr
      imu_sim_subsc_;
    // rclcpp::Subscription<communication::msg::MclActuator>::SharedPtr
    //   jnt_real_subsc_;
    
            /*simulation*/
    rclcpp::Publisher<communication::msg::MclActuator>::SharedPtr ctrl_sim_pub_;
    std::shared_ptr<SimRobotState> sim_state_cmd_ptr();
    
            /*real robot*/ 
    // rclcpp::Publisher<communication::msg::MclActuator>::SharedPtr ctrl_real_pub_;
    // std::shared_ptr<RealRobotState> real_state_cmd_ptr();

    // rclcpp::Service<communication::srv::SimulationReset>::SharedPtr service_;

    // std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

    // std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

    

    
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;

};
#include "msg_handler.hpp"


MsgHandler::MsgHandler():Node("mcl_node"),name_prefix("MCL_quad/")
{
    // this->declare_parameter(model_param_name, ""); // pid 게인같은거 설정해야겠다.

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    
    // 실행 트리거 있어야 할듯 
    ctrl_sim_pub_ = this->create_publisher<communication::msg::MclActuator>(name_prefix + "sim_actuator",qos);
    ctrl_real_pub_ = this->create_publisher<communication::msg::MclActuator>(name_prefix + "real_actuator",qos);
    
    jnt_sim_subsc_ = this -> create_subscription<communication::msg::MclActuator>(
        name_prefix + "actuators_cmds",qos, std::bind(&MsgHandler::jnt_callback,this,std::placeholders::_1)
    );
    jnt_real_subsc_  = this-> create_subscription<communication::msg::MclActuator>(
        name_prefix + "actuators_cmds",qos, std::bind(&MsgHandler::jnt_callback,this,std::placeholders::_1)
    );
    
    imu_sim_subsc_  = this-> create_subscription<communication::msg::MclImu>(
        name_prefix + "actuators_cmds",qos, std::bind(&MsgHandler::jnt_callback,this,std::placeholders::_1)
    );
    imu_real_subsc_ = this-> create_subscription<communication::msg::MclImu>(
        name_prefix + "actuators_cmds",qos, std::bind(&MsgHandler::imu_callback,this,std::placeholders::_1)
    ); 
    

    // 이거 타이머로하면 안될꺼같은 느낌 -> 데이터를 구독했을때 전체 제어루프가 돌아가게 하는게 맞을듯 
    timers_.emplace_back(this->create_wall_timer(
      1ms, std::bind(&MuJoCoMessageHandler::imu_callback, this)));
    timers_.emplace_back(this->create_wall_timer(
      1ms, std::bind(&MuJoCoMessageHandler::imu_callback, this)));
}


void MsgHandler::ctrl_callback()  // simulation 연산 + 실제 연산
{
    // controller pointer 받아오기 
    auto sim_msg = commuincation::msg::MclActuator();
    auto real_msg = commuincation::msg::MclActuator();
    // header frameid는 필요 없는건가 
    sim_msg.header.stamp = rclcpp::Clock().now();
    real_msg.header.stamp = rclcpp::Clock().now();
    
    // const std::lock_guard<std::mutex> lock();
    for(int i = 0 ;i<SNUMOFSLAVES; i++){
        sim_msg.jtorque[i] = 1 // ctrl -> jtorque[i]
    }
    ctrl_sim_pub_
}

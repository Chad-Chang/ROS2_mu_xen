#include "msg_handler.hpp"


MsgHandler::MsgHandler():Node("mcl_node"),sim_name_prefix("simulation/"), real_name_prefix("real_robot/")
{

    sim_state_cmd_ptr_ = std::make_shared<SimRobotState>();
    real_state_cmd_ptr_ = std::make_shared<RealRobotState>();
        //파라미터를 yaml파일에서 받은거를 사용하지 못하고 있음. -> 왜 그런지 모르겠네 
    // rw gain
    this->declare_parameter<std::vector<double>>("kp_rw_sim", std::vector<double>{0});  
    sim_state_cmd_ptr_ -> kp_rw = this -> get_parameter("kp_rw_sim").get_value<std::vector<double>>(); // 이거는 걍 위의 값을 가져옴.
    std::cout << sim_state_cmd_ptr_ -> kp_rw[0]<< " "<<sim_state_cmd_ptr_ -> kp_rw[1]<< " "<<sim_state_cmd_ptr_ -> kp_rw[2]<< " " <<std::endl;
    this->declare_parameter<std::vector<double>>("ki_rw_sim", std::vector<double>{0}); 
    sim_state_cmd_ptr_ -> ki_rw = this -> get_parameter("ki_rw_sim").get_value<std::vector<double>>();
    this->declare_parameter<std::vector<double>>("kd_rw_sim", std::vector<double>{0}); 
    sim_state_cmd_ptr_ -> kd_rw = this -> get_parameter("kd_rw_sim").get_value<std::vector<double>>();
    this->declare_parameter<std::vector<double>>("qd_rw_sim", std::vector<double>{1}); 
    sim_state_cmd_ptr_ -> qd_rw = this -> get_parameter("qd_rw_sim").get_value<std::vector<double>>();
    this->declare_parameter<std::vector<double>>("q_rwDob_sim", std::vector<double>{0.1}); 
    sim_state_cmd_ptr_ -> q_rwDob = this -> get_parameter("q_rwDob_sim").get_value<std::vector<double>>();
    this->declare_parameter<std::vector<double>>("q_oriDob_sim", std::vector<double>{0.1}); 
    sim_state_cmd_ptr_ -> q_oriDob = this -> get_parameter("q_oriDob_sim").get_value<std::vector<double>>();
    
    // // // joint gain parameter
    this->declare_parameter<std::vector<double>>("kp_j_sim", std::vector<double>{100}); 
    sim_state_cmd_ptr_ -> kp_j = this -> get_parameter("kp_j_sim").get_value<std::vector<double>>();
    this->declare_parameter<std::vector<double>>("ki_j_sim", std::vector<double>{0}); 
    sim_state_cmd_ptr_ -> ki_j = this -> get_parameter("ki_j_sim").get_value<std::vector<double>>();
    this->declare_parameter<std::vector<double>>("kd_j_sim", std::vector<double>{0}); 
    sim_state_cmd_ptr_ -> kd_j = this -> get_parameter("kd_j_sim").get_value<std::vector<double>>();
    this->declare_parameter<std::vector<double>>("qd_j_sim", std::vector<double>{1}); 
    sim_state_cmd_ptr_ -> qd_j = this -> get_parameter("qd_j_sim").get_value<std::vector<double>>();
    this->declare_parameter<std::vector<double>>("q_jDob_sim", std::vector<double>{0.1}); 
    sim_state_cmd_ptr_ -> q_jDob = this -> get_parameter("q_jDob_sim").get_value<std::vector<double>>();
    this -> update_parameter();
    

    this->declare_parameter("qos_depth", 1);  // 기본값 설정 
    int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth), rmw_qos_profile_sensor_data);
    
    // // 실행 트리거 있어야 할듯 
    ctrl_sim_pub_ = this->create_publisher<communication::msg::MclActuator>(sim_name_prefix + "actuators_cmds",qos);
    
    /*simulation robot*/
    // Mcl Actuator에 시뮬레이션인지 실제값인지도 적게 해야함. +받는 노드에서 이름 맞춰줘야함.
        // 만약에 콜백함수를 같은 것을 쓴다고했을때 공유포인터는 따로 생성되는가 아니면 그대로 사용되는가
    jnt_sim_subsc_ = this -> create_subscription<communication::msg::MclActuator>(
        sim_name_prefix + "joint_states",qos, std::bind(&MsgHandler::sensor_jnt_callback,this,std::placeholders::_1)
    );
    imu_sim_subsc_  = this-> create_subscription<communication::msg::MclImu>(
        sim_name_prefix + "imu_data",qos, std::bind(&MsgHandler::sensor_imu_callback,this,std::placeholders::_1)
    );
    
    /*real robot*/

    // ctrl_real_pub_ = this->create_publisher<communication::msg::MclActuator>(real_name_prefix + "actuators_cmds",qos);
    // jnt_real_subsc_  = this-> create_subscription<communication::msg::MclActuator>(
    //     name_prefix + "actuators_cmds",qos, std::bind(&MsgHandler::jnt_callback,this,std::placeholders::_1)
    // );
    // imu_real_subsc_ = this-> create_subscription<communication::msg::MclImu>(
    //     name_prefix + "actuators_cmds",qos, std::bind(&MsgHandler::imu_callback,this,std::placeholders::_1)
    // ); 
    
    // // 이거 타이머로하면 안될꺼같은 느낌 -> 데이터를 구독했을때 전체 제어루프가 돌아가게 하는게 맞을듯 
    timers_.emplace_back(this->create_wall_timer(
        1ms, std::bind(&MsgHandler::ctrl_callback, this))); // 이건 둘다 동시에 적용해도 될듯
    // timers_.emplace_back(this->create_wall_timer(
    //   1ms, std::bind(&MuJoCoMessageHandler::imu_callback, this)));
    
    
}

MsgHandler::~MsgHandler(){};

void MsgHandler::update_parameter()
        // 나중에 하드웨어까지 할때 
    // (std::shared_ptr<SimRobotState> sim_state_cmd_ptr, std::shared_ptr<RealRobotState> real_state_cmd_ptr)    
{
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    while(!parameters_client_ -> wait_for_service(1s)){
        if(!rclcpp::ok()){
             RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");

        // event: 등록, 변경, 삭제 
    auto param_event_callback = 
        [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for(auto &changed_parameter : event -> changed_parameters)
        {
            if(changed_parameter.name == "kp_rw_sim")
            {
                auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double_array();
                sim_state_cmd_ptr_ -> kp_rw = value;
            }
            else if(changed_parameter.name == "ki_rw_sim")
            {
                auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double_array();
                sim_state_cmd_ptr_ -> ki_rw = value;

            }
            else if(changed_parameter.name == "kd_rw_sim")
            {
                auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double_array();
                sim_state_cmd_ptr_ -> kd_rw = value;
            }
        }
    };
    parameter_event_sub_ = parameters_client_->on_parameter_event(param_event_callback);
}

void MsgHandler::ctrl_callback()  // simulation 연산 + 실제 연산
{
    // controller pointer 받아오기 
    auto sim_msg = communication::msg::MclActuator();
    // auto real_msg = communication::msg::MclActuator();
    // std::cout << "sending "<<std::endl;
    // RCLCPP_INFO(this->get_logger(), "publish actuator cmds ...");
    sim_msg.header.stamp = rclcpp::Clock().now();
    
    // const std::lock_guard<std::mutex> lock();
    for(int i = 0 ;i<SNUMOFSLAVES ; i++){
        sim_msg.jtorque.push_back(i); // 배열 메시지는 반드시 벡터 method를 이용해 넣어주기
    }
    ctrl_sim_pub_ -> publish(sim_msg);
}


void MsgHandler::sensor_jnt_callback(
    const communication::msg::MclActuator::SharedPtr msg)  const{
        if(!msg->mode){} // 시뮬레이션일때 
            // RCLCPP_INFO(this->get_logger(), "subscribe joint data");
}


void MsgHandler::sensor_imu_callback(
    const communication::msg::MclImu::SharedPtr msg)  const{
        if(!msg->mode){} // 시뮬레이션일때 
            // RCLCPP_INFO(this->get_logger(), "subscribe imu data");
}

std::shared_ptr<MsgHandler::SimRobotState>
MsgHandler::get_sim_cmd_ptr() 
{
  return sim_state_cmd_ptr_;
}

std::shared_ptr<MsgHandler::RealRobotState>
MsgHandler::get_real_cmd_ptr() 
{
  return real_state_cmd_ptr_;
}
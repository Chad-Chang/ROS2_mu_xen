
#include <string>
#include "mujoco2.hpp"
#include <iostream>

using std::placeholders::_1;
MujocoSimNode* MujocoSimNode::sim_node_instance_ = nullptr;

MujocoSimNode::MujocoSimNode(const rclcpp::NodeOptions & node_options) 
: Node("mujoco_sim_node",node_options)
{
    
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    // const auto QOS_RKL10V =
    //     rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    RCLCPP_INFO(this->get_logger(), "Starting MuJoCo Simulation Node");
    


    // MuJoCo 및 시뮬레이션 설정
    char error[1000] = "Could not load MuJoCo model";
    
    // 현재 인스턴스를 전역 포인터에 저장
    sim_node_instance_ = this;
    
    std::string model_file = std::string("/home/chad/ros2_ws/src/mujoco_sim_test/model/scene.xml");
    // std::string package_share_directory = ament_index_cpp::get_package_share_directory("mujoco_sim_test");
    // std::string model_file = package_share_directory + "/model/scene.xml";
    
    // MuJoCo 모델 로드
    m_ = mj_loadXML(model_file.c_str(), nullptr, error, 1000);
    if (!m_) {
        RCLCPP_ERROR(this->get_logger(), "MuJoCo model load error: %s", error);
        rclcpp::shutdown();
    }

    d_ = mj_makeData(m_);

    // GLFW 초기화
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    window_ = glfwCreateWindow(1244, 700, "MuJoCo Simulation", nullptr, nullptr);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    // 시각화 데이터 설정
    mjv_defaultCamera(&this -> cam_);
    mjv_defaultOption(&this -> opt_);
    mjv_defaultScene(&this -> scn_);
    mjr_defaultContext(&this -> con_);
    mjv_makeScene(this ->m_, &this -> scn_, 2000);
    mjr_makeContext(this ->m_, &this ->con_, mjFONTSCALE_150);
    
    //무조코 call back 이벤트
    glfwSetKeyCallback(window_, MujocoSimNode::keyboard);
    glfwSetCursorPosCallback(window_, MujocoSimNode::mouse_move);
    glfwSetMouseButtonCallback(window_, MujocoSimNode::mouse_button);
    glfwSetScrollCallback(window_, MujocoSimNode::scroll);


    double arr_view[] = {-88.95, -17.5, 1.8, 0,d_->qpos[0], 0.27};
    cam_.azimuth = arr_view[0];
    cam_.elevation = arr_view[1];
    cam_.distance = arr_view[2];
    cam_.lookat[0] = arr_view[3];
    cam_.lookat[1] = arr_view[4];
    cam_.lookat[2] = arr_view[5];
    // mjcb_control = mycontroller;



    test_subscriber_ = this -> create_subscription<std_msgs::msg::String>( // subscription callback
        "test", qos_profile, std::bind(&MujocoSimNode::subscribe_topic_msg, this, _1)
    );
    
    // auto get_operator = [this](const std::shared_ptr<ArithmeticOperator::Request> request)->void
    // {
    //        RCLCPP_INFO(this->get_logger(), "service operator");
    // }

    // test_server_ =
    //     create_service<ArithmeticOperator>("arithmetic_operator", get_arithmetic_operator); // 서버 노드 생성

    // test_action_server_= rclcpp_action::create_server<ArithmeticChecker>( 
    //     this->get_node_base_interface(),
    //     this->get_node_clock_interface(),
    //     this->get_node_logging_interface(),
    //     this->get_node_waitables_interface(),
    //     "arithmetic_checker",
    //     std::bind(&MujocoSimNode::handle_goal, this, _1, _2), // this는 함수가 어떤 instance와 연결되어있는지 명시적으로 표시
    //     std::bind(&MujocoSimNode::handle_cancel, this, _1), 
    //     std::bind(&MujocoSimNode::execute_checker, this, _1)
    // );


    // ROS 2 타이머로 시뮬레이션 루프 실행

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000/60), 
        std::bind(&MujocoSimNode::simulation_loop, this));


    
}

MujocoSimNode::~MujocoSimNode()
{
    // MuJoCo 리소스 정리
    mj_deleteData(d_);
    mj_deleteModel(m_);
    mjr_freeContext(&con_);
    mjv_freeScene(&scn_);
    glfwTerminate();
}

void MujocoSimNode::simulation_loop()
{   
    RCLCPP_INFO(this->get_logger(), "simulation working");
    // double simEndtime = 5;
    mjtNum simstart = d_->time;
    while (d_->time - simstart < 1.0 / 60.0) // 60hz주기로 시각화
    {   
        mj_step(m_, d_);
    }
    // if (d->time >= simEndtime) {    
    //     break;
    // }

    // 뷰포트 설정
    mjrRect viewport = { 0, 0, 0, 0 };
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);


    // 장면 업데이트 및 렌더링
    mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport, &scn_, &con_);

    // OpenGL 버퍼 스왑
    glfwSwapBuffers(window_);
    glfwPollEvents();

}
void MujocoSimNode::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    (void) window;  (void) mods;
    (void) scancode; 
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(sim_node_instance_->m_, sim_node_instance_-> d_);
        mj_forward(sim_node_instance_->m_, sim_node_instance_->d_);
    }
}

void MujocoSimNode::mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // RCLCPP_INFO(sim_node_instance->get_logger(), "button event");
    (void) mods; (void) button;  (void) act;
    // update button state
    sim_node_instance_-> button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    sim_node_instance_->button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    sim_node_instance_->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
            
    // update mouse position
    glfwGetCursorPos(window, &sim_node_instance_->lastx_, &sim_node_instance_->lasty_);
    
}
void MujocoSimNode::mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // RCLCPP_INFO(sim_node_instance->get_logger(), "mouse move");
    if(!sim_node_instance_-> button_left_ && !sim_node_instance_-> button_middle_ &&!sim_node_instance_->button_right_)
        return ;
    // compute mouse displacement, save
    double dx = xpos - sim_node_instance_->lastx_;
    double dy = ypos - sim_node_instance_->lasty_;
    sim_node_instance_->lastx_ = xpos;
    sim_node_instance_->lasty_ = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (sim_node_instance_->button_right_)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (sim_node_instance_-> button_left_)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(sim_node_instance_-> m_, action, dx / height, dy / height, &sim_node_instance_-> scn_, &sim_node_instance_-> cam_);
}
void MujocoSimNode::scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    (void) window; (void) xoffset;
    // emulate vertical mouse motion = 5% of window height
    
        mjv_moveCamera(sim_node_instance_-> m_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &sim_node_instance_->scn_, &sim_node_instance_->cam_);
}


// void MujocoSimNode::execute_checker(const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle)
// {
//   RCLCPP_INFO(this->get_logger(), "Execute arithmetic_checker action!");
//   rclcpp::Rate loop_rate(1);

//   auto feedback_msg = std::make_shared<ArithmeticChecker::Feedback>();
//   float total_sum = 0.0;
//   float goal_sum = goal_handle->get_goal()->goal_sum;

//   while ((total_sum < goal_sum) && rclcpp::ok()) {
//     total_sum += argument_result_;
//     feedback_msg->formula.push_back(argument_formula_);
//     if (argument_formula_.empty()) {
//       RCLCPP_WARN(this->get_logger(), "Please check your formula");
//       break;
//     }
//     RCLCPP_INFO(this->get_logger(), "Feedback: ");
//     for (const auto & formula : feedback_msg->formula) {
//       RCLCPP_INFO(this->get_logger(), "\t%s", formula.c_str());
//     }
//     goal_handle->publish_feedback(feedback_msg);
//     loop_rate.sleep();
//   }

//   if (rclcpp::ok()) {
//     auto result = std::make_shared<ArithmeticChecker::Result>();
//     result->all_formula = feedback_msg->formula;
//     result->total_sum = total_sum;
//     goal_handle->succeed(result);
//   }
// }



// rclcpp_action::GoalResponse Calculator::handle_goal( // goal handler  : 
//   //input : Goal 주소 , Goal구조체
//   const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ArithmeticChecker::Goal> goal)  action의 goal 구조체
// // goal마다 부여되는 고유 식별자 (UUID) 클라이언트가 여러개의 목표를 보낼수 있으므로 각 목표를 구분하기 위해 고유한 UUID
// // 실제 goal
// {
//   (void)uuid;  // void : 실제로 사용하지 않는다는 말
//   (void)goal; // void : 실제로 사용하지 않는다는 말
//   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // 목표를 수락하고  작업을 즉시 실행
// }

// rclcpp_action::CancelResponse Calculator::handle_cancel(
//   //GoalHandleArithmeticChecker : 헤더에서 정의한 자료형
//   const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle)
// {
//   RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
//   (void)goal_handle;
//   return rclcpp_action::CancelResponse::ACCEPT;
// }

// void MujocoSimNode::mycontroller(const mjModel* m, mjData* d)
// {
//     (void) m;
//     (void) d;
// }


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::executors::MultiThreadedExecutor executor;
    
    auto node = std::make_shared<MujocoSimNode>();
    rclcpp::spin(node);
    // executor.add_node(node);
    // executor.spin();
    rclcpp::shutdown();
    return 0;
}


#ifndef MUJOCO_SIMULATION_MUJOCO_SIM_NODE_HPP_
#define MUJOCO_SIMULATION_MUJOCO_SIM_NODE_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>
#include <thread>
#include <mutex>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "msg_srv_action_interface_example/msg/arithmetic_argument.hpp"
#include "msg_srv_action_interface_example/srv/arithmetic_operator.hpp"
#include "msg_srv_action_interface_example/action/arithmetic_checker.hpp"



class MujocoSimNode : public rclcpp::Node
{
public:
    using ArithmeticArgument = msg_srv_action_interface_example::msg::ArithmeticArgument;
    using ArithmeticOperator = msg_srv_action_interface_example::srv::ArithmeticOperator;
    using ArithmeticChecker = msg_srv_action_interface_example::action::ArithmeticChecker;
    using GoalHandleArithmeticChecker = rclcpp_action::ServerGoalHandle<ArithmeticChecker>;


    explicit MujocoSimNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
    virtual ~MujocoSimNode();

private:
    void simulation_loop();
    //콜백함수는 반드시 정적으로 처리해야함.
    static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    static void mouse_button(GLFWwindow* window, int button, int act, int mods);
    static void mouse_move(GLFWwindow* window, double xpos, double ypos);
    static void scroll(GLFWwindow* window, double xoffset, double yoffset);
    // // void mycontroller(const mjModel* m, mjData* d);

    // rclcpp_action::GoalResponse handle_goal(
    // const rclcpp_action::GoalUUID & uuid,
    // std::shared_ptr<const ArithmeticChecker::Goal> goal);
    // rclcpp_action::CancelResponse handle_cancel(
    // const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle);
    // void execute_checker(const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle);


    static MujocoSimNode* sim_node_instance_;  // 정적 인스턴스 포인터

    bool button_left_ = false;
    bool button_middle_= false;
    bool button_right_= false;
    double lastx_= 0;
    double lasty_= 0;

  
    // rclcpp::Subscription<ArithmeticArgument>::SharedPtr test_subscriber_;
    
    // rclcpp::Service<ArithmeticOperator>::SharedPtr test_server_;
    // rclcpp_action::Server<ArithmeticChecker>::SharedPtr test_action_server_;
    
    void subscribe_topic_msg(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received message : '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr test_subscriber_;

    mjModel* m_ = nullptr;         // MuJoCo 모델
    mjData* d_ = nullptr;          // MuJoCo 데이터
    
    GLFWwindow* window_ = nullptr; // GLFW 창
    mjvCamera cam_;                // MuJoCo 카메라
    mjvOption opt_;                // MuJoCo 옵션
    mjvScene scn_;                 // MuJoCo 장면
    mjrContext con_;               // MuJoCo 렌더링 컨텍스트
    rclcpp::TimerBase::SharedPtr timer_; // ROS 2 타이머
};

#endif // MUJOCO_SIMULATION_MUJOCO_SIM_NODE_HPP_
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>
#include <thread>
#include <mutex>
// #include <condition_variable>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"


// namespace mujoco 
// {
class Simulate
{
public:
    explicit Simulate();
    virtual ~Simulate();
    
    void render(); // 콜백 함수는 밖에 해줘야겠다.
// private:
    mjModel* m_ = nullptr;         // MuJoCo 모델
    mjData* d_ = nullptr;          // MuJoCo 데이터
    mjModel* mnew_ = nullptr;         // MuJoCo 모델
    mjData* dnew_ = nullptr;          // MuJoCo 데이터

    GLFWwindow* window_ = nullptr; // GLFW 창
    mjvCamera cam_;                // MuJoCo 카메라
    mjvOption opt_;                // MuJoCo 옵션
    mjvScene scn_;                 // MuJoCo 장면
    mjrContext con_;               // MuJoCo 렌더링 컨텍스트
    // rclcpp::TimerBase::SharedPtr timer_; // ROS 2 타이머
    std::mutex mtx;
    // std::condition_variable cond_loadrequest;
};



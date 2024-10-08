#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <thread>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "sim.hpp"

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"


namespace mujoco{
    bool button_left = false;
    bool button_middle= false;
    bool button_right= false;
    double lastx= 0;
    double lasty= 0;



// void mouse_move(GLFWwindow* window, double xpos, double ypos)
// {
//     // RCLCPP_INFO(sim_node_instance->get_logger(), "mouse move");
//     if(!button_left &&  button_middle &&!button_right)
//         return ;
//     // compute mouse displacement, save
//     double dx = xpos - sim_node_instance_->lastx;
//     double dy = ypos - sim_node_instance_->lasty;
//     lastx = xpos;
//     lasty = ypos;

//     // get current window size
//     int width, height;
//     glfwGetWindowSize(window, &width, &height);

//     // get shift key state
//     bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
//         glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

//     // determine action based on mouse button
//     mjtMouse action;
//     if (button_right)
//         action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
//     else if (button_left_)
//         action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
//     else
//         action = mjMOUSE_ZOOM;

//     // move camera
//     mjv_moveCamera(sim_node_instance_-> m_, action, dx / height, dy / height, &sim_node_instance_-> scn_, &sim_node_instance_-> cam_);
// }

// void mouse_button(GLFWwindow* window, int button, int act, int mods)
// {
//     // RCLCPP_INFO(sim_node_instance->get_logger(), "button event");
//     (void) mods; (void) button;  (void) act;
//     // update button state
//     button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
//     button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
//     button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
            
//     // update mouse position
//     glfwGetCursorPos(window, &sim_node_instance_->lastx_, &sim_node_instance_->lasty_);
    
// }
// void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
// {
//     (void) window;  (void) mods;
//     (void) scancode; 
//     // backspace: reset simulation
//     if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
//     {
//         mj_resetData(sim_node_instance_->m_, sim_node_instance_-> d_);
//         mj_forward(sim_node_instance_->m_, sim_node_instance_->d_);
//     }
// }


void PhysicsLoop(Simulate &sim) 
{
    // std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    // mjtNum syncSim = 0;
    while(1)
    {
        {
            // const std::lock_guard<std::mutex> lock(sim.mtx);
             if (sim.m_) 
             {
                // const auto startCPU = mj::Simulate::Clock::now();
                // // elapsed CPU and simulation time since last sync
                // const auto elapsedCPU = startCPU - syncCPU;
                // double elapsedSim = d->time - syncSim;
                
                // apply_ctrl(sim.m, sim.d);
                mj_step(sim.m_, sim.d_);
             }
        }
    }

}

void PhysicsThread(Simulate *sim)
{
    mj_forward(sim->m_, sim->d_);
    
    PhysicsLoop(*sim);

    rclcpp::shutdown();
    // ~Simulate();
    sim -> ~Simulate();
}
// void mj_callback(Simulate &sim)
// {
//     glfwSetMouseButtonCallback(window_, MujocoSimNode::mouse_button);
// }
}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto sim = std::make_unique<Simulate>();    
    // std::thread physicsthreadhandle(&mujoco::PhysicsThread, sim.get(), nullptr); 
    

    sim->render();
    // physicsthreadhandle.join();
    rclcpp::shutdown();
    return 0;
}
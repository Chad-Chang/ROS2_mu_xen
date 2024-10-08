
#include "sim.hpp"
#include <atomic>
#include <chrono>
// #include <cstdio>
// #include <cstring>
#include <memory>
#include <mutex>
#include <ratio>
#include <string>
#include <thread>
#include <utility>


Simulate::Simulate() 
{
      // MuJoCo 및 시뮬레이션 설정
    char error[1000] = "Could not load MuJoCo model";
    
    std::string model_file = std::string("/home/chad/ros2_ws/src/mujoco_sim_test/model/scene.xml");
    m_ = mj_loadXML(model_file.c_str(), nullptr, error, 1000);
    d_ = mj_makeData(m_);

}

void Simulate::render() // 콜백 함수는 밖에 해줘야겠다.
{
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

    double arr_view[] = {-88.95, -17.5, 1.8, 0,d_->qpos[0], 0.27};
    cam_.azimuth = arr_view[0];
    cam_.elevation = arr_view[1];
    cam_.distance = arr_view[2];
    cam_.lookat[0] = arr_view[3];
    cam_.lookat[1] = arr_view[4];
    cam_.lookat[2] = arr_view[5];
}

Simulate::~Simulate()
{
    // MuJoCo 리소스 정리
    mj_deleteData(d_);
    mj_deleteModel(m_);
    mjr_freeContext(&con_);
    mjv_freeScene(&scn_);
    glfwTerminate();
}
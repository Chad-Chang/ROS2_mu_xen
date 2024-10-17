#include <stdio.h>
#include <iostream>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "leg_model.hpp"
#include "kinodynamics.hpp"
#include "controller.hpp"
#include "trajectory.hpp"
#include "msg_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

using namespace Eigen;
using namespace std;
// input : MsgHandler 포인터 + 시뮬레이션인지 아닌지
// 로봇 계산 함수 -> simulation에 대한 결과, 실제 로봇 제어 결과
// leg model -> 다리별로 제어기 할당 
// joint model -> 전체 joint의 제어값을 관리하는 structure
// 클래스 멤버 변수를 simulation, real으로 나누지 않기 

// 최종 토크값 담는 변수 
std::vector<double> sim_ctrl_input;
std::vector<double> real_ctrl_input;

std::vector<LegModel*> leg_FL_ptr; std::vector<LegModel*> leg_FR_ptr; std::vector<LegModel*> leg_RR_ptr; std::vector<LegModel*> leg_RL_ptr;

std::vector<Trajectory> Traj_FL; std::vector<Trajectory> Traj_FR; std::vector<Trajectory> Traj_RR; std::vector<Trajectory> Traj_RL;
std::vector<Controller> C_FL; std::vector<Controller> C_FR; std::vector<Controller> C_RR; std::vector<Controller> C_RL;
std::vector<Kinodynamics> K_FL; std::vector<Kinodynamics> K_FR; std::vector<Kinodynamics> K_RR; std::vector<Kinodynamics> K_RL;
// for(int i = 0; i<2; i++)







void update_msg_handler(MsgHandler *msg_handler)
{
	auto robot_ctrl_data = msg_handler->get_real_cmd_ptr();
	robot_ctrl_data -> jtorque = real_ctrl_input;
	auto sim_ctrl_data = msg_handler->get_sim_cmd_ptr();
	sim_ctrl_data -> jtorque = sim_ctrl_input;
	// publish하기
}

void leg_ctrl(MsgHandler *msg_handler, vector<Trajectory> T_FL,vector<Trajectory> T_FR,vector<Trajectory> T_RR,vector<Trajectory> T_RL,
 	std::vector<Kinodynamics> K_FL, std::vector<Kinodynamics> K_FR, std::vector<Kinodynamics> K_RR, std::vector<Kinodynamics> K_RL,
	vector<Controller> C_FL, vector<Controller> C_FR, vector<Controller> C_RR, vector<Controller> C_RL,
	int play_option) // play_option : [0 : 시뮬레이션만], [1: 실제 플랜트만]. [2 : 둘다 ]
{	
	(void) msg_handler;
	int i;
	while(1)
	{
		if(play_option != 3) // only simulation 
		{
			if(play_option == 0) i = 0; 
			else i = 1;
			C_FL[i].ctrl_update(); C_FR[i].ctrl_update(); C_RR[i].ctrl_update(); C_RL[i].ctrl_update();
			T_FL[i].hold(); T_FR[i].hold(); T_RR[i].hold(); T_RL[i].hold();// trajectory 선택
			K_FL[i].sensor_update(i); K_FR[i].sensor_update(i); K_RR[i].sensor_update(i); K_RL[i].sensor_update(i);
			K_FL[i].model_update(); K_FR[i].model_update(); K_RR[i].model_update(); K_RL[i].model_update();
			K_FL[i].FK(); K_FR[i].FK(); K_RR[i].FK(); K_RL[i].FK();
			//pos pid
			leg_FL_ptr[i] -> torque_bi = leg_FL_ptr[i]->jacbRW_trans * C_FL[i].rw_pos_pid(); 
			leg_FR_ptr[i] -> torque_bi = leg_FR_ptr[i]->jacbRW_trans * C_FR[i].rw_pos_pid(); 
			leg_RR_ptr[i] -> torque_bi = leg_RR_ptr[i]->jacbRW_trans * C_RR[i].rw_pos_pid(); 
			leg_RL_ptr[i] -> torque_bi = leg_RL_ptr[i]->jacbRW_trans * C_RL[i].rw_pos_pid();

			//DOB
			leg_FL_ptr[i] -> torque_bi =  leg_FL_ptr[i] -> torque_bi + C_FL[i].DOBRW();
			leg_FR_ptr[i] -> torque_bi =  leg_FR_ptr[i] -> torque_bi + C_FR[i].DOBRW();
			leg_RR_ptr[i] -> torque_bi =  leg_RR_ptr[i] -> torque_bi + C_RR[i].DOBRW();
			leg_RL_ptr[i] -> torque_bi =  leg_RL_ptr[i] -> torque_bi + C_RL[i].DOBRW();

			// Force Observer
			C_FL[i].FOBRW(); C_FR[i].FOBRW(); C_RR[i].FOBRW(); C_RL[i].FOBRW(); 
		}
		else // 둘다 동시에 
		{
			for(int i = 0 ; i < 2; i++)
			{
				C_FL[i].ctrl_update(); C_FR[i].ctrl_update(); C_RR[i].ctrl_update(); C_RL[i].ctrl_update();
				T_FL[i].hold(); T_FR[i].hold(); T_RR[i].hold(); T_RL[i].hold();// trajectory 선택
				K_FL[i].sensor_update(i); K_FR[i].sensor_update(i); K_RR[i].sensor_update(i); K_RL[i].sensor_update(i);
				K_FL[i].model_update(); K_FR[i].model_update(); K_RR[i].model_update(); K_RL[i].model_update();
				K_FL[i].FK(); K_FR[i].FK(); K_RR[i].FK(); K_RL[i].FK();
				//pos pid
				leg_FL_ptr[i] -> torque_bi = leg_FL_ptr[i]->jacbRW_trans * C_FL[i].rw_pos_pid(); 
				leg_FR_ptr[i] -> torque_bi = leg_FR_ptr[i]->jacbRW_trans * C_FR[i].rw_pos_pid(); 
				leg_RR_ptr[i] -> torque_bi = leg_RR_ptr[i]->jacbRW_trans * C_RR[i].rw_pos_pid(); 
				leg_RL_ptr[i] -> torque_bi = leg_RL_ptr[i]->jacbRW_trans * C_RL[i].rw_pos_pid();

				//DOB
				leg_FL_ptr[i] -> torque_bi =  leg_FL_ptr[i] -> torque_bi + C_FL[i].DOBRW();
				leg_FR_ptr[i] -> torque_bi =  leg_FR_ptr[i] -> torque_bi + C_FR[i].DOBRW();
				leg_RR_ptr[i] -> torque_bi =  leg_RR_ptr[i] -> torque_bi + C_RR[i].DOBRW();
				leg_RL_ptr[i] -> torque_bi =  leg_RL_ptr[i] -> torque_bi + C_RL[i].DOBRW();

				// Force Observer
				C_FL[i].FOBRW(); C_FR[i].FOBRW(); C_RR[i].FOBRW(); C_RL[i].FOBRW(); 
			}
		}
	}
	
}

	
	
int main(int argc, char **argv)

{
	rclcpp::init(argc, argv);
	auto msg_handler = std::make_shared<MsgHandler>(); 
	for(int i = 0 ; i<2 ;i++)// leg객체 초기화 : 각각 실제 인스턴스, 시뮬레이션 인스턴스임.
	{
		leg_FL_ptr.push_back(new LegModel(0)); leg_FR_ptr.push_back(new LegModel(1));
		leg_RR_ptr.push_back(new LegModel(2)); leg_RL_ptr.push_back(new LegModel(3));
		Traj_FL.emplace_back(leg_FL_ptr[i]); Traj_FR.emplace_back(leg_FR_ptr[i]);
		Traj_RR.emplace_back(leg_RR_ptr[i]); Traj_RL.emplace_back(leg_RL_ptr[i]);
		C_FL.emplace_back(leg_FL_ptr[i]);  C_FR.emplace_back(leg_FR_ptr[i]);  
		C_RL.emplace_back(leg_RL_ptr[i]);  C_RR.emplace_back(leg_RR_ptr[i]);
		K_FL.emplace_back(msg_handler.get(), leg_FL_ptr[i]);  K_FR.emplace_back(msg_handler.get(), leg_FR_ptr[i]);
		K_RR.emplace_back(msg_handler.get(), leg_RR_ptr[i]);  K_RL.emplace_back(msg_handler.get(), leg_RL_ptr[i]);
	}
	
	
  	
	
	std::cout <<"mcl workding"<<std::endl;

	rclcpp::spin(msg_handler);
  	rclcpp::shutdown();
	return 0;
}

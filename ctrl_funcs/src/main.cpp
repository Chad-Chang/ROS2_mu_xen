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


#include "msg_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

using namespace Eigen;


int main(int argc, char **argv)
{

  	rclcpp::init(argc, argv);
	std::cout <<"mcl workding"<<std::endl;

	auto msg_handler = std::make_shared<MsgHandler>();
	
	rclcpp::spin(msg_handler);
  	rclcpp::shutdown();
	return 0;
}

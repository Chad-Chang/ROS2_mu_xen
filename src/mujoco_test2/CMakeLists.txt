cmake_minimum_required(VERSION 3.5)
project(mujoco_test2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(msg_srv_action_interface_example REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_action REQUIRED)


link_directories(${Eigen_INCLUDE_DIRS})
set(SOURCES src/sim.cpp src/main.cpp)
set(MUJOCO_PATH ) 


add_executable (mujoco_node ${SOURCES})


ament_target_dependencies(mujoco_node
			  msg_srv_action_interface_example
			  rclcpp
			  rclcpp_action
			  std_msgs)
		
target_include_directories(mujoco_node PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_3.1.6/include/ # mujoco 
      )	  


target_link_directories(mujoco_node PUBLIC       #Library directory
    ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_3.1.6/lib)



target_link_libraries(mujoco_node
    mujoco
    glfw
    dl
    libmujoco.so
    ${EIGEN_LIBRARIES})  
    
ament_target_dependencies(mujoco_node rclcpp)

    
#add_executable (publish_node src/publisher.cpp)
#ament_target_dependencies(publish_node rclcpp std_msgs)

install(TARGETS 
	mujoco_node 
	#publish_node
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

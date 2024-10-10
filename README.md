
## Installation

```
# Create your own workspace
cd ~/ & mkdir -p your_workspace/src
```

```
# Clone this package to your_workspace/src
cd ~/your_workspace/src & git clone https://github.com/MindSpaceInc/Spot-MuJoCo-ROS2.git
```

```
# Build (DO NOT remove `--symlink-install`)
colcon build --symlink-install 
```

```
# Setup env
source ../install/setup.bash
```

```
# Run simulation 
ros2 launch mujoco simulation_launch.py
```

Now your can open a new terminal and use command `ros2 topic list` to see the below topics
```
/parameter_events
/rosout
/simulation/actuators_cmds
/simulation/imu_data
/simulation/joint_states
/simulation/odom

```


## 파일 설명 

1. communication : msg, srv, action 정의 
2. core : 
3. description : 모델 xml정의 
4. real_robot : xenomai 쓰레드 기반 elmo contorller
5. simulation : mujoco simulation node : msghandler thread, simulation thread로 이루어짐.
6. common_node : kinematics/Dynamics/Control/logging/trajectory/optimization(online)...

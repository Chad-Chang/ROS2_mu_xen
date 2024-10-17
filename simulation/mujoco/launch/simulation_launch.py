import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
#    xml_file_name = "model/xml/spot_mini/spot_mini.xml"
    xml_file_name = "model/scene.xml"
    xml_file = os.path.join(get_package_share_path("description"), xml_file_name)
    
#    ctrl_param_dir = LaunchConfiguration('ctrl_param_dir',default=os.path.join(
 #   	get_package_share_path('mcl'),
  #  	'param','ctrl_param_config.yaml'))
#    ctrl_param_dir = os.path.join(get_package_share_directory('mcl'),'param','ctrl_param_config.yaml')
    ctrl_param_dir = "/home/chad/robot_ws_humble/src/MCL_quad/ctrl_funcs/param/ctrl_param_config.yaml"
    
    return LaunchDescription(
        [
            Node(
                package="mujoco",
                executable="simulation",
                name="simulation_mujoco",
                output="screen",
                parameters=[
                    {"simulation/model_file": xml_file},
                    {"simulation/model_file": xml_file},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],),

            Node(
            	package="mcl",
            	executable="mcl_node",
            	name="mcl",
            	parameters=[ctrl_param_dir],
            	output="screen",),
        ]
    )

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():

    use_rviz = LaunchConfiguration('rviz', default=False)
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam_toolbox = LaunchConfiguration('slam_toolbox', default=False)

    gz_launch_path = os.path.join(get_package_share_path("ros_gz_sim"),
                                  'launch')

    gz_bridge_config_path = os.path.join(get_package_share_path("differential_robot_bringup"),
                                         'config', 'gz_bridge.yaml')
    
    gz_world_path = os.path.join(get_package_share_path('differential_robot_bringup'),
                                 'worlds', 'my_world.sdf')
    # gz_world_path = "empty.sdf"
    
    urdf_path = os.path.join(get_package_share_path("differential_robot_description"),
                             'urdf', 'differential_robot.urdf.xacro')

    rviz_config_path = os.path.join(get_package_share_path("differential_robot_description"),
                                    'rviz', 'rviz_config.rviz')
    
    # gz_args = f'{gz_world_path} -r'
    # Declare launch arguments for Xacro parameters
    gz_args = DeclareLaunchArgument(
        'gz_args', 
        default_value= f'{gz_world_path} -r',
        description='Parameter file with all property values of the robot'
    )

    run_args = LaunchConfiguration('gz_args')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type= str)

    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gz_launch_path,
            "/gz_sim.launch.py"
        ]), launch_arguments={'gz_args': run_args}.items()
    )

    gz_entity_create_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description']
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file':gz_bridge_config_path}]
    )

    rviz_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)    
    )

    ## SLAM Toolbox
    slam_toolbox_path = os.path.join(get_package_share_path('slam_toolbox'),
                                    'launch')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            slam_toolbox_path,
            "/online_async_launch.py"
        ]), launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(use_slam_toolbox)
    )

    ## Navigation
    nav2_bringup_path = os.path.join(get_package_share_path('nav2_bringup'), 
                                     'launch')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_path,
            "/navigation_launch.py"
        ]), launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(use_nav)
        )

    return LaunchDescription([
        gz_args,
        robot_state_publisher,
        gz_sim_launch,
        gz_entity_create_node,
        gz_bridge_node,
        rviz_node,
        slam_toolbox,
        navigation
    ])
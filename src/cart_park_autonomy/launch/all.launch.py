from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 경로 가져오기
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_dir = os.path.join(realsense_dir, 'launch')
    # slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    # slam_toolbox_launch_dir = os.path.join(slam_toolbox_dir, 'launch')

    # 7. odrive
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("odrive_demo_description"),
                    "urdf",
                    "odrive_diffbot.urdf.xacro"
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_demo_bringup"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("my_bot_description"),
            "rviz",
            "diffbot.rviz"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[('diffbot_base_controller/cmd_vel_unstamped', '/cmd_vel')]
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )
    
    # 3. usb_cam
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[os.path.join(os.path.expanduser('~/my_ws/src/usb_cam/config/params_3.yaml'))],
        remappings=[('/usb_cam/image_raw', '/image_raw')]
    )
    
    usb_cam_node_2 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[os.path.join(os.path.expanduser('~/my_ws/src/usb_cam/config/params_2.yaml'))],
        remappings=[('/usb_cam/image_raw', '/image_raw')]
    )
    
    # 4. aruco marker - usb_cam
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node_marker',
        parameters=[{'image_topic': '/image_raw'}, {'camera_info_topic': '/camera_info'}, {'marker_size': 0.2}],
    )
    
    # 2. aruco marker - realsense
    aruco_node_camera = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node_camera',
        parameters=[{'image_topic': '/camera/camera/color/image_raw'}, {'camera_info_topic': '/camera/camera/color/camera_info'}, {'marker_size': 0.2}],
    )
    
    # 5. my_tf2_package
    my_static_cart_tf2_broadcaster_node = Node(
            package='my_tf2_package',
            executable='my_static_cart_tf2_broadcaster'
    )

    # 6. lidar
    ydlidar_ros2_driver_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node'
    )

    open_action_server_node = Node(
            package='open_action_server',
            executable='open_action_server'
    )

    # 1. realsense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, 'rs_launch.py'))
    )
    # 8. slam_toolbox
    # slam_toolbox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(slam_toolbox_launch_dir, 'online_async_launch.py')),
    #     launch_arguments={'params_file': './src/slam_toolbox/config/mapper_params_online_async.yaml'}.items()
    # )
    # 9. rviz
    # rviz_node2 = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=["-d", rviz_config_file2],
    # )



    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
        usb_cam_node,
        usb_cam_node_2,
        aruco_node,
        aruco_node_camera,
        my_static_cart_tf2_broadcaster_node,
        ydlidar_ros2_driver_node,
        open_action_server_node,
        realsense_launch
    ])

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare(
                "ptz_robot"  # MUST match the package name in "package.xml", the name used to create the package
            ),
            "urdf",
            "ptz_robot.urdf.xacro",  # MUST be the exact main ".urdf.xarco" file describing your robot
        ]
    )

    # Run xacro and **explicitly** say: this is a string parameter
    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_file]), value_type=str
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,  # add other parameters here if required
            }
        ],
        output="screen",
    )

    # Launch Gazebo Sim (.sdf World)
    world_file_arg = DeclareLaunchArgument(  # define an argument named 'world' with a default value of 'empty.sdf'
        "world",
        default_value="empty.sdf",
        description="Gazebo world .sdf file name"
    )
    # Create a variable name substitution to access the value of the argument
    world_file = LaunchConfiguration("world")
    world_name = PythonExpression(["'", world_file, "'.removesuffix('.sdf')"])

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ["-r", " ", world_file]}.items(),
    )

    # Spawner node to spawn robot model in Gazebo Sim
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", world_name,
            "-name", "my_robot",
            "-topic", "/robot_description",
            '-z', '4'
        ],
        output="screen",
    )

    # ROS-Gazebo bridge: connects the Gazebo camera topic to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Camera Image (Gazebo -> ROS)
            '/ptz/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image]',
            # Camera Info (Gazebo -> ROS)
            '/ptz/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo]',
            # Joint States (Gazebo -> ROS)
            '/world/empty/model/ptz_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model]',
        ],
        output='screen'
    )
    # arguments=[
    #     # Joint command bridges
    #     '/cmd_pan@std_msgs/msg/Float64@ignition.msgs.Double[ROS_TO_IGN]',
    #     '/cmd_tilt@std_msgs/msg/Float64@ignition.msgs.Double[ROS_TO_IGN]',
    #     '/cmd_zoom@std_msgs/msg/Float64@ignition.msgs.Double[ROS_TO_IGN]',
    #     # Camera data bridges
    #     # (Note: replace <world_name> and <model_name> with actual names; 'default' is default world)
    #     '/world/default/model/ptz_camera/link/camera_link/sensor/camera_sensor/image@sensor_msgs/msg/Image@ignition.msgs.Image[IGN_TO_ROS]',
    #     '/world/default/model/ptz_camera/link/camera_link/sensor/camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo[IGN_TO_ROS]'
    # ]),

    # PTZ control node (NBV + IBVS logic)
    controller_node = Node(
        package='ptz_control_package', executable='ptz_controller_node', output='screen',
        parameters=[{'target_class': 'bottle'}]  # example parameter for target class name
    ),
    # YOLO detector node (assumed to be implemented elsewhere, listening on /camera/image_raw)
    yolo_node = Node(
        package='yolov5_ros', executable='yolov5_detector', output='screen',
        parameters=[{'confidence_thresh': 0.5}]
    )

    # We can also include a launch description to launch Gazebo directly from this launch file

    return LaunchDescription(
        [
            world_file_arg,
            rsp_node,
            gazebo_sim,
            spawn_entity_node,
        ]
    )

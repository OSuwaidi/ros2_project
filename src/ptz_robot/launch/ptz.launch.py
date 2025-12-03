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
            "-name", "my_robot",  # Gazebo model name
            "-topic", "/robot_description",
            '-z', '0.01',
            "-Y", "3.14"
        ],
        output="screen",
    )

    # ROS-Gazebo bridge: connects the Gazebo camera topic to ROS 2
    # image bridge: Gazebo /ptz/camera/image_raw -> ROS /ptz/camera/image_raw
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="gz_image_bridge_ptz",
        output="screen",
        arguments=[
            "/ptz/camera/image_raw",  # Gazebo image topic to bridge
        ],
    )

    # parameter bridge for camera_info + zoom joint command
    param_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_param_bridge_ptz",
        output="screen",
        arguments=[
            # Camera info: Gazebo <-> ROS
            "/ptz/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            # Zoom joint velocity command: ROS <-> Gazebo
            # "/model/my_robot/joint/zoom_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double",
        ],
    )

    # Include YOLOv8 launch (from yolo_ros / yolo_bringup) listening on /camera/image_raw)
    yolo_bringup_dir = get_package_share_directory("yolo_bringup")
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_bringup_dir, "launch", "yolov8.launch.py")
        ),
        launch_arguments={
            # Use the bridged camera topic
            "input_image_topic": "/ptz/camera/image_raw",
            # Encoding from ros_gz_image is typically rgb8 for R8G8B8 images
            "yolo_encoding": "rgb8",
            # Choose model size: yolov8n.pt / yolov8s.pt / ...
            "model": "yolov8n.pt",
            # Optional extra params (only if you want to override defaults):
            "threshold": "0.6",
            "device": "cpu",
        }.items(),
    )

    # Visualize camera/YOLO image topics:
    rqt_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        output="screen",
    )

    # PTZ control node (NBV + IBVS logic)
    # controller_node = Node(
    #     package='ptz_control_package', executable='ptz_controller_node', output='screen',
    #     parameters=[{'target_class': 'bottle'}]  # example parameter for target class name
    # ),

    return LaunchDescription(
        [
            world_file_arg,
            rsp_node,
            gazebo_sim,
            spawn_entity_node,
            image_bridge,
            param_bridge,
            yolov8_launch,
            rqt_node
        ]
    )

import os
import xacro
import random
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Point to the package root (first package name directory inside of "src": "~/<name_ws>/src/<pkg_name>")
    pkg_dir = get_package_share_directory("ptz_robot")  # MUST match the package name in "package.xml", the name used to create the package
    xacro_file = os.path.join(pkg_dir, "urdf", "ptz_robot.urdf.xacro")

    # Process file via xacro and **explicitly** generate URDF as a string
    robot_description = xacro.process_file(xacro_file).toxml()

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
        name="world",
        default_value="empty.sdf",
        description="Gazebo world .sdf file name"
    )
    # Create a variable name substitution to access the value of the argument
    world_file = LaunchConfiguration("world")
    world_file_pth = PathJoinSubstitution([pkg_dir, "worlds", world_file])

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ["-r", " ", world_file_pth]}.items(),
    )

    # Spawner node to spawn robot model in Gazebo Sim
    world_name = PythonExpression(["'", world_file, "'.removesuffix('.sdf')"])
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", world_name,
            "-name", "my_robot",  # Gazebo model name
            "-topic", "/robot_description",
            '-z', '0.1',
            '-x', "0.0",
            "-y", "0.0",
            "-Y", "0.0"  # yaw
        ],
        output="screen",
    )

    # Spawner node to spawn target object in Gazebo Sim
    balloon_file = os.path.join(pkg_dir, "models", "balloon.sdf")
    spawn_balloon_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", world_name,
            "-name", "red_balloon",
            "-file", balloon_file,
            '-x', str(random.uniform(-5.0, 5.0)),
            "-y", str(random.uniform(-5.0, 5.0)),
            "-z", str(random.uniform(1.0, 5.0))
        ],
        output="screen",
    )

    # Image bridge connecting Gazebo Transport topic: /ptz/camera/image_raw -> republished as ROS 2 topic
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="gz_image_bridge_ptz",
        output="screen",
        arguments=[
            "/ptz/camera/image_raw",
        ],
    )

    # Parameter bridge for camera_info + joint commands (expose them to ROS)
    param_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_param_bridge_ptz",
        output="screen",
        arguments=[
            # Camera info: Gazebo <-> ROS
            "/ptz/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            # Joint velocity commands: Gazebo <-> ROS (to be able to drive joints from ROS)
            "/model/my_robot/joint/pan_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double",
            "/model/my_robot/joint/tilt_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double",
            "/model/my_robot/joint/zoom_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double",
            # Joint states Gazebo <-> ROS publishing
            "/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model"
        ],
    )

    # Include YOLOv8 launch (via yolo_bringup) listening on /camera/image_raw
    yolo_bringup_dir = get_package_share_directory("yolo_bringup")
    yoloe_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_bringup_dir, "launch", "yoloe.launch.py")
        ),
        launch_arguments={
            # Use the bridged camera topic
            "input_image_topic": "/ptz/camera/image_raw",
            "yolo_encoding": "rgb8",
            "model": "yoloe-11l-seg-pf.pt",
            "threshold": "0.45",
            "iou": "0.1",
            "device": "cpu",
            "max_det": "5",
            "use_tracking": "False",
            "imgsz_width": "320",
            "imgsz_height": "240",
            # "image_reliability": "2",  # "Best Effort QoS"
        }.items(),
    )

    yolo_image_view = Node(
        package="image_view",
        executable="image_view",
        name="yolo_image_view",
        output="screen",
        remappings=[
            ("image", "/yolo/dbg_image"),
        ],
        parameters=[
            {"window_name": "YOLO Debug Image"},
        ],
    )

    camera_compressed_view = Node(
        package="image_view",
        executable="image_view",
        name="camera_compressed_view",
        output="screen",
        remappings=[
            ("image", "/ptz/camera/image_raw"),
        ],
        parameters=[
            {"transport": "compressed"},
            {"window_name": "PTZ Camera (Compressed)"},
        ],
    )

    # Visualize camera/YOLO image topics (relatively heavier):
    # rqt_node = Node(
    #     package="rqt_image_view",
    #     executable="rqt_image_view",
    #     output="screen",
    # )

    # PTZ control node
    controller_node = Node(
        package='ptz_controller',
        executable='controller',  # class name
        name="ptz_controller",
        output='screen',
        # parameters=[
        #     {
        #         'target_class': 'balloon',
        #     }
        # ]
    )

    return LaunchDescription(
        [
            world_file_arg,
            rsp_node,
            gazebo_sim,
            spawn_robot_node,
            spawn_balloon_node,
            image_bridge,
            param_bridge,
            yoloe_launch,
            yolo_image_view,
            camera_compressed_view,
            controller_node
        ]
    )

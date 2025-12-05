# ptz_controller/controller_node.py

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Float64

# yolo_ros message type (from mgonzs13/yolo_ros)
from yolo_msgs.msg import DetectionArray


class NextBestViewController(Node):
    def __init__(self):
        super().__init__("controller")

        # ---------------- Parameters ----------------
        self.declare_parameter("target_class", "balloon")

        # Base search parameters (MUST be floats!)
        self.declare_parameter("pan_vel", 0.25)  # base pan speed (rad/s)
        self.declare_parameter("tilt_vel", 0.25)  # base tilt speed (rad/s)

        # How often to refresh/randomize the pattern if still searching
        self.declare_parameter("search_pattern_refresh_time", 30.0)  # seconds

        # Tilt joint soft limits (URDF: lower=-1.2, upper=0.5)
        self.declare_parameter("tilt_lower_limit", -1.2)
        self.declare_parameter("tilt_upper_limit", 0.5)

        # ---- Read parameters ----
        target_class = self.get_parameter("target_class").get_parameter_value().string_value
        self.target_class = (target_class, "sphere") if target_class == "balloon" else (target_class,)
        self.pan_vel = self.get_parameter("pan_vel").get_parameter_value().double_value
        self.tilt_vel = self.get_parameter("tilt_vel").get_parameter_value().double_value
        self.zoom_vel = 0.1
        self.search_pattern_refresh_time = self.get_parameter("search_pattern_refresh_time").get_parameter_value().double_value

        # Tilt limits
        tilt_lower_limit = self.get_parameter("tilt_lower_limit").get_parameter_value().double_value
        tilt_lower_buffer = 0.6
        tilt_upper_limit = self.get_parameter("tilt_upper_limit").get_parameter_value().double_value
        tilt_upper_buffer = 0.1

        # Precompute soft limits
        self.soft_tilt_lower = tilt_lower_limit + tilt_lower_buffer
        self.soft_tilt_upper = tilt_upper_limit - tilt_upper_buffer

        # ---- State ----
        self.image_width = None
        self.image_height = None
        self.current_tilt_position = None  # from /joint_states
        self.pan_cmd = Float64()
        self.tilt_cmd = Float64()
        self.zoom_cmd = Float64()


        # Flags & pattern state
        self.target_found = False
        self.tilt_direction = 1.0
        self.last_pattern_change = self.get_clock().now()

        # ---------------- Publishers ----------------
        self.pan_pub = self.create_publisher(
            Float64,
            "/model/my_robot/joint/pan_joint/cmd_vel",
            10,
        )
        self.tilt_pub = self.create_publisher(
            Float64,
            "/model/my_robot/joint/tilt_joint/cmd_vel",
            10,
        )
        self.zoom_pub = self.create_publisher(
            Float64,
            "/model/my_robot/joint/zoom_joint/cmd_vel",
            10,
        )

        # ---------------- Our Subscriptions ----------------
        # Callbacks trigger everytime a message is published on our subscribed topics
        self.create_subscription(
            CameraInfo,
            "/ptz/camera/camera_info",
            self.camera_info_callback,
            10,
        )

        self.create_subscription(
            DetectionArray,
            "/yolo/detections",
            self.detections_callback,
            10,
        )

        # subscribe to joint_states so we know where the tilt joint is
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        # ---------------- Timer (control loop) ----------------
        self.control_timer = self.create_timer(0.02, self.control_loop)  # runs at 20 Hz (times per second)

        self.get_logger().info(
            f"PTZ NBV controller started.\n"
            f"base_pan_vel={self.pan_vel:.3f} rad/s\n"
            f"tilt_base={self.tilt_vel:.3f} rad/s\n"
            f"pattern_refresh={self.search_pattern_refresh_time:.1f} s\n"
            f"target_class='{self.target_class}'."
        )

    # ---------- Helpers for randomization ----------

    def randomize_search_pattern(self):
        """
        Pick a new random search pattern:
        - pan speed varies around base
        - tilt speed & switch time vary around base
        - random initial directions for pan & tilt
        """
        pan_scale = random.uniform(0.8, 1.3)
        self.pan_vel = self.pan_vel * pan_scale

        tilt_scale = random.uniform(0.8, 1.3)
        self.tilt_vel = self.tilt_vel * tilt_scale

        self.last_pattern_change = self.get_clock().now()

        self.get_logger().info(
            "New search pattern: "
            f"pan_vel={self.pan_vel:.3f} rad/s, "
            f"tilt_vel={self.tilt_vel:.3f} rad/s, "
        )

    # ---------- Callbacks ----------

    def camera_info_callback(self, msg: CameraInfo):
        self.image_width = msg.width
        self.image_height = msg.height

    def joint_state_callback(self, msg: JointState):
        """
        Track the current tilt joint position so we can avoid hitting hard limits.
        """
        for name, pos in zip(msg.name, msg.position):
            if name == "tilt_joint":
                self.current_tilt_position = pos
                return

    def detections_callback(self, msg: DetectionArray):
        """
        Only treat detections of the TARGET CLASS as meaningful.
        Other classes are ignored.
        """
        if not self.target_found:
            if msg.detections:
                for det in msg.detections:
                    if det.class_name in self.target_class:
                        self.get_logger().info(f"Target '{self.target_class}' detected. Stopping scan & starting zoom.")
                        self.target_found = True
                        return

                # If we get here: detections exist, but none match target_class -> ignore them.

    def control_loop(self):
        """
        Runs at 20 Hz (20 runs per second in a continuous loop).
        - Search = continuous pan + oscillating tilt, with randomized pattern,
          but avoid pushing tilt joint into hard limits.
        - If target seen -> stop pan/tilt, zoom in.
        """
        now = self.get_clock().now()

        if not self.pan_cmd.data and not self.target_found:  # initially has a value of 0
            self.get_logger().info("Entering search mode.")

        if not self.target_found:
            # --- SEARCH MODE ---
            # Periodically refresh the pattern
            if (now - self.last_pattern_change).nanoseconds / 1e9 > self.search_pattern_refresh_time:
                self.randomize_search_pattern()

            # Continuous pan
            self.pan_cmd.data = self.pan_vel

            # Handle tilt direction based on joint soft limits first
            if self.current_tilt_position is not None:
                if self.current_tilt_position >= self.soft_tilt_upper:
                    # Too close to upper limit, tilt down instead
                    self.tilt_direction = -1.0
                elif self.current_tilt_position <= self.soft_tilt_lower:
                    # Too close to lower limit, tilt up instead
                    self.tilt_direction = 1.0

            # Apply tilt velocity
            self.tilt_cmd.data = self.tilt_vel * self.tilt_direction

            # Publish commands
            self.pan_pub.publish(self.pan_cmd)
            self.tilt_pub.publish(self.tilt_cmd)

        else:
            # --- DETECTION MODE: target recently seen ---
            self.pan_cmd.data = 0.0
            self.tilt_cmd.data = 0.0
            self.pan_pub.publish(self.pan_cmd)
            self.tilt_pub.publish(self.tilt_cmd)

            self.zoom_cmd.data = self.zoom_vel
            self.zoom_pub.publish(self.zoom_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NextBestViewController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

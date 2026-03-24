import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from enum import Enum


class RobotState(Enum):
    NAV_TO_DIG = 1
    DIG = 2
    NAV_TO_DUMP = 3
    DUMP = 4


class CentralController(Node):
    def __init__(self):
        super().__init__('central_controller')

        # Parameters for zones (replace with arena config)
        self.declare_parameter('dig_zone_center', [2.0, 2.0])
        self.declare_parameter('dump_zone_center', [8.0, 2.0])

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, 'pose_in', self.pose_callback, 10)

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 10)

        self.arm_pub = self.create_publisher(
            Float64, 'regolith_arm_angle', 10)

        self.drum_pub = self.create_publisher(
            Float64, 'regolith_drum_speed', 10)

        # Internal state
        self.map = None
        self.pose = None
        self.state = RobotState.NAV_TO_DIG

    # -----------------------------
    # Callbacks
    # -----------------------------
    def map_callback(self, msg):
        self.map = msg

    def pose_callback(self, msg):
        self.pose = msg
        self.update_state_machine()

    # -----------------------------
    # State Machine Logic
    # -----------------------------
    def update_state_machine(self):
        if self.map is None or self.pose is None:
            return

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        dig_x, dig_y = self.get_parameter('dig_zone_center').value
        dump_x, dump_y = self.get_parameter('dump_zone_center').value

        # --- State transitions ---
        if self.state == RobotState.NAV_TO_DIG:
            if self.is_near(x, y, dig_x, dig_y):
                self.state = RobotState.DIG
            else:
                self.publish_goal(dig_x, dig_y)

        elif self.state == RobotState.DIG:
            self.publish_digging_commands()
            # Example: after digging for a while, switch state
            self.state = RobotState.NAV_TO_DUMP

        elif self.state == RobotState.NAV_TO_DUMP:
            if self.is_near(x, y, dump_x, dump_y):
                self.state = RobotState.DUMP
            else:
                self.publish_goal(dump_x, dump_y)

        elif self.state == RobotState.DUMP:
            self.publish_dumping_commands()
            # After dumping, return to dig zone
            self.state = RobotState.NAV_TO_DIG

    # -----------------------------
    # Helper functions
    # -----------------------------
    def is_near(self, x, y, gx, gy, tol=0.3):
        return abs(x - gx) < tol and abs(y - gy) < tol

    def publish_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        self.goal_pub.publish(goal)

    def publish_digging_commands(self):
        arm = Float64()
        drum = Float64()
        arm.data = -0.5   # lower arm
        drum.data = 5.0   # spin drum
        self.arm_pub.publish(arm)
        self.drum_pub.publish(drum)

    def publish_dumping_commands(self):
        arm = Float64()
        drum = Float64()
        arm.data = 1.0    # raise arm
        drum.data = -3.0  # reverse drum
        self.arm_pub.publish(arm)
        self.drum_pub.publish(drum)


def main(args=None):
    rclpy.init(args=args)
    node = CentralController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

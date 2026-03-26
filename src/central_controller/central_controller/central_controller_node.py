# loads ros2 library and the Node class
import rclpy
from rclpy.node import Node

# imports message types the node subscribes and publishes to
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float64

# enum is used for the robot's state-machine, random for simple exploration behavior
from enum import Enum
import random

# all robot behaviors, switches depending on sensor input
class RobotState(Enum):
    SEARCH_FOR_DIG_ZONE = 0
    NAV_TO_DIG = 1
    DIG = 2
    SEARCH_FOR_DUMP_ZONE = 3
    NAV_TO_DUMP = 4
    DUMP = 5

# creates a ros2 node named central_controller
class CentralController(Node):
    def __init__(self):
        super().__init__('central_controller')

        # -----------------------------
        # subscribers (inputs)
        # -----------------------------
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, 'pose_in', self.pose_callback, 10)

        # Perception topics for zone detection
        self.dig_zone_sub = self.create_subscription(
            Point, 'dig_zone_center', self.dig_zone_callback, 10)

        self.dump_zone_sub = self.create_subscription(
            Point, 'dump_zone_center', self.dump_zone_callback, 10)

        # -----------------------------
        # publishers (outputs)
        # -----------------------------
        self.goal_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 10)

        self.arm_pub = self.create_publisher(
            Float64, 'regolith_arm_angle', 10)

        self.drum_pub = self.create_publisher(
            Float64, 'regolith_drum_speed', 10)

        # -----------------------------
        # internal state
        # -----------------------------
        self.map = None
        self.pose = None

        self.dig_zone = None
        self.dump_zone = None

        self.state = RobotState.SEARCH_FOR_DIG_ZONE

    # ============================================================
    # callbacks (storing and updating data)
    # ============================================================
    def map_callback(self, msg):
        self.map = msg

    def pose_callback(self, msg):
        self.pose = msg
        self.update_state_machine()

    def dig_zone_callback(self, msg: Point):
        self.dig_zone = (msg.x, msg.y)
        self.get_logger().info(f"Dig zone detected at {self.dig_zone}")

    def dump_zone_callback(self, msg: Point):
        self.dump_zone = (msg.x, msg.y)
        self.get_logger().info(f"Dump zone detected at {self.dump_zone}")

    # ============================================================
    # state-machine logic
    # ============================================================
    def update_state_machine(self):
        if self.pose is None:
            return

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # -------------------------
        # if dig_zone is known, go there. otherwise, search randomly
        # -------------------------
        if self.state == RobotState.SEARCH_FOR_DIG_ZONE:
            if self.dig_zone is not None:
                self.get_logger().info("→ NAV_TO_DIG")
                self.state = RobotState.NAV_TO_DIG
                return
            self.publish_exploration_goal()
            return

        # -------------------------
        # if close to dig_zone, start digging. otherwise, keep sending navigation goals
        # -------------------------
        if self.state == RobotState.NAV_TO_DIG:
            dig_x, dig_y = self.dig_zone
            if self.is_near(x, y, dig_x, dig_y):
                self.get_logger().info("→ DIG")
                self.state = RobotState.DIG
            else:
                self.publish_goal(dig_x, dig_y)
            return

        # -------------------------
        # lowers arms, spins drum, then searches where to dump (state-machine)
        # -------------------------
        if self.state == RobotState.DIG:
            self.publish_digging_commands()
            self.get_logger().info("→ SEARCH_FOR_DUMP_ZONE")
            self.state = RobotState.SEARCH_FOR_DUMP_ZONE
            return

        # -------------------------
        # if dump_zone is known, go there. Otherwise, search randomly
        # -------------------------
        if self.state == RobotState.SEARCH_FOR_DUMP_ZONE:
            if self.dump_zone is not None:
                self.get_logger().info("→ NAV_TO_DUMP")
                self.state = RobotState.NAV_TO_DUMP
                return
            self.publish_exploration_goal()
            return

        # -------------------------
        # if close to dump_zone, dump. Otherwise, keep sending navigation goals
        # -------------------------
        if self.state == RobotState.NAV_TO_DUMP:
            dump_x, dump_y = self.dump_zone
            if self.is_near(x, y, dump_x, dump_y):
                self.get_logger().info("→ DUMP")
                self.state = RobotState.DUMP
            else:
                self.publish_goal(dump_x, dump_y)
            return

        # -------------------------
        # raises arms, reverses drum, searches for next dig zone (state-machine)
        # -------------------------
        if self.state == RobotState.DUMP:
            self.publish_dumping_commands()
            self.get_logger().info("→ SEARCH_FOR_DIG_ZONE")
            self.state = RobotState.SEARCH_FOR_DIG_ZONE
            return

    # ============================================================
    # helper functions
    # ============================================================
    def is_near(self, x, y, gx, gy, tol=0.3):
        return abs(x - gx) < tol and abs(y - gy) < tol # distance check

    def publish_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        self.goal_pub.publish(goal) # sends a PoseStamped goal to path generator

    def publish_exploration_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map' # random walking until zone is found

        # simple random exploration (replace with frontier exploration later)
        goal.pose.position.x = random.uniform(0.0, 10.0)
        goal.pose.position.y = random.uniform(0.0, 10.0)

        self.goal_pub.publish(goal)
        self.get_logger().info(
            f"Exploring... new goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})"
        )

    def publish_digging_commands(self): # digging arm and drum commands
        arm = Float64()
        drum = Float64()
        arm.data = -0.5   # lower arm
        drum.data = 5.0   # spin drum forward
        self.arm_pub.publish(arm)
        self.drum_pub.publish(drum)

    def publish_dumping_commands(self): # dumping arm and drum commands
        arm = Float64()
        drum = Float64()
        arm.data = 1.0    # raise arm
        drum.data = -3.0  # reverse drum
        self.arm_pub.publish(arm)
        self.drum_pub.publish(drum)


def main(args=None): # main function that takes in command-line args
    rclpy.init(args=args) # initializes ros2 client library
    node = CentralController() # creates an instance of the node class
    rclpy.spin(node) # event loop, "keep this node up and run its callbacks when messages come"
    node.destroy_node() # removes node when program shuts down
    rclpy.shutdown() # shuts down ros2 client library


if __name__ == '__main__': # only run main if this file is executed directly
    main()

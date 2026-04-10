# UNTESTED, found this on the internet and it may work for testing.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class FakeFollower(Node):
    def __init__(self):
        super().__init__('fake_follower')

        self.goal = None
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_in', 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)

        self.timer = self.create_timer(0.1, self.update_pose)

        # start at (0,0)
        self.x = 0.0
        self.y = 0.0

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"New goal received: {self.goal}")

    def update_pose(self):
        if self.goal is None:
            return

        gx, gy = self.goal
        dx = gx - self.x
        dy = gy - self.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.05:
            # reached goal
            pass
        else:
            # move 0.05 m per tick
            step = 0.05
            self.x += step * dx/dist
            self.y += step * dy/dist

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = FakeFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

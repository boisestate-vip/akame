#
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import LaserScan
import math

Class FakeScan(Node):
    def __init__(self):
      super().__init__('fake scanner')
  
      self.goal = None
      self.odom_pub = self.create_publisher(Odometry, '/demo/odom', 10)
      self.scan_pub = self.create_publisher(LaserScan, '/demo/scan', 10)
  
    def update_scan(self):
      odom = Odometry() 
      odom.header.frame_id = 'odom' 
      odom.pose.pose.position.x = self.x 
      odom.pose.pose.position.y = self.y 
      self.odom_pub.publish(odom) 
  
    def publish_scan(self):
      scan = LaserScan() 
      scan.header.frame_id = "base_link" 
      scan.angle_min = -1.57 
      scan.angle_max = 1.57 
      scan.angle_increment = 0.01 
      scan.range_min = 0.0 
      scan.range_max = 10.0 
      scan.ranges = [10.0] * 300 scan_pub.publish(scan) 

def main(args=None):
    rclpy.init(args=args)
    node = FakeScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

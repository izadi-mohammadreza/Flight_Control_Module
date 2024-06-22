import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

class CirclePublisherNode(Node):
    def __init__(self):
        super().__init__('circle_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'command/pose', 10)
        timer_period = 0.01  # seconds
        self.timer_ = self.create_timer(timer_period, self.publish_circle_pose)
        self.angle = 0.0
        

    def publish_circle_pose(self):
        radius = 1.0
        ww = 1 * np.pi/4
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'base_link'  # Change this to your desired frame ID

        pose_stamped.pose.position.x = radius * math.cos( ww*self.angle)
        pose_stamped.pose.position.y = radius * math.sin(ww*self.angle)
        
        #pose_stamped.pose.position.x = -1.0
        #pose_stamped.pose.position.y = 2.0
        pose_stamped.pose.position.z = 1.0
        pose_stamped.pose.orientation.w = 1.0


        self.publisher_.publish(pose_stamped)

        self.angle += 0.01  # Change this value to control the angular speed of the circular path

def main(args=None):
    rclpy.init(args=args)
    circle_publisher_node = CirclePublisherNode()
    rclpy.spin(circle_publisher_node)
    circle_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

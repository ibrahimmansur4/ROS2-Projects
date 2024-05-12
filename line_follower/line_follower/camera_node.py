import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy 


class MinimalSubscriber(Node):
	def __init__(self):
		super().__init__('image_subscriber')
		self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
		self.subscription
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.move = Twist()
		self.br = CvBridge()
		
	def listener_callback(self, data):
		self.get_logger().info('Receiving video frame')
		img = self.br.imgmsg_to_cv2(data, 'bgr8')
		img = cv2.resize(img, None, 1, 0.5, 0.5, cv2.INTER_CUBIC)
		cv2.imshow('Camera', img)
		cv2.waitKey(2)
		


def main(args = None):
	rclpy.init(args = args)
	simple_sub = MinimalSubscriber()
	rclpy.spin(simple_sub)
	simple_sub.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()

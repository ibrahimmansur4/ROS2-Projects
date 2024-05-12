import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class LineFollower(Node):
    def _init_(self):
        super()._init_("task_3")
        self.subscription = self.create_subscription(
            Image, "camera/image_raw", self.callback, 10
        )
        self.subscription
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.move = Twist()
        self.br = CvBridge()

        # Proportional control parameters
        self.Kp = 0.01  # Proportional gain

    def callback(self, msg):
        self.get_logger().info("Receiving video frame")
        img = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = cv2.resize(img, None, 1, 0.2, 0.2, cv2.INTER_CUBIC)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        twist = Twist()

        linear_x = 0.3
        h_min = 15
        h_max = 73
        s_min = 25
        s_max = 255
        v_min = 35
        v_max = 255

        # Making the mask
        low = np.array([h_min, s_min, v_min])
        upp = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(img_hsv, low, upp)
        result = cv2.bitwise_and(img, img, mask=mask)

        # Region of interest (keep the lower 1/5 of the image)
        height, width = mask.shape
        roi = deepcopy(mask)
        roi[: 4 * height // 5, :] = 0

        # Calculate centroid of the region of interest
        M = cv2.moments(roi)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = width // 2, (4 * height // 5) // 2  # Default to center

        center_roi = (width // 2, (4 * height // 5) // 2)
        distance_center = cX - center_roi[0]

        # Proportional control to adjust the angular velocity
        angular_z = -self.Kp * distance_center

        # Move the robot based on the distance from the center of the region of interest
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if _name_ == "_main_":
    main()

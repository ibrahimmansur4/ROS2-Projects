import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleSimController(Node):

    def __init__(self):
        super().__init__('turtle_sim_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.twist_msg = Twist()

    def timer_callback(self):
        # Move the turtle in a circle
        self.twist_msg.linear.x = 1.0  # Linear velocity in the x-axis
        self.twist_msg.angular.z = 1.0  # Angular velocity around the z-axis
        self.publisher_.publish(self.twist_msg)
        self.get_logger().info('Moving turtle in a circle')

def main(args=None):
    rclpy.init(args=args)

    turtle_sim_controller = TurtleSimController()

    rclpy.spin(turtle_sim_controller)

    turtle_sim_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

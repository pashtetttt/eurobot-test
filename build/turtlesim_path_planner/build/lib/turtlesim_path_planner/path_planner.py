# path_planner.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import sin, cos, pi
from std_srvs.srv import Empty

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Parameters to select trajectory
        self.declare_parameter('trajectory', 'circle')
        self.trajectory = self.get_parameter('trajectory').get_parameter_value().string_value

        # Publisher to cmd_vel to control the turtle's movement
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create timer to publish velocity commands at a fixed rate
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Reset service to reposition the turtle
        self.reset_service = self.create_client(Empty, '/reset')

        # Time elapsed for movement control
        self.time_elapsed = 0.0

        # Wait for reset service to be available
        while not self.reset_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reset service...')

        # Call the reset service to initialize the turtle's position
        self.reset_turtle()

    def reset_turtle(self):
        req = Empty.Request()
        self.reset_service.call_async(req)

    def timer_callback(self):
        # Publish velocity based on the chosen trajectory
        if self.trajectory == 'circle':
            self.move_in_circle()
        elif self.trajectory == 'square':
            self.move_in_square()
        elif self.trajectory == 'infinity':
            self.move_in_infinity()
        elif self.trajectory == 'star':
            self.move_in_star()

        self.time_elapsed += 0.1

    def move_in_circle(self):
        """Moves the turtle in a circular path."""
        velocity_msg = Twist()
        velocity_msg.linear.x = 2.0  # Move forward
        velocity_msg.angular.z = 1.0  # Constant angular velocity to create a circle
        self.velocity_publisher.publish(velocity_msg)

    def move_in_square(self):
        """Moves the turtle in a square path."""
        velocity_msg = Twist()
        side_duration = 2.0  # time to move along one side
        turn_duration = 1.0  # time to make a 90-degree turn

        # Move forward for side_duration, then turn
        if int(self.time_elapsed) % 3 == 0:
            velocity_msg.linear.x = 2.0
            velocity_msg.angular.z = 0.0
        elif int(self.time_elapsed) % 3 == 2:
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = pi / 2  # 90 degree turn

        self.velocity_publisher.publish(velocity_msg)

    def move_in_infinity(self):
        """Moves the turtle in an infinity (figure 8) path."""
        velocity_msg = Twist()
        velocity_msg.linear.x = 2.0 * cos(self.time_elapsed)
        velocity_msg.angular.z = 2.0 * sin(self.time_elapsed)
        self.velocity_publisher.publish(velocity_msg)

    def move_in_star(self):
        """Moves the turtle in a star-shaped path."""
        velocity_msg = Twist()
        star_duration = 5.0  # Duration for each star segment
        if int(self.time_elapsed) % star_duration < 3.0:
            velocity_msg.linear.x = 2.0
            velocity_msg.angular.z = 0.0
        else:
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = pi * 2 / 5  # Star angle
        self.velocity_publisher.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


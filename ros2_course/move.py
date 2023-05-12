import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt

class MoveToXY(Node):
    def __init__(self):
        super().__init__('move_to_xy')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.subscriber_.__setattr__('_buffer_size', 1) # Workaround for https://github.com/ros2/rclpy/issues/675
        self.odometry_ = Odometry()
        self.destination_x_ = 0.0
        self.destination_y_ = 0.0
        self.threshold_distance_ = 0.1
        self.max_linear_velocity_ = 0.2
        self.max_angular_velocity_ = 1.5

    def set_destination(self, x, y):
        self.destination_x_ = x
        self.destination_y_ = y

    def odometry_callback(self, message):
        self.odometry_ = message

    def get_distance_to_destination(self):
        dx = self.destination_x_ - self.odometry_.pose.pose.position.x
        dy = self.destination_y_ - self.odometry_.pose.pose.position.y
        return sqrt(dx*dx + dy*dy)

    def get_yaw_to_destination(self):
        dx = self.destination_x_ - self.odometry_.pose.pose.position.x
        dy = self.destination_y_ - self.odometry_.pose.pose.position.y
        return atan2(dy, dx)

    def run(self):
        while rclpy.ok():
            if self.get_distance_to_destination() > self.threshold_distance_:
                twist = Twist()
                twist.linear.x = min(self.max_linear_velocity_, 0.5*self.get_distance_to_destination())
                twist.angular.z = self.max_angular_velocity_ * self.get_yaw_to_destination()
                self.publisher_.publish(twist)
            else:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)

            self.get_logger().info('Distance to destination: {:.2f}'.format(self.get_distance_to_destination()))

            rclpy.spin_once(self)

def main():
    rclpy.init()
    node = MoveToXY()
    node.set_destination(2.0, 10.0)
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry


class Turtlebot3Controller(Node):

    def __init__(self):
        super().__init__('turtlebot3_controller')

        self.declare_parameter('speed', 0.2)
        self.declare_parameter('omega', 1.0)

        self.pose = None
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.cb_pose,
            10)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cb_pose(self, msg):
        self.pose = msg.pose.pose.position

    def go_straight(self, distance):
        speed = self.get_parameter('speed').get_parameter_value().double_value
        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Set loop rate
        loop_rate = self.create_rate(100) # Hz

        # Calculate time
        T = abs(distance/speed)     # s

        # Publish first msg and note time
        self.twist_pub.publish(vel_msg)
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() < when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            loop_rate.sleep()

        # Set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)

    def turn(self, angle):
        omega = self.get_parameter('omega').get_parameter_value().double_value
        vel_msg = Twist()

        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        if angle > 0:
            vel_msg.angular.z = math.radians(omega)
        else:
            vel_msg.angular.z = math.radians(-omega)

        # Set loop rate
        loop_rate = self.create_rate(100) # Hz

        # Calculate time
        T = abs(angle/omega)     # s

        # Publish first msg and note time
        self.twist_pub.publish(vel_msg)
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() < when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            loop_rate.sleep()

        # Set velocity to 0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)


    def draw_square(self, a):
        for i in range(4):
            self.go_straight(a)
            self.turn(90)

    def draw_poly(self, N, a):
        angle = 360 / N
        for i in range(N):
            self.go_straight(a)
            self.turn(angle)


    def go_to(self, x, y):
        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        rclpy.spin_once(self)

        x0 = self.pose.x
        y0 = self.pose.y
        theta_0 = self.pose.theta

        print("Going to (", x, ", ", y, ")")
        angle = (((math.atan2(y - y0, x - x0) - theta_0) * 180) / math.pi)
        print("angle ", angle)
        self.turn(angle)

        distance = math.sqrt(((x- x0) * (x - x0)) + ((y - y0) * (y - y0)))
        self.go_straight(distance)
        print("Arrived to (", self.pose.x, ", ", self.pose.y, ")")

def main(args=None):
    rclpy.init(args=args)
    tc = Turtlebot3Controller()
    #tc.turn(20.0,90.0)
    #tc.go_straight(1.0,4.0)

    #tc.draw_poly(1.0, 90.0, 6, 3.0)

    tc.go_to(2, 20)
    #tc.go_to(2, 2)
    #tc.go_to(3, 4)
    #tc.go_to(6, 2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


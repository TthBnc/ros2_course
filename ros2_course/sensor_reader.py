import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu

class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader')

        self.laser = None
        self.imu = None

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.imu_subscriber = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)

    def laser_callback(self, msg):
        self.laser = msg

    def imu_callback(self, msg):
        self.imu = msg

def main(args=None):
    rclpy.init(args=args)
    node = SensorReader()
    rclpy.spin(node)
    print('So far so good')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

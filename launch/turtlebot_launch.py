from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_course',
            namespace='turtlebot3',
            executable='controller',
            name='controller',
            remappings=[
                ('/turtlebot3/odom', '/odom'),
                ('/turtlebot3/cmd_vel', '/cmd_vel'),
                ('/turtlebot3/scan', '/scan'),
                ('/turtlebot3/imu', '/imu'),
            ]
        )
    ])

#from launch import LaunchDescription
#from launch_ros.actions import Node

#def generate_launch_description():
#    return LaunchDescription([
#        Node(
#            package='ros2_course',
#            namespace='turtlebot3',
#            executable='controller',
#        ),
#    ])

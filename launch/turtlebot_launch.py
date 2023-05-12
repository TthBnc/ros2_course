from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_course',
            namespace='turtlebot3',
            executable='controller',
            name='controller',
            #remappings=[
            #    ('/turtle1/pose', '/turtlesim1/turtle1/pose'),
            #    ('/turle1/cmd_vel', '/turtlesim1/turtle1/cmd_vel'),
            #]
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

#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class Twist2Wheels:
    """
    Class that represents a robot and converts Twist messages to wheel velocities.
    """

    def __init__(self):
        """
        Constructor for Twist2Wheels class.
        Initializes physical parameters of robot, and initializes publisher and subscriber for Twist and wheel velocities.
        """
        # Robot physical parameters
        self.b = 0.230  # wheelbase
        self.r = 0.035  # wheel radius

        # Initialize publisher for wheel velocities
        self.wheel_pub = rospy.Publisher('/turtlebot/kobuki/commands/wheel_velocities', Float64MultiArray, queue_size=10)
        
        # Initialize subscriber for Twist messages
        self.twist_sub = rospy.Subscriber('/turtlebot/twist', Twist, self.twist_msg_callback)


    def twist_msg_callback(self, msg):
        """
        Callback function for Twist message subscriber.
        Converts Twist message to left and right wheel velocities, and publishes the result.
        :param msg: Twist message received by subscriber
        """
        v = msg.linear.x
        w = msg.angular.z

        print(f"v {v}, w {w}")

        left_lin_vel = v - (w * self.b / 2.0)
        right_lin_vel = v + (w * self.b / 2.0)

        left_wheel_vel = left_lin_vel / self.r
        right_wheel_vel = right_lin_vel / self.r
        print(f"left_wheel_vel {left_wheel_vel}, right_wheel_vel {right_wheel_vel}")


        # Publish wheel velocities
        self.wheel_pub.publish(Float64MultiArray(data = [right_wheel_vel, left_wheel_vel]))


if __name__ == '__main__':
    """
    Main function that initializes node and Twist2Wheels object, and spins node.
    """
    rospy.init_node('Twist2RobotWheels')
    robot = Twist2Wheels()
    rospy.spin()


#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import tf
from geometry_msgs.msg import PoseStamped
import math

from tf.broadcaster import TransformBroadcaster
class kinematics:
    def __init__(self):

        self.theta = None
        self.theta2 = None
        self.theta3 = None
        self.theta4 = None
        self.T1 = np.eye(4)
        self.T2 = np.eye(4)
        self.T3 = np.eye(4)
        self.T4 = np.eye(4)
        self.T = np.eye(4)
        self.transformation_mat = np.eye(4)
        self.pub = rospy.Publisher('/swiftpro/joint_velocity_controller/command', Float64MultiArray, queue_size=10)
        self.pose_stamped_pub = rospy.Publisher('/position_EE', PoseStamped, queue_size=10)   
        # self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.JointState_callback)
        self.joints_sub = rospy.Subscriber('/turtlebot/joint_states', JointState, self.JointState_callback)
        self.odom_sub = rospy.Subscriber('/turtlebot/odom', Odometry, self.get_odom)
        # self.odom_sub = rospy.Subscriber('/turtlebot/kobuki/ground_truth', Odometry, self.get_odom)

        # self.odom_pub = rospy.Publisher('', Odometry, queue_size=10)   
        #######################################################
        self.wheel_radius = 0.035
        self.wheel_base_distance = 0.230
        self.x_robot = 0.0
        self.y_robot  = 0.0
        self.th_robot  = 0.0
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.left_wheel_received = False

        self.last_time = rospy.Time.now()
        #######################################################
        # self.current_pose = rospy.Subscriber('/odom', Odometry, self.get_odom, queue_size=10)
        self.current_pose = [0, 0, 0, 0]
        self.tf_br = TransformBroadcaster()
        
        
   
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, yaw])

    def rot(self, axis, theta):
        # Generate rotation matrix around specified axis by theta (in radians)
        if axis == 'x':
            matrix = np.array([[1, 0, 0, 0],
                            [0, round(np.cos(theta), 2), -round(np.sin(theta), 2), 0],
                            [0, round(np.sin(theta), 2), round(np.cos(theta), 2), 0],
                            [0, 0, 0, 1]])
        elif axis == 'y':
            matrix = np.array([[round(np.cos(theta), 2), 0, round(np.sin(theta), 2), 0],
                            [0, 1, 0, 0],
                            [-round(np.sin(theta), 2), 0, round(np.cos(theta), 2), 0]
                            , [0, 0, 0, 1]])
        elif axis == 'z':
            matrix = np.array([[round(np.cos(theta), 2), -round(np.sin(theta), 2), 0, 0],
                            [round(np.sin(theta), 2), round(np.cos(theta), 2), 0, 0],
                            [0, 0, 1, 0], 
                            [0, 0, 0, 1]])
        else:
            raise ValueError("Invalid axis. Must be 'x', 'y', or 'z'.")
        matrix
        return matrix
    
    def translation_matrix(self, translation):
        # Check if the translation vector has the correct size (3 elements)
        if len(translation) != 3:
            raise ValueError("Invalid translation vector. Must have three elements.")

        # Create a 4x4 identity matrix
        matrix = np.eye(4)

        # Set the last column (except the last element) to the translation vector
        matrix[:3, 3] = translation

        # Return the resulting translation matrix
        return matrix


    def JointState_callback(self, data):

        # Check if the joint names in the received message match the expected names
        
        if data.name == ['turtlebot/swiftpro/joint1', 'turtlebot/swiftpro/joint2', 'turtlebot/swiftpro/joint3', 'turtlebot/swiftpro/joint4']:
            
            # Extract joint angles from the message
            self.theta, self.theta2, self.theta3, self.theta4 = data.position

            self.x = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta)
            self.y = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.sin(self.theta)
            self.z = -0.108 -  0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3)  + 0.0722
                
                
            self.transformation_mat= np.array([
                                [1,0,0, self.x],
                                [0,1,0, self.y],
                                [0,0,1, self.z],
                                [0,0,0,  1    ]
                                            ]) 
            
            
            T_robot_manipulator = np.array([
                                [math.cos(-1.571) , -math.sin(-1.571), 0, 0.051],
                                [math.sin(-1.571), math.cos(-1.571), 0, 0],
                                [0             ,  0               , 1 , -0.198],
                                [0             ,  0               , 0, 1]
                                                                        ])
#               rosrun tf tf_echo turtlebot/kobuki/base_footprint turtlebot/swiftpro/manipulator_base_link
#           - Translation: [0.051, 0.000, -0.198]
#           - Rotation: in Quaternion [0.000, 0.000, -0.707, 0.707]
#             in RPY (radian) [0.000, 0.000, -1.571]
#             in RPY (degree) [0.000, 0.000, -90.000]

            # Mobile base position with respect to the world  
            self.T_world_robot = np.array([
                                [math.cos(self.current_pose[3]) , -math.sin(self.current_pose[3]), 0, self.current_pose[0]],
                                [math.sin(self.current_pose[3]), math.cos(self.current_pose[3]), 0, self.current_pose[1]],
                                [0             ,  0               , 1 , self.current_pose[2]],
                                [0             ,  0               , 0, 1]
                                                                        ])
            
            # Modify the world-to-robot transformation matrix by applying the rotation
            self.T = self.T_world_robot @ T_robot_manipulator @ self.transformation_mat
            
            self.marker_EE()

        
    '''
    - turtlebot/swiftpro/joint1
    - turtlebot/swiftpro/joint2
    - turtlebot/swiftpro/joint3
    - turtlebot/swiftpro/joint4
    '''
    def marker_EE(self):
        # Extract the position of the end effector from the transformation matrix
        position_EE = self.T[:, -1]
        print("position_EE", position_EE)
        # Create a Marker object to visualize the end effector position
        pose_stamped = PoseStamped()
        # Set the reference frame for the marker
        pose_stamped.header.frame_id = "world_ned"
        
        # Set the position of the pose based on the end effector position
        pose_stamped.pose.position.x = position_EE[0]
        pose_stamped.pose.position.y = position_EE[1]
        pose_stamped.pose.position.z = position_EE[2]

         # Set the orientation of the pose 
        q = quaternion_from_euler(0, 0, self.theta4)
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]

        # Publish the PoseStamped to the specified topic
        self.pose_stamped_pub.publish(pose_stamped)
        
        




if __name__ == '__main__':
    try:
        rospy.init_node('kinematics_node', anonymous=True)
        K = kinematics()
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
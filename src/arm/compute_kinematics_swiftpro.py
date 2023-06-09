#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import math
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
        self.transformation_mat = np.eye(4)
        self.pub = rospy.Publisher('/swiftpro/joint_velocity_controller/command', Float64MultiArray, queue_size=10)
        self.pose_stamped_pub = rospy.Publisher('/position_EE', PoseStamped, queue_size=10)   
        

        self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.JointState_callback)
        self.J = np.zeros((6, 4))
        # self.robot_pose = rospy.Subscriber('/turtlebot/odom', Odometry, self.get_odom, queue_size=10)

    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y,odom.pose.pose.position.z, yaw])
   
        
        
        
        
    def kinematics_publisher(self):
        rate = rospy.Rate(10)  # 10Hz, sets the desired rate for the loop

        # Continuously run the loop until a shutdown request is received
        while not rospy.is_shutdown():
            # Create a Float64MultiArray message object
            msg = Float64MultiArray()

            # Set the message data to zeros (no joint velocity)
            msg.data = [0, 0, 0, 0]

            # Publish the message to the joint_velocity_controller topic
            self.pub.publish(msg)

            # Sleep to maintain the desired rate (10Hz)
            rate.sleep()

            

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
        if data.name == ['swiftpro/joint1', 'swiftpro/joint2', 'swiftpro/joint3', 'swiftpro/joint4']:
            # Extract joint angles from the message
            self.theta, self.theta2, self.theta3, self.theta4 = data.position

            # Compute transformation matrices for each joint based on their angles
            # self.T1 = self.rot('z', self.theta) @ self.translation_matrix(np.array([0.0132, 0, 0])) @ self.rot('x', -np.pi/2) @ self.translation_matrix(np.array([0, 0.108, 0]))
            # self.T2 = self.rot('z', self.theta2) @ self.translation_matrix(np.array([0, 0.142, 0])) @ self.rot('z', -np.pi/2) @ self.rot('z', -self.theta2)
            # self.T3 = self.rot('z', self.theta3) @ self.translation_matrix(np.array([0, 0.1588, 0])) @ self.rot('z', -self.theta3) @ self.translation_matrix(np.array([0, 0.0565, 0])) @ self.rot('z', np.pi/2) @ self.rot('x', np.pi/2)
            # self.T4 = self.rot('z', self.theta4) @ self.translation_matrix(np.array([0, 0, 0.0722]))
            ##################################
            
            self.x = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta)
            self.y = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.sin(self.theta)
            self.z = -0.108 -  0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3)  + 0.0722
            
            
            self.transformation_mat= np.array([
                            [1,0,0, self.x],
                            [0,1,0, self.y],
                            [0,0,1, self.z],
                            [0,0,0,  1    ]
                                          ]) 
            
      
      

            #######################################
            #^Cppppp [ 0.2286  0.     -0.1778  1.    ]
            #ppppp [ 2.28167463e-01 -6.97524647e-07  1.77984499e-01  1.00000000e+00]

            # Compute the overall transformation matrix for the end effector
            # self.transformation_mat = self.T1 @ self.T2 @ self.T3 @ self.T4

            print("EE position", self.x, self.y, self.z)
            # print("tranformation matrix:", self.transformation_mat)

            # Call the marker_EE method to visualize the end effector position
            self.marker_EE()

    '''
    - turtlebot/swiftpro/joint1
    - turtlebot/swiftpro/joint2
    - turtlebot/swiftpro/joint3
    - turtlebot/swiftpro/joint4
    '''
    
    
    def marker_EE(self):
        # Extract the position of the end effector from the transformation matrix
        position_EE = self.transformation_mat[:, -1]

        # Create a Marker object to visualize the end effector position
        pose_stamped = PoseStamped()
        # Set the reference frame for the marker
        pose_stamped.header.frame_id = "swiftpro/manipulator_base_link"
        
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
        # K.kinematics_publisher()
        
        # print(K.rot('x', np.pi))
        
        # translation = np.array([5, 0, 0])
        # print(K.translation_matrix(translation))
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



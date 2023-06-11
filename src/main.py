#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Twist
import time
import numpy as np
import math
from std_msgs.msg import String, Float32MultiArray

from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from std_srvs.srv import SetBool, SetBoolResponse
import tf
from nav_msgs.msg import Odometry
from tasks_class import *
import sys
import matplotlib.pyplot as plt


class task_priority():
    def __init__(self, theta, theta2, theta3, theta4):
        # Initialize ROS node
        rospy.init_node('task_priority_node', anonymous=True)
        
        # Set the weight matrix
        self.we = np.diag([1, 1, 1, 1, 1, 1])

        # Define joint types
        self.revolute = [True, True, True, True]
        self.base = [False, True]
        self.base.extend(self.revolute)
        self.dof = len(self.base)
        
        # Initialize joint configuration
        self.q = np.zeros(self.dof).reshape(-1, 1)
        
        # Set initial values for theta parameters
        self.theta, self.theta2, self.theta3, self.theta4 = theta, theta2, theta3, theta4
        
        # Initialize transformation matrices and variables
        self.robot_angle = 0
        self.goal = np.zeros((1, 4))
        self.armBase_T_EE = np.zeros((4, 4))
        self.robotBase_T_armBase = np.zeros((4, 4))
        self.world_T_robotBase = np.zeros((4, 4))
        self.current_pose = [0, 0, 0, 0]
        self.goal_reached = 0
        self.aruco_recieved = True
        self.on_off = True
        
        # Initialize lists for recording data
        self.timesteps = []
        self.position_error = []
        self.q_1 = []
        self.q_2 = []
        self.q_3 = []
        self.q_4 = []
        self.q_5 = []
        self.q_6 = []
        self.position_EE_x = []
        self.position_EE_y = []

        self.position_mb_x = [] #mobile base position
        self.position_mb_y = [] #mobile base position
        
        # Define ROS subscribers and publishers
        self.aruco_sub = rospy.Subscriber("measured_data", Float32MultiArray, self.get_aruco_goal)
        self.joints_sub = rospy.Subscriber('/turtlebot/joint_states', JointState, self.JointState_callback)
        self.joint_velocity = rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)
        self.robot_velocity = rospy.Publisher("/turtlebot/twist", Twist, queue_size=10)
        self.goal_check = rospy.Publisher('/goal_check', PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber('/turtlebot/odom', Odometry, self.get_odom)
        self.marker_goal_pub = rospy.Publisher('goal_marker', Marker, queue_size=10)

        
        

        
    def get_aruco_goal(self, data):
        # Extract x and y coordinates from the received data
        x = data.data[1]
        y = data.data[2]
        
        # Check if an ArUco goal has been received before
        if self.aruco_recieved:
            # Set the goal coordinates
            self.goal = [x, y]
            
            # Mark that an ArUco goal has been received
            self.aruco_recieved = False

            
    def turn_on_pump(self, value):
        # Wait for the service to become available
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')

        try:
            # Create a service proxy for the "set_pump" service
            set_bool = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
            
            # Call the service with the specified value
            response = set_bool(value)
            
            # Return the success status from the response
            return response.success
        except rospy.ServiceException as e:
            # Print an error message if the service call failed
            print("Service call failed: %s" % e)

            
    def JointState_callback(self, data):
        if data.name == ['turtlebot/swiftpro/joint1', 'turtlebot/swiftpro/joint2', 'turtlebot/swiftpro/joint3', 'turtlebot/swiftpro/joint4']:
            # Extract joint angles from the message
            self.theta, self.theta2, self.theta3, self.theta4 = data.position 
            
            # Update the task priority algorithm with the new joint angles
            self.update(self.theta, self.theta2, self.theta3, self.theta4)
            
            # Define goals based on the current goal_reached value
            if self.goal_reached == 0:
                goals = [[0, [float(self.goal[0]), float(self.goal[1]), -0.25]]]
                self.f_goal([float(self.goal[0]), float(self.goal[1]), -0.25])
            elif self.goal_reached == 1:
                goals = [[0, [float(self.goal[0]), float(self.goal[1]), -0.13]]]
                self.f_goal([float(self.goal[0]), float(self.goal[1]), -0.13])
            elif self.goal_reached == 2:
                goals = [[0, [float(self.goal[0]), float(self.goal[1]), -0.22]]]
                self.f_goal([float(self.goal[0]), float(self.goal[1]), -0.22])
            elif self.goal_reached == 3:
                goals = [[0, [1, 1, -0.22]]]
                self.f_goal([1, 1, -0.22])
            elif self.goal_reached == 4:
                self.on_off = False
                goals = [[0, [1, 1, -0.13]]]
                self.f_goal([1, 1, -0.13])
            elif self.goal_reached == 5:
                goals = [[0, [1, 1, -0.22]]]
                self.f_goal([1, 1, -0.22])
            elif self.goal_reached == 6:
                rospy.signal_shutdown("End of program") 
            
            # Execute the task priority algorithm with the defined goals
            self.Task_priority_algorithm(goals)

    def get_odom(self, odom):
        # Extract the yaw angle from the orientation quaternion
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])
        
        # Update the current pose and robot angle
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, yaw])
        self.robot_angle = yaw


    def kinematics(self):
        # Calculate the x, y, z coordinates of the end effector
        self.x = (0.0132 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3) + 0.0565) * np.cos(self.theta)
        self.y = (0.0132 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3) + 0.0565) * np.sin(self.theta)
        self.z = -0.108 - 0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3) + 0.0722
        
        # Compute the transformation matrix from the arm base to the end effector
        self.armBase_T_EE = np.array([
            [1, 0, 0, self.x],
            [0, 1, 0, self.y],
            [0, 0, 1, self.z],
            [0, 0, 0, 1]
        ]) 
        
        # Define the transformation matrix from the robot base to the arm base
        self.robotBase_T_armBase = np.array([
            [math.cos(-math.pi/2), -math.sin(-math.pi/2), 0, 0.051],
            [math.sin(-math.pi/2), math.cos(-math.pi/2), 0, 0],
            [0, 0, 1, -0.198],
            [0, 0, 0, 1]
        ])
        
        # Compute the transformation matrix from the world frame to the robot base frame
        self.world_T_robotBase = np.array([
            [math.cos(self.current_pose[3]), -math.sin(self.current_pose[3]), 0, self.current_pose[0]],
            [math.sin(self.current_pose[3]), math.cos(self.current_pose[3]), 0, self.current_pose[1]],
            [0, 0, 1, self.current_pose[2]],
            [0, 0, 0, 1]
        ])
        
        # Compute the overall transformation matrix from the world frame to the end effector
        self.T = self.world_T_robotBase @ self.robotBase_T_armBase @ self.armBase_T_EE
        
        return self.T

    
    def Jacobian(self):
        # Compute the partial derivatives of x, y, z with respect to q1, q2, q3, q4
        dx_dq1 = -0.0565 * np.sin(self.theta) * math.sin(self.current_pose[3]) + 0.0565 * np.cos(self.theta) * (math.cos(self.current_pose[3]) - math.sin(self.current_pose[3]))
        dy_dq1 = ((0.0132 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3) + 0.0565) * np.cos(self.theta)) * (math.cos(self.current_pose[3]) + math.sin(self.current_pose[3])) \
                + ((0.0132 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3) + 0.0565) * np.cos(self.theta)) * (math.sin(self.current_pose[3]))
        dz_dq1 = 0
        
        dx_dq2 = -0.142 * np.cos(self.theta2) * np.cos(self.theta) * math.sin(self.current_pose[3]) - 0.142 * np.cos(self.theta2) * np.sin(self.theta) * (math.cos(self.current_pose[3]) - math.sin(self.current_pose[3]))
        dy_dq2 = -0.142 * np.cos(self.theta2) * np.sin(self.theta) * (math.cos(self.current_pose[3]) + math.sin(self.current_pose[3])) + 0.142 * np.cos(self.theta2) * np.cos(self.theta) * math.cos(self.current_pose[3])
        dz_dq2 = 0.142 * np.sin(self.theta2)
        
        dx_dq3 = -0.1588 * np.sin(self.theta3) * np.cos(self.theta) * math.sin(self.current_pose[3]) - 0.1588 * np.sin(self.theta3) * np.sin(self.theta) * (math.cos(self.current_pose[3]) - math.sin(self.current_pose[3]))
        dy_dq3 = -0.1588 * np.sin(self.theta3) * np.sin(self.theta) * (math.cos(self.current_pose[3]) + math.sin(self.current_pose[3])) + 0.1588 * np.sin(self.theta3) * np.cos(self.theta) * math.cos(self.current_pose[3])
        dz_dq3 = -0.1588 * np.cos(self.theta3)
        
        dx_dq4 = 0
        dy_dq4 = 0
        dz_dq4 = 0
        
        # Compute the partial derivatives of x, y, z with respect to d, theta_robot
        dx_dd = math.cos(self.current_pose[3])
        dy_dd = math.sin(self.current_pose[3])
        dz_dd = 0
        
        d = self.current_pose[0]
        dx_dtheta_r = ((0.0132 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3) + 0.0565) * np.cos(self.theta)) * math.cos(self.current_pose[3]) \
                    - 0.051 * math.sin(self.current_pose[3]) - d * math.sin(self.current_pose[3]) + ((0.0132 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3) + 0.0565) * np.sin(self.theta)) \
                    * (-math.sin(self.current_pose[3]) - math.cos(self.current_pose[3]))
        dy_dtheta_r = 0.051 * math.cos(self.current_pose[3]) + d * math.cos(self.current_pose[3]) + ((0.0132 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3) + 0.0565) * np.sin(self.theta)) \
                    * (math.cos(self.current_pose[3]) - math.sin(self.current_pose[3])) \
                    + ((0.0132 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3) + 0.0565) * np.cos(self.theta)) \
                    * (math.sin(self.current_pose[3]))
        dz_dtheta_r = 0
        
        # Create the Jacobian matrix
        self.J = np.array([
            [dx_dd, dx_dtheta_r, dx_dq1, dx_dq2, dx_dq3, dx_dq4],
            [dy_dd, dy_dtheta_r, dy_dq1, dy_dq2, dy_dq3, dy_dq4],
            [dz_dd, dz_dtheta_r, dz_dq1, dz_dq2, dz_dq3, dz_dq4],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 1]
        ])

        return self.J

    
    def update(self, theta, theta2, theta3, theta4):
        """
        Update the joint angles of the robot arm.
        
        Args:
            theta: Joint angle of q.
            theta2: Joint angle of q2.
            theta3: Joint angle of q3.
            theta4: Joint angle of q4.
        """
        self.theta = theta
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.q = [0, self.robot_angle, self.theta, self.theta2, self.theta3, self.theta4]

    def getEEJacobian(self, link):
        """
        Get the Jacobian matrix of the end effector for a specific link.
        
        Args:
            link: Index of the link (column) in the Jacobian matrix.
        
        Returns:
            The Jacobian matrix for the specified link.
        """
        return self.Jacobian()[:, link]

    def getJacobian(self):
        """
        Get the Jacobian matrix of the robot arm.
        
        Returns:
            The Jacobian matrix.
        """
        return self.J

    def getEETransform(self):
        """
        Get the transformation matrix of the end effector.
        
        Returns:
            The transformation matrix of the end effector.
        """
        return self.kinematics()[:, -1]

    def get_link_Jacobian(self, link):
        """
        Get the Jacobian matrix of a specific link.
        
        Args:
            link: Index of the link (column) in the Jacobian matrix.
        
        Returns:
            The Jacobian matrix for the specified link.
        """
        return self.Jacobian()

    def get_link_Transform(self, link):
        """
        Get the transformation matrix of a specific link.
        
        Args:
            link: Index of the link (column) in the transformation matrix.
        
        Returns:
            The transformation matrix for the specified link.
        """
        return self.kinematics()

    def getDOF(self):
        """
        Get the degrees of freedom (DOF) of the robot arm.
        
        Returns:
            The degrees of freedom of the robot arm.
        """
        return self.dof

    def getJointPos(self, joint):
        """
        Get the position of a specific joint.
        
        Args:
            joint: Index of the joint.
        
        Returns:
            The position of the specified joint.
        """
        return self.q[joint]
    
    def DLS(self, A: np.ndarray, damping: float) -> np.ndarray:
        """
        Compute the damped least squares (DLS) solution for a linear system.
        
        Args:
            A: Coefficient matrix of the linear system.
            damping: Damping factor to ensure numerical stability.
        
        Returns:
            The DLS solution of the linear system.
        """
        weights = np.diag([300, 300, 4, 4, 4, 4])
        A_TA = (A @ np.linalg.inv(weights)) @ A.T
        I = np.identity(A_TA.shape[0])
        DLS = np.linalg.inv(A_TA + damping**2 * I)
        DLS = np.linalg.inv(weights) @ A.T @ DLS
        return DLS

    
    def f_goal(self, g):
        """
        Publish a goal marker visualization in the ROS environment.
        
        Args:
            g: List representing the goal position [x, y, z].
        """
        print("goal marker")
        marker = Marker()
        marker.header.frame_id = "world_ned"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = g[0]
        marker.pose.position.y = g[1]
        marker.pose.position.z = g[2]
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.g = 0.5
        marker.color.r = 0.5
        marker.color.a = 0.4
        self.marker_goal_pub.publish(marker)


    def send_commnd(self, q):
        """
        Send a command to control the robot's joint and base velocities.
        
        Args:
            q: List representing the desired velocities [linear_x, angular_z, q1, q2, q3, q4].
        """
        q = q / 2  # Divide velocities by 2
        
        p = Float64MultiArray()
        p.data = [float(q[2]), float(q[3]), float(q[4]), float(q[5])]  # Set joint velocities
        
        velocities = Twist()
        velocities.linear.x = q[0]
        velocities.linear.y = 0
        velocities.linear.z = 0
        velocities.angular.x = 0
        velocities.angular.y = 0
        velocities.angular.z = q[1]  # Set base velocities
        
        self.joint_velocity.publish(p)  # Publish joint velocities
        self.robot_velocity.publish(velocities)  # Publish base velocities


    def tasks(self, goals):
        """
        Create task objects based on the specified goals.
        
        Args:
            goals: List of goals, where each goal is a tuple (goal_type, goal_value).
                goal_type: int representing the type of goal.
                goal_value: List or numpy array representing the goal value.

        Returns:
            tasks: List of task objects based on the specified goals.
        """
        tasks = []
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world_ned"
        
        for goal in goals:
            if goal[0] == 0:
                tasks.append(Position3D("End-effector position", np.array(goal[1]).reshape(3, 1), link=3))
                pose_stamped.pose.position.x = goal[1][0]
                pose_stamped.pose.position.y = goal[1][1]
                pose_stamped.pose.position.z = goal[1][2]
            elif goal[0] == 1:
                tasks.append(Orientation3D("end-effector orientation", np.array(goal[1]), link=3))
                q = quaternion_from_euler(0, 0, goal[1][0])
                pose_stamped.pose.orientation.x = q[0]
                pose_stamped.pose.orientation.y = q[1]
                pose_stamped.pose.orientation.z = q[2]
                pose_stamped.pose.orientation.w = q[3]
            elif goal[0] == 2:
                tasks.append(Configuration3D("end-effector configuration", np.array(goal[1]).reshape(4, 1), link=3))
                pose_stamped.pose.position.x = goal[1][0]
                pose_stamped.pose.position.y = goal[1][1]
                pose_stamped.pose.position.z = goal[1][2]
                q = quaternion_from_euler(0, 0, goal[1][3])
                pose_stamped.pose.orientation.x = q[0]
                pose_stamped.pose.orientation.y = q[1]
                pose_stamped.pose.orientation.z = q[2]
                pose_stamped.pose.orientation.w = q[3]
            elif goal[0] == 3:
                tasks.append(JointPosition("joint 1 position", np.array(goal[1][0]), link=goal[1][1]))
            elif goal[0] == 4:
                tasks.append(JointLimitTask("Joint 1 limit", np.array([-0.5, 2.5]), np.array([-0.5, 0.5]), 1))
        
        self.goal_check.publish(pose_stamped)
        return tasks


    def Task_priority_algorithm(self, goal):
        """
        Perform the task priority algorithm to control the robot based on the specified goal.

        Args:
            goal: Goal to be achieved.

        Returns:
            None
        """
        tasks = self.tasks(goal)  # Create task objects based on the specified goal
        P = np.eye(6)  # Initialize the pseudo-inverse matrix P
        dq = np.zeros((6, 1))  # Initialize the joint velocity vector dq
        error = np.zeros((1, len(tasks)))  # Initialize the error array
        
        for i in range(len(tasks)):
            tasks[i].update(self)  # Update the task
            aug_Jacobian = tasks[i].getJacobian() @ P  # Calculate the augmented Jacobian matrix
            dq = dq + self.DLS(aug_Jacobian, 0.1) @ (tasks[i].getError() - (tasks[i].getJacobian() @ dq))  # Update joint velocities using the Damped Least Squares (DLS) method
            P = P - np.linalg.pinv(aug_Jacobian) @ aug_Jacobian  # Update the pseudo-inverse matrix P
            
            # Store values for tracking and visualization
            self.position_error.append(np.linalg.norm(tasks[0].getError()))
            self.q_1.append(dq[0])
            self.q_2.append(dq[1])
            self.q_3.append(dq[2])
            self.q_4.append(dq[3])
            self.q_5.append(dq[4])
            self.q_6.append(dq[5])
            self.position_mb_x.append(self.current_pose[0])
            self.position_mb_y.append(self.current_pose[1])
            self.position_EE_x.append(self.getEETransform()[0])
            self.position_EE_y.append(self.getEETransform()[1])

            if self.timesteps == []:
                self.timesteps.append(0)
            else:
                self.timesteps.append(self.timesteps[-1] + 1.0 / 60.0)
            
            print("error", tasks[i].getError())
            
            # Check if the goal has been reached
            if np.abs(tasks[i].getError()[0]) < 0.03 and np.abs(tasks[i].getError()[1]) < 0.03 and np.abs(tasks[i].getError()[2]) < 0.03:
                error[i] = 1
                self.goal_reached += 1
                time.sleep(0.5)
            
            # Take appropriate actions if all goals have been reached
            if np.all(error) == 1:
                dq = np.zeros((6, 1))
                self.send_commnd(dq)
                
                if self.on_off:
                    print("picking up")
                    success = self.turn_on_pump(True)
                    if success:
                        print("Pump set to True")
                    else:
                        print("Failed to set pump to True")
                else:
                    print("dropping off")
                    success = self.turn_on_pump(False)
                    if success:
                        print("Pump set to False")
                    else:
                        print("Failed to set pump to false")
            else:
                self.send_commnd(dq)  # Send joint velocities to the robot



if __name__ == '__main__':
    try:
        rospy.init_node('Task_priority_node', anonymous=True)
        a = task_priority(0, 0, 0, 0)
        rospy.spin()
        print("END")
        
        plt.figure()
        plt.plot(a.timesteps, a.position_error, label='e1 (end-effector position)')
        plt.xlabel('Time step(s)')
        plt.ylabel('Error(m)')
        plt.title('Task-priority (Two tasks)')
        plt.grid()
        plt.legend()
        plt.show()
        
        plt.figure()
        plt.plot(a.position_EE_x, a.position_EE_y, label='end-effector position')
        plt.plot(a.position_mb_x, a.position_mb_y, label='mobile base position')
        plt.xlabel('Time step(s)')
        plt.ylabel('position(m)')
        plt.title('Task-priority control')
        plt.grid()
        plt.legend()
        plt.show()
        
        plt.figure()
        plt.plot(a.timesteps, a.q_1, label='q1')
        plt.plot(a.timesteps, a.q_2, label='q2')
        plt.plot(a.timesteps, a.q_3, label='q3')
        plt.plot(a.timesteps, a.q_4, label='q4')
        plt.plot(a.timesteps, a.q_5, label='q5')
        plt.plot(a.timesteps, a.q_6, label='q6')
        plt.xlabel('Time step(s)')
        plt.ylabel('Velocity(m/s)')
        plt.title('Task-priority control')
        plt.grid()
        plt.legend()
        plt.show()
        
        sys.exit(0)
    except rospy.ROSInterruptException:
        pass

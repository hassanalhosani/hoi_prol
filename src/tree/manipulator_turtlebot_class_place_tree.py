#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped , Twist 
import time
import numpy as np # Import Numpy
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

class manipulator():
    def __init__(self, theta, theta2, theta3, theta4):
        self.initialize_parameters(theta, theta2, theta3, theta4)
        self.initialize_subscribers_and_publishers()
        time.sleep(1)
        self.we = np.diag([1,1,1, 1, 1, 1])

    def initialize_parameters(self, theta, theta2, theta3, theta4):
        self.revolute = [True, True, True, True]
        self.base = [False, True]
        self.base.extend(self.revolute) 
        self.dof = len(self.base)
        self.q = np.zeros(self.dof).reshape(-1, 1)
        self.theta, self.theta2, self.theta3, self.theta4 = theta, theta2, theta3, theta4
        self.robot_angle = 0
        self.goal = np.zeros((1,4))
        self.transformation_mat = np.zeros((4,4))
        self.T_robot_manipulator = np.zeros((4,4))
        self.T_world_robot = np.zeros((4,4))
        self.current_pose = [0, 0, 0, 0]
        self.goal_reached = 0
        self.aruco_recieved = True
        self.on_off = True
        self.z_recieved = True
        self.ratio = 1
        self.timesteps = []
        self.position_error = []
        

    def initialize_subscribers_and_publishers(self):
        self.pose_stamped_pub = rospy.Subscriber('/position_EE', PoseStamped, self.get_ee)   
        self.set_goal([2, 0])

        # self.receive_Goal = rospy.Subscriber("goal_sent", Float32MultiArray, self.set_goal)
        self.joints_sub = rospy.Subscriber('/turtlebot/joint_states', JointState, self.JointState_callback)
        self.joint_velocity = rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)
        self.robot_velocity = rospy.Publisher("/turtlebot/twist", Twist, queue_size=10)
        self.goal_check = rospy.Publisher('/goal_check', PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber('/turtlebot/odom', Odometry, self.get_odom)
        # self.odom_sub = rospy.Subscriber('/turtlebot/kobuki/ground_truth', Odometry, self.get_odom)
        self.marker_goal_pub = rospy.Publisher('goal_marker', Marker, queue_size=10)   

    def turn_on_pump(self, value):
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')  
        try:
            set_bool = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
            response = set_bool(value)
            return response.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_ee(self, data):
        
        if self.z_recieved:
            self.EE_z = data.pose.position.z
            self.z_recieved = False  
            
    def set_goal(self, data):
        # self.goal = [data.data[0],  data.data[1]]
        self.goal = [1, 1]
 
    def JointState_callback(self,data):
        if data.name == ['turtlebot/swiftpro/joint1', 'turtlebot/swiftpro/joint2', 'turtlebot/swiftpro/joint3', 'turtlebot/swiftpro/joint4']:
            # Extract joint angles from the message
            self.theta, self.theta2, self.theta3, self.theta4 = data.position 
            self.update(self.theta, self.theta2, self.theta3, self.theta4)
            #![task_name or task number, tasks'goal]
            
            if self.goal_reached == 0:
                self.ratio = 5

                self.we = np.diag([10,10,100, 100, 100, 100])
                goals = [[0, [float(self.goal[0]), float(self.goal[1]), self.EE_z+0.04]]]
                self.f_goal([float(self.goal[0]), float(self.goal[1]), self.EE_z+0.04]) 
            elif self.goal_reached == 1:
                self.ratio = 1
                self.we = np.diag([300, 300, 4, 4, 4, 4])
                self.on_off = False
                goals = [[0, [float(self.goal[0]), float(self.goal[1]), -0.125]]]
                # goals = [[2, [float(self.goal[0]), float(self.goal[1]), -0.15, math.radians(90)]]]
                self.f_goal([float(self.goal[0]), float(self.goal[1]), -0.125])          
            elif self.goal_reached == 2:
                # self.aruco_recieved = True
                goals = [[0, [float(self.goal[0]), float(self.goal[1]), -0.22]]]
                # goals = [[2, [float(self.goal[0]), float(self.goal[1]), -0.15, math.radians(90)]]]
                self.f_goal([float(self.goal[0]), float(self.goal[1]), -0.22])
            elif self.goal_reached == 3:
                rospy.signal_shutdown("box is placed") 

            self.Task_priority_algorithm(goals)

    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, yaw])
        self.robot_angle = yaw
             
    def kinematics(self): 
        # Geometric end-effector position with respect to the manipulator base 
        self.x = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta)
        self.y = (0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.sin(self.theta)
        self.z = -0.108 -  0.142 * np.cos(self.theta2) - 0.1588 * np.sin(self.theta3)  + 0.0722
            
        self.transformation_mat= np.array([
                            [1,0,0, self.x],
                            [0,1,0, self.y],
                            [0,0,1, self.z],
                            [0,0,0,  1    ]
                                          ]) 
        
        self.T_robot_manipulator = np.array([
                            [math.cos(-math.pi/2) , -math.sin(-math.pi/2), 0, 0.051],
                            [math.sin(-math.pi/2), math.cos(-math.pi/2), 0, 0],
                            [0             ,  0               , 1 , -0.198],
                            [0             ,  0               , 0, 1]
                                                                    ])

        # Mobile base position with respect to the world  
        self.T_world_robot = np.array([
                            [math.cos(self.current_pose[3]) , -math.sin(self.current_pose[3]), 0, self.current_pose[0]],
                            [math.sin(self.current_pose[3]), math.cos(self.current_pose[3]), 0, self.current_pose[1]],
                            [0             ,  0               , 1 , self.current_pose[2]],
                            [0             ,  0               , 0, 1]
                                                                    ])
        
        # Modify the world-to-robot transformation matrix by applying the rotation
        self.T = self.T_world_robot @ self.T_robot_manipulator @ self.transformation_mat

        return self.T
    
    def Jacobian(self):
        #! check jacobians again x, y, z DONE
        # Arm Jacobian 
        # partial derivative of x, y, z with respect to q1
        dx_dq1 = -0.0565*np.sin(self.theta)*math.sin(self.current_pose[3]) + 0.0565*np.cos(self.theta)*(math.cos(self.current_pose[3])-math.sin(self.current_pose[3]))
        dy_dq1 = ((0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta))*(math.cos(self.current_pose[3])+math.sin(self.current_pose[3]))\
            +((0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta))*(math.sin(self.current_pose[3]))
        dz_dq1 = 0
        # partial derivative of x, y, z with respect to q2
        dx_dq2 = -0.142*np.cos(self.theta2)*np.cos(self.theta)*math.sin(self.current_pose[3]) - 0.142*np.cos(self.theta2)*np.sin(self.theta)*(math.cos(self.current_pose[3])-math.sin(self.current_pose[3]))
        dy_dq2 = -0.142*np.cos(self.theta2)*np.sin(self.theta)*(math.cos(self.current_pose[3])+math.sin(self.current_pose[3]))+0.142*np.cos(self.theta2)*np.cos(self.theta)*math.cos(self.current_pose[3])
        dz_dq2 = 0.142 * np.sin(self.theta2)
        # partial derivative of x, y, z with respect to q3
        dx_dq3 = -0.1588*np.sin(self.theta3)*np.cos(self.theta)*math.sin(self.current_pose[3]) - 0.1588*np.sin(self.theta3)*np.sin(self.theta)*(math.cos(self.current_pose[3])-math.sin(self.current_pose[3]))
        dy_dq3 = -0.1588 * np.sin(self.theta3) * np.sin(self.theta)*(math.cos(self.current_pose[3])+math.sin(self.current_pose[3])) + 0.1588*np.sin(self.theta3) * np.cos(self.theta)*math.cos(self.current_pose[3])
        dz_dq3 = -0.1588 * np.cos(self.theta3) 
        # partial derivative of x, y, z with respect to q4
        dx_dq4 = 0
        dy_dq4 = 0
        dz_dq4 = 0
        
        #Linear velocity for the base 
        dx_dd = math.cos(self.current_pose[3])
        dy_dd = math.sin(self.current_pose[3])
        dz_dd = 0
        
        d = self.current_pose[0]
        # Angular velocity 
        dx_dtheta_r = ((0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta))*math.cos(self.current_pose[3])\
                        -0.051*math.sin(self.current_pose[3]) - d*math.sin(self.current_pose[3]) + ((0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.sin(self.theta))\
                            *(-math.sin(self.current_pose[3])-math.cos(self.current_pose[3]))
        dy_dtheta_r = 0.051*math.cos(self.current_pose[3]) + d*math.cos(self.current_pose[3])+((0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.sin(self.theta))\
                            *(math.cos(self.current_pose[3])-math.sin(self.current_pose[3]))\
                                +((0.0132 - 0.142 * np.sin(self.theta2)  + 0.1588 * np.cos(self.theta3)  + 0.0565) * np.cos(self.theta))\
                            *(math.sin(self.current_pose[3]))
                                
        dz_dtheta_r = 0

        # Mobile_manipulator Jacobian 
        self.J = np.array([ 
                            [dx_dd, dx_dtheta_r ,dx_dq1 , dx_dq2  ,dx_dq3  ,dx_dq4],
                            [dy_dd, dy_dtheta_r ,dy_dq1 , dy_dq2  ,dy_dq3  ,dy_dq4],
                            [dz_dd,dz_dtheta_r  ,dz_dq1 , dz_dq2  ,dz_dq3  ,dz_dq4],
                            [ 0,     0    ,0    , 0     ,   0   , 0 ],
                            [ 0,     0    ,0    , 0     ,   0   , 0 ],
                            [ 0,     1    ,1    , 0     ,   0   , 1 ]
                                                                           ])
    
         
        # self.kinematics()
        return self.J
    
    def update(self, theta, theta2, theta3, theta4):
        self.theta = theta 
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.q = [0, self.robot_angle, self.theta, self.theta2, self.theta3, self.theta4]
    '''
        Method that returns the end-effector Jacobian.
    '''
    def getEEJacobian(self, link):
        return self.Jacobian()[:, link]
    
    def getJacobian(self):
        return self.J
    '''
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        return self.transformation_mat
    
    def get_link_Jacobian(self, link):
        return self.Jacobian()#[:, link]

    def get_link_Transform(self,link):
            return self.kinematics()
    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof
    
    def getJointPos(self, joint):
        return self.q[joint]
                                     
    def DLS(self, A: np.ndarray, damping: float) -> np.ndarray:
        '''
            Function computes the damped least-squares (DLS) solution to the matrix inverse problem.

            Arguments:
            A (Numpy array): matrix to be inverted
            damping (double): damping factor

            Returns:
            (Numpy array): inversion of the input matrix
        '''
        # weights = np.diag([300, 300, 4, 4, 4, 4])
        weights = self.we
        # weights = np.diag([4, 4, 300, 300, 300, 300])

        A_TA = (A @ np.linalg.inv(weights)) @ A.T
        I = np.identity(A_TA.shape[0])
        DLS = np.linalg.inv(A_TA + damping**2 * I)
        DLS = np.linalg.inv(weights) @ A.T @ DLS
        # np.matmul(DLS, (np.linalg.inv(weights) @ A.T))
        # A_TA = np.matmul(A.T, A)
        # I = np.identity(A_TA.shape[0])
        # DLS = np.linalg.inv(A_TA + damping**2 * I)
        # DLS = np.matmul(DLS, A.T)
        # Implement the formula to compute the DLS of matrix A.
        return  DLS
    def f_goal(self,g):
        print("goal marker")
        marker = Marker()
        marker.header.frame_id = "world_ned"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = g[0]
        marker.pose.position.y = g[1]
        marker.pose.position.z = g[2]

        # marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.g = 0.5
        marker.color.r = 0.5
        marker.color.a = 0.4

        self.marker_goal_pub.publish(marker)
        
    def send_commnd(self, q):
        # Manipulator base publisher 
        # q = q/2
        p = Float64MultiArray()

        p.data = [float(q[2]), float(q[3]), float(q[4]), float(q[5])]
        # print("to send", p.data)
        # q[0] = np.clip(q[0], -0.1, 0.1)
        # q[1] = np.clip(q[1], -0.1, 0.1)
        print("q", q)

       
        velocities = Twist()
        velocities.linear.x = q[0]
        velocities.linear.y = 0
        velocities.linear.z = 0
        
        velocities.angular.x = 0
        velocities.angular.y = 0
        velocities.angular.z = q[1]

        self.joint_velocity.publish(p)
        self.robot_velocity.publish(velocities)


    # Task definition                                          
    def tasks(self, goals):
        tasks = []
        pose_stamped = PoseStamped()
        # Set the reference frame for the marker

        pose_stamped.header.frame_id = "world_ned"


        for goal in goals:
            if goal[0] == 0:
                tasks.append(Position3D("End-effector position", np.array(goal[1]).reshape(3,1), link=3))               
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
                tasks.append(Configuration3D("end-effector configuration", np.array(goal[1]).reshape(4, 1),  link=3))
                pose_stamped.pose.position.x = goal[1][0]
                pose_stamped.pose.position.y = goal[1][1]
                pose_stamped.pose.position.z = goal[1][2]
                q = quaternion_from_euler(0, 0, goal[1][3])
                pose_stamped.pose.orientation.x = q[0]
                pose_stamped.pose.orientation.y = q[1]
                pose_stamped.pose.orientation.z = q[2]
                pose_stamped.pose.orientation.w = q[3]
            elif goal[0] == 3:
                tasks.append(JointPosition("joint 1 position", np.array(goal[1][0]), link = goal[1][1]))
            elif goal[0] == 4:
                tasks.append(JointLimitTask("Joint 1 limit", np.array([-0.5, 2.5]), np.array([-0.5, 0.5]), 1))
        self.goal_check.publish(pose_stamped)
        return tasks
    
    def Task_priority_algorithm(self, goal):
     
        tasks = self.tasks(goal)
        
        dof = self.getDOF()
        # # Initialize null-space projector
        P = np.eye(6)
        # # Initialize output vector (joint velocity)
        dq = np.zeros((6, 1))
        error = np.zeros((1,len(tasks)))
        
        # P = np.eye(dof)
        # # Initialize output vector (joint velocity)
        # dq = np.zeros((dof,1)) 
        for i in range(len(tasks)):
 
            tasks[i].update(self)
            # Compute augmented Jacobian
            aug_Jacobian = tasks[i].getJacobian() @ P

            # Compute task velocity
            dq = dq +\
            self.DLS(aug_Jacobian,
                0.1) @ (tasks[i].getError() - (tasks[i].getJacobian() @ dq))
            
            # Accumulate velocity
            # Update null-space projector
            P = P - np.linalg.pinv(aug_Jacobian) @ aug_Jacobian
            
            self.position_error.append(np.linalg.norm(tasks[0].getError()))
            if self.timesteps == []:
                self.timesteps.append(0)
            else:
                self.timesteps.append(self.timesteps[-1]+1.0/60.0)
            print("error", tasks[i].getError())

            if np.abs(tasks[i].getError()[0]) < 0.03 and np.abs(tasks[i].getError()[1]) < 0.03 and np.abs(tasks[i].getError()[2]) <0.03:
                error[i] = 1
                self.goal_reached += 1
                

                time.sleep(0.5)
            if np.all(error) == 1:
                
                # self.goal = np.random.rand(2, 1)*3 - 1
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
                self.send_commnd(dq)
                
                
                      
if __name__ == '__main__':
    try:
        rospy.init_node('Task_priority_node', anonymous=True)
        a = manipulator(0, 0, 0, 0)
        rospy.spin()
        
        

    except rospy.ROSInterruptException:
        pass

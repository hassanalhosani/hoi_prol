#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped , Twist 
import time
import numpy as np # Import Numpy
import math 
from tf.transformations import quaternion_from_euler
import tf
from nav_msgs.msg import Odometry
class manipulator():
    '''
        Constructor.

        Arguments:
        Robot position : (x,y,z,yaw) 
        Manipulator joint angle : ( q1,q2,q3,q4)
    '''
    def __init__(self, theta, theta2, theta3, theta4):
        self.revolute = [True, True , True , True]
        self.dof = len(self.revolute)
        self.q = np.zeros(self.dof).reshape(-1, 1)

        self.theta = theta
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.joints_sub = rospy.Subscriber('/swiftpro/joint_states', JointState, self.JointState_callback)
        self.T_end_effector= np.zeros((4,4))
        # Send position and velocity to the manipulator 
        self.joint_velocity= rospy.Publisher("/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=10)
                            
        self.theta = theta
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        # send goal to the manipulator 
        self.goal_check = rospy.Publisher('/goal_check', PoseStamped, queue_size=10)   
        time.sleep(1)  
        # Activate Task_priority_algorithm
        # rospy.Timer(rospy.Duration(0.1), self.Task_priority_algorithm)
        # self.pose_stamped_sub = rospy.Subscriber('/position_EE', Odometry, self.get_ee_position)

       # Manipulator parameters 
        self.revolute = [True, True , True , True]
        self.dof = len(self.revolute)
        # Desired position 
        self.ee_position = [0, 0, 0, 0]
        self.goals = [[0, [0.2, 0.2, -0.15]], 
            [0, [0.4, 0, 0.1]],
            [0, [0, 0.25 , 0]],
            [0, [0.3, 0.07, 0]],
            [0, [0.8, -0.5, 0]],
            [0, [0.4, 0, 0.1]]]
        
        
        
        
        
    def get_ee_position(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.ee_position = [odom.pose.pose.position.x, odom.pose.pose.position.y,odom.pose.pose.position.z]
   
        
        
    # def call_task(self, goal):
    #     self.Task_priority_algorithm(goal)
        
    def JointState_callback(self,data):
        if data.name == ['swiftpro/joint1', 'swiftpro/joint2', 'swiftpro/joint3', 'swiftpro/joint4']:
            # Extract joint angles from the message
            self.theta, self.theta2, self.theta3, self.theta4 = data.position 
            self.update(self.theta, self.theta2, self.theta3, self.theta4)
            #![task_name or task number, tasks'goal]
            # goals = [[0, [0.2, 0.2, -0.15]]] #![position, angle] Position 3d check 

            goals = [[0, [0.2, 0.2, -0.15]], [1, [math.radians(90)]]] #![position, angle] Position 3d and orientation 3d check 
            # goals = [[2, [0.2, 0.2, -0.15, math.radians(90)]]] #![position, angle] Configuration check
            # goals = [[3, [math.radians(0), 0]], [0, [0.2, 0.2, -0.15]]] #![angle, link] joint postion check
            # goals = [[4, [[-0.1, 0.1], [-math.radians(90), math.radians(90)], 0]], [0, [0.2, 0.2, -0.15]]] #![threshold, Q, link] joint limit check
            
            self.Task_priority_algorithm(goals)

            
                    


            
            


            
            
            
            


            # self.Task_priority_algorithm([math.radians(90)])


            
            
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
        return self.transformation_mat, self.x, self.y, self.z
    
    def Jacobian(self):
        #! check jacobians again x, y, z DONE
        # Arm Jacobian 
        # partial derivative of x, y, z with respect to q1
        dx_dq1 = -np.sin(self.theta) * (0.0697 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3))
        dy_dq1 = np.cos(self.theta) * (0.0697 - 0.142 * np.sin(self.theta2) + 0.1588 * np.cos(self.theta3))
        dz_dq1 = 0
        # partial derivative of x, y, z with respect to q2
        dx_dq2 = -0.142 * np.cos(self.theta) * np.cos(self.theta2)
        dy_dq2 = -0.142 * np.sin(self.theta) * np.cos(self.theta2)
        dz_dq2 = 0.142 * np.sin(self.theta2)
        # partial derivative of x, y, z with respect to q3
        dx_dq3 = -0.1588 * np.sin(self.theta3) * np.cos(self.theta)
        dy_dq3 = -0.1588 * np.sin(self.theta3) * np.sin(self.theta)
        dz_dq3 = -0.1588 * np.cos(self.theta3) 
        # partial derivative of x, y, z with respect to q4
        dx_dq4 = 0
        dy_dq4 = 0
        dz_dq4 = 0
        self.J = np.array([ 
                            [dx_dq1 , dx_dq2  ,dx_dq3  ,dx_dq4],
                            [dy_dq1 , dy_dq2  ,dy_dq3  ,dy_dq4],
                            [dz_dq1 , dz_dq2  ,dz_dq3  ,dz_dq4],
                            [ 0    , 0     ,   0   , 0 ],
                            [ 0    , 0     ,   0   , 0 ],
                            [1    , 0     ,   0   , 1 ]
                                                                           ])
        return self.J
    
    def update(self, theta, theta2, theta3, theta4):
        self.theta = theta
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.q = [self.theta, self.theta2, self.theta3, self.theta4]
        

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

    def getLinkTransform(self,link):
            return self.kinematics()[0]
    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof
    
    def getJointPos(self, joint):
        return self.q[joint]

   
   

       
        

                                                                       
    def DLS(self, A, damping):
        '''
            Function computes the damped least-squares (DLS) solution to the matrix inverse problem.

            Arguments:
            A (Numpy array): matrix to be inverted
            damping (double): damping factor

            Returns:
            (Numpy array): inversion of the input matrix
        '''
        A_TA = np.matmul(A.T, A)
        I = np.identity(A_TA.shape[0])
        DLS = np.linalg.inv(A_TA + damping**2 * I)
        DLS = np.matmul(DLS, A.T)
        # Implement the formula to compute the DLS of matrix A.
        return  DLS
    
    def send_commnd(self, q):
        # Manipulator base publisher 
        p = Float64MultiArray()

        
        p.data = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        print("to send", p.data)
        self.joint_velocity.publish(p)

    # Task definition                                          
    def tasks(self, goals):
        tasks = []
        pose_stamped = PoseStamped()
        # Set the reference frame for the marker

        pose_stamped.header.frame_id = "swiftpro/manipulator_base_link"


        for goal in goals:
            if goal[0] == 0:
                tasks.append(Position3D("End-effector position", np.array(goal[1]).reshape(3,1), link=3))                
                # Set the position of the pose based on the end effector position
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
                tasks.append(JointLimitTask("Joint 1 limit", np.array(goal[1][0]), np.array(goal[1][1]), link = goal[1][2]))
                
           

        self.goal_check.publish(pose_stamped)

        # tasks = [ 
        #     Position3D("End-effector position", np.array([goal[0], goal[1], goal[2]]).reshape(3,1), link=3),
        #     Orientation3D("end-effector orientation", np.array([goal]), link=3)
        #     ] 
        return tasks
    
    def Task_priority_algorithm(self, goal):
     
        tasks = self.tasks(goal)
        # self.time = rospy.Time.now().to_sec()
        # robot = manipulator(self.theta, self.theta2, self.theta3, self.theta4)        
        
        # tasks[0].update(robot)
        # self.error = tasks[0].getError()
        
        dof = self.getDOF()
        # # Initialize null-space projector
        P = np.eye(dof)
        # # Initialize output vector (joint velocity)
        dq = np.zeros((dof, 1))
        
        
        
        P = np.eye(self.getDOF())
        # Initialize output vector (joint velocity)
        dq = np.zeros((self.getDOF(),1)) 
        # for i in range(len(tasks)):

        #     tasks[i].update(self)
        #     if tasks[i].Joint_limit_activation() != 0:
        #         # Compute augmented Jacobian
        #         aug_Jacobian = tasks[i].getJacobian() @ P
        #         dq = dq + \
        #             self.DLS(aug_Jacobian,
        #                 0.5) @ (tasks[i].Joint_limit_activation() * tasks[i].getError() - (tasks[i].getJacobian() @ dq))
        #         P = P - np.linalg.pinv(aug_Jacobian) @ aug_Jacobian
        #     else:
        #         dq = dq
        #         P = P
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
            # print("dq", dq)
            
        # self.send_commnd(dq)
            
            
        self.send_commnd(dq)
                    
        # self.error = tasks[0].getError()
                  
           
        
class Task:
    '''
        Constructor.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)
    '''
    def __init__(self, name, desired, activation=True):
        
        self.name = name # task title
        self.sigma_d = desired # desired sigma
        self.activation_function = True
        self.limit_activation = -1
                   
    '''
        Method updating the task variables (abstract).

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def update(self, robot):
        pass

    '''
        Method updating activity of the task 

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def isActive(self):
        return True 

    ''' 
        Method setting the desired sigma.

        Arguments:
        value(Numpy array): value of the desired sigma (goal)
    '''
    def setDesired(self, value):
        self.sigma_d = value

    '''
        Method returning the desired sigma.
    '''
    def getDesired(self):
        return self.sigma_d

    '''
        Method returning the task Jacobian.
    '''
    def getJacobian(self):
        return self.J

    '''
        Method returning the task error (tilde sigma).
    '''    
    def getError(self):
        return self.err
    
    def is_active(self):
        return self.activation_function
   
    def Joint_limit_activation(self):
        return self.limit_activation
    

  

    
'''
    Subclass of Task, representing the 3D position task.
'''
class Position3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired, activation=True)
        self.err = np.zeros((3,1))# Initialize with proper dimensions 3 x 1
        self.link = link 
        self.activation_function = True
        self.limit_activation = 0
               
    def update(self, robot):

        self.J = robot.get_link_Jacobian(self.link)[0:3]

        self.err = (self.getDesired() - robot.getLinkTransform(self.link)
                    [0:3, 3].reshape(3, 1)).reshape(3, 1)
        
        
'''
    Subclass of Task, representing the 3D orientation task.
'''
class Orientation3D(Task):
    def __init__(self, name, desired,link):
        super().__init__(name, desired, activation=True)
        self.err = np.zeros((1)) # Initialize with proper dimensions
        self.link = link 
        self.activation_function = True
        self.limit_activation = 1
        
        
    def update(self, robot):
        self.J = robot.get_link_Jacobian(self.link)[-1].reshape(1, 4)
        # print(self.J)
        
        angle = np.arctan2(robot.getLinkTransform(self.link)[1,0], robot.getLinkTransform(self.link)[0,0])
        self.err = self.sigma_d - np.array([[angle]]) # Update task error
'''
    Subclass of Task, representing the 3D configuration task.
'''
class Configuration3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired, activation=True)
        self.link = link 
        self.activation_function = True


        #self.J = # Initialize with proper dimensions
        #self.err = # Initialize with proper dimensions
        
    def update(self, robot):
        self.J      = np.concatenate([robot.get_link_Jacobian(self.link)[0:3], robot.get_link_Jacobian(self.link)[-1].reshape(1, robot.getDOF())])
        sigma       = np.zeros((4,1))
        
        sigma[0:3] = robot.getLinkTransform(self.link)[0:3, 3].reshape(3, 1)

        sigma[3]    = np.arctan2(robot.getLinkTransform(self.link)[1,0], robot.getLinkTransform(self.link)[0,0])
        self.err    = (self.sigma_d - sigma).reshape(4,1)
        
'''
    Subclass of Task, representing the joint position task.
'''


class JointPosition(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired, activation=True)
        # self.J = # Initialize with proper dimensions  
        # self.err = # Initialize with proper dimensions
        self.link = link
       
    def update(self, robot):
        self.J = np.zeros((1, robot.getDOF()))
        self.J[0, self.link] = 1
        self.err = self.getDesired() - robot.getJointPos(self.link)        
        

class JointLimitTask(Task):
    def __init__(self, name, threshold, Q, link):
        super().__init__(name, threshold)
        self.link = link
        self.alpha = threshold[0]
        self.delta = threshold[1]
        self.qi_min = Q[0]
        self.qi_max = Q[1]
        self.limit_activation = -1

    def update(self, robot):
        self.J = robot.get_link_Jacobian(self.link)[5, :].reshape(1, robot.getDOF())
        self.err = 1
        q = math.atan2(robot.getLinkTransform(self.link)[1, 0], robot.getLinkTransform(self.link)[0, 0])
        self.joint_position = robot.getJointPos(self.link)

        if (self.limit_activation == 0) and (q >= (self.qi_max - self.alpha)):
            print("a")
            self.limit_activation = -1
        elif (self.limit_activation == 0) and (q <= (self.qi_min + self.alpha)):
            print("b")
            self.limit_activation = 1
        elif (self.limit_activation == -1) and (q <= (self.qi_max - self.delta)):
            print("c")
            self.limit_activation = 0
        elif (self.limit_activation == 1) and (q >= (self.qi_min + self.delta)):
            print("d")
            self.limit_activation = 0

        
if __name__ == '__main__':
    try:
        rospy.init_node('Task_priority_node', anonymous=True)
        a = manipulator(0, 0, 0, 0)
        
      
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
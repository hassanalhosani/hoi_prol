import numpy as np
import math
class Task:
    """
    Base class for task definition.

    Arguments:
    name (string): title of the task
    desired (Numpy array): desired task outcome (goal)
    """

    def __init__(self, name, desired):
        self.name = name # Name of the task
        self.sigma_d = desired # Desired outcome
        self.limit_activation = -1


    def update(self, robot):
        """Placeholder for method to update task variables."""
        pass

    def isActive(self):
        """Returns task activity status. Default is active."""
        return True 

    def setDesired(self, value):
        """Sets the desired outcome."""
        self.sigma_d = value

    def getDesired(self):
        """Returns the desired outcome."""
        return self.sigma_d

    def getJacobian(self):
        """Returns the Jacobian of the task."""
        return self.J

    def getError(self):
        """Returns the error of the task."""
        return self.err
    
    def Joint_limit_activation(self):
        """Returns the joint limit activation of the task."""
        return self.limit_activation

class Position3D(Task):
    """
    Subclass of Task, representing the 3D position task.
    """

    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.err = np.zeros((3,1))# Initialize error with proper dimensions 3 x 1
        self.link = link # The link for the task
        self.limit_activation = 1

    def update(self, robot):
        """Updates the task variables based on the current state of the robot."""
        self.J = robot.get_link_Jacobian(self.link)[0:3]
        self.err = (self.getDesired() - robot.get_link_Transform(self.link)[0:3, 3].reshape(3, 1)).reshape(3, 1)
        
class Orientation3D(Task):
    """
    Subclass of Task, representing the 3D orientation task.
    """

    def __init__(self, name, desired,link):
        super().__init__(name, desired)
        self.err = np.zeros((1)) # Initialize error with proper dimensions
        self.link = link # The link for the task
        self.limit_activation = 1

    def update(self, robot):
        """Updates the task variables based on the current state of the robot."""
        self.J = robot.get_link_Jacobian(self.link)[-1].reshape(1, 6)
        angle = np.arctan2(robot.get_link_Transform(self.link)[1,0], robot.get_link_Transform(self.link)[0,0])
        self.err = self.sigma_d - np.array([[angle]]) # Update task error

class Configuration3D(Task):
    """
    Subclass of Task, representing the 3D configuration task.
    """

    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link 
        self.activation_function = 1

    def update(self, robot):
        """Updates the task variables based on the current state of the robot."""
        self.J = np.concatenate([robot.get_link_Jacobian(self.link)[0:3], robot.get_link_Jacobian(self.link)[-1].reshape(1,6)])
        sigma = np.zeros((4,1))
        sigma[0:3] = robot.get_link_Transform(self.link)[0:3, 3].reshape(3, 1)
        sigma[3] = np.arctan2(robot.get_link_Transform(self.link)[1,0], robot.get_link_Transform(self.link)[0,0])
        self.err = (self.sigma_d - sigma).reshape(4,1)

class JointPosition(Task):
    """
    Subclass of Task, representing the joint position task.
    """

    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.link = link # The link for the task

    def update(self, robot):
        """Updates the task variables based on the current state of the robot."""
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
        q = robot.getJointPos(self.link)

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
            
            
            
            
            
class JointLimit3D(Task):  
    def __init__(self, name, threshold, Q, link):
        super().__init__(name, threshold)
        self.link = link
        self.alpha = threshold[0]
        self.delta = threshold[1]
        self.qi_min = Q[0]
        self.qi_max = Q[1]
        self.active = False

        # self.J = np.zeros((1,5))   
        # self.J = np.zeros((1,5)) 
        # self.err = np.zeros((1))  
        # def _init_(self, name, thresh, min_angle, max_angle, link): 
        # super()._init_(name, thresh)
        # self.min_angle = min_angle 
        # self.max_angle = max_angle
        # # Assigning the value to the maximal and minimal possible angle
        # self.ralpha =  thresh[0]
        # self.rdelta = thresh[1]
        # # Assigning the value to delta and alpha
        # self.link = link
        # self.active = False
        # self.a = 0

    # wraps the angle between -pi to +pi 
    def wrap_angle(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi

    def update(self, robot):    
        self.J = (robot.getEEJacobian(self.link)[5,:]).reshape(1,6) 
        # print('jac',self.J)  
        # self.err = self.getDesired() - robot.getJointPos(0)   

        # self.distance = self.getDesired() - robot.getJointPos(self.link)  
          
        # orien = robot.getJointPos(self.link)  
        link_transform = robot.get_Se_LTransform(self.link)     
        q = np.arctan2(link_transform[1,0], link_transform[0,0]) 
        print('angle', q)        

        if self.limit_activation == 1 and q > (self.qi_min + self.delta): 
            self.limit_activation = 0   
            self.active = False  
            #self.err = 0.0   
  
        if self.limit_activation == -1 and q < (self.qi_max - self.delta):   
            self.limit_activation = 0 
            self.active = False 
            #self.err = 0.0   

        if self.limit_activation == 0 and q > (self.qi_max - self.alpha):   
            self.limit_activation = -1 
            self.active = True 
            #self.err = -1.0   

        if self.limit_activation == 0 and q < (self.qi_min + self.alpha):   
            self.limit_activation = 1 
            self.active = True   
            #self.err = 1.0   
        self.err = self.limit_activation
            

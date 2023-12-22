import rclpy
from rclpy.node import Node
import math
from simple_pid import PID
import numpy as np
import time
from geometry_msgs.msg import Quaternion, PoseStamped



class Point_class:
    def __init__(self,x:float,y:float,z:float):
        self.x=x
        self.y=y
        self.z=z

class Quaternion_class:
    def __init__(self,x:float,y:float,z:float,w:float=None): 
        #If the Inputs were (x,y,z), then we are using Euler Cooridnates. If (x,y,z,w) then the input is Quaternion  
        if w != None:
            self.x=x
            self.y=y
            self.z=z
            self.w=w
        else: #Euler to Quaternion
            self.x = math.sin(x/2) * math.cos(y/2) * math.cos(z/2) - math.cos(x/2) * math.sin(y/2) * math.sin(z/2)
            self.y = math.cos(x/2) * math.sin(y/2) * math.cos(z/2) + math.sin(x/2) * math.cos(y/2) * math.sin(z/2)
            self.z = math.cos(x/2) * math.cos(y/2) * math.sin(z/2) - math.sin(x/2) * math.sin(y/2) * math.cos(z/2)
            self.w = math.cos(x/2) * math.cos(y/2) * math.cos(z/2) + math.sin(x/2) * math.sin(y/2) * math.sin(z/2)
        #print(self.x, self.y, self.z, self.w)
    def quaternion(self):
        return np.array([self.x, self.y, self.z, self.w])
    def euler_zyx(self):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x=self.x
        y=self.y
        z=self.z
        w=self.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return np.array([roll_x, pitch_y, yaw_z]) #In radians

class Pose_class:
    def __init__(self,position:Point_class=None, orientation:Quaternion_class=None):
        self.position=position
        self.orientation=orientation
    def get_position(self):
        return np.array([self.position.x, self.position.y, self.position.z])




class controller_cleaned(Node):
    def __init__(self):
        super().__init__('controller_cleaned')
        ##################################
        #initializations pub/sub
        self.pub=self.create_publisher(Quaternion, '/control_commands', 10)
        self.sub_current_pose = self.create_subscription(PoseStamped,'/Pos',  self.set_current_pose,10)
        self.sub_goal_pose = self.create_subscription(PoseStamped,'/Control_Pose',  self.set_goal_pose,10)


        
     

        ##################################
        #initializations
        #Setting Points 
        self.current_state = Pose_class(Point_class(0,0,0), Quaternion_class(0,0,0))
        self.goal_point = Pose_class(Point_class(0,0,0), Quaternion_class(0,0,0))

        #PID Controllers initialization
        #throttle_pid = PID(0, 0, 0, setpoint=0)
        self.z_dot_pid = PID(100, 20, 0.0, setpoint=0)

        #y_pid = PID(0, 0, 0, setpoint=0)
        #x_pid = PID(0, 0, 0, setpoint=0)
        #x_pid.output_limits = (-0.02, 0.02)       #maximum drone pitch angle
        #y_pid.output_limits = (-0.02, 0.02)       #maximum drone roll angle

        self.x_dot_pid = PID(0.4, 0.001, 0, setpoint=0)
        self.y_dot_pid = PID(0.4, 0.001, 0, setpoint=0)
        self.x_dot_pid.output_limits = (-0.5, 0.5)       #maximum drone pitch angle
        self.y_dot_pid.output_limits = (-0.5, 0.5)       #maximum drone roll angle

        self.yaw_dot_pid = PID(2, 0, 0, setpoint=0)
        self.yaw_dot_pid.output_limits = (-0.5, 0.5)       #maximum drone yaw rate

        self.roll_pid = PID(0.1, 0.001, 0.05, setpoint=0)
        self.pitch_pid = PID(0.1, 0.001, 0.05, setpoint=0)

        self.x_prev=0
        self.y_prev=0
        self.z_prev=0
        self.yaw_prev=0
        self.time_prev=time.time()
        ##################################
        
        ##################################
        #functions
    
    def pi_2_pi(self,angle):
        #A function to wrap the angle between -pi and pi (for numerical stability)
        return (angle + np.pi) % (2 * np.pi) - np.pi



    def set_goal_pose(self,goal_msg): # Pose msg
        """
            This is the callback of a ros subscriber,
            It receives the Goal Pose and store it in a global variable (goal_point)
        """
        
        #print(goal_msg)
        goal_msg=goal_msg.pose
        #print(goal_msg)
        x,y,z,w=goal_msg.orientation.x,goal_msg.orientation.y,goal_msg.orientation.z,goal_msg.orientation.w
        self.goal_point = Pose_class(goal_msg.position,Quaternion_class(x,y,z,w))
        #goal_point.position.z=2
        #print("position: ",goal_point.get_position())
        #print("orientation quaternion: ",goal_point.orientation.quaternion())
        #print("orientation euler_zyx: ",goal_point.orientation.euler_zyx())
        #print("goal_point set")


    def set_current_pose(self,current_msg): # Pose msg
        """
            This is the callback of a ros subscriber,
            It receives the Current Drone Pose and store it in a global variable (current_state)
            Then do one iteration in the control loop
        """
        
        current_msg=current_msg.pose
        x,y,z,w=current_msg.orientation.x,current_msg.orientation.y,current_msg.orientation.z,current_msg.orientation.w
        #print(current_msg)
        #print(x,y,z,w)
        self.current_state = Pose_class(current_msg.position,Quaternion_class(x,y,z,w))
        #print("position: ",current_state.get_position())
        #print("orientation quaternion: ",current_state.orientation.quaternion())
        #print("orientation euler_zyx: ",current_state.orientation.euler_zyx())
        #print("current_state set")
        self.control_loop()

    def control_loop(self):
        """
            This function do the control loop for x,y,z positions and roll,pitch,yaw angles
            using 6 pid controllers
            and publish the control commands(TYPR) to ros
            Inputs: goal point pose (x,y,z,roll,pitch,yaw)
                    drone current pose (x,y,z,roll,pitch,yaw)
            Outputs: publish TYPR control commands
        """
        
        time_now = time.time()
        dt = time_now - self.time_prev

        theta=self.current_state.orientation.euler_zyx()[2]      #Drone Orientation

        dx = (self.current_state.position.x-self.x_prev)/dt
        dy = (self.current_state.position.y-self.y_prev)/dt
        dz = (self.current_state.position.z-self.z_prev)/dt
        self.x_prev = self.current_state.position.x
        self.y_prev = self.current_state.position.y
        self.z_prev = self.current_state.position.z
        yaw_dot= (theta-self.yaw_prev)/dt
        self.yaw_prev = theta
        self.time_prev = time_now
        #print("dt = ",dt)
        print("dx = ",dx,"dy = ",dy,"dz = ",dz,"dt = ",dt,"yaw_dot = ",yaw_dot)
        #Height Controller
    #   throttle_pid.setpoint= goal_point.position.z
    #  throttle_ = throttle_pid(current_state.position.z)

        #Yaw angle Controller
        phi= self.pi_2_pi(self.goal_point.orientation.euler_zyx()[2] - self.current_state.orientation.euler_zyx()[2]) #In radians

        self.yaw_dot_pid.setpoint = phi

        print("yaw_dot_pid.setpoint = ",self.yaw_dot_pid.setpoint)
        yaw_ = self.yaw_dot_pid(yaw_dot)
        #print("phi = ",phi)
        
        #roll angle Controller
        alpha= self.pi_2_pi(-self.current_state.orientation.euler_zyx()[0]) #In radians
        roll_ = -self.roll_pid(alpha)
        #print("roll = ",roll_)
        
        #pitch angle Controller
        beta= self.pi_2_pi(-self.current_state.orientation.euler_zyx()[1]) #In radians
        pitch_ = -self.pitch_pid(beta)
        #print("pitch = ",pitch_)

        #x,y position Controller
        delta = self.goal_point.get_position()-self.current_state.get_position()  #Difference in positions in x,y,z between goal point and current state point
                                                                        # with reference to the global frame
        x_=delta[0]*math.cos(theta)+delta[1]*math.sin(theta)    #Get the difference in x with reference to the drone frame
        y_=-delta[0]*math.sin(theta)+delta[1]*math.cos(theta)   #Get the difference in y with reference to the drone frame
        #print("delta = ",delta)

        x_dot = dx*math.cos(theta) + dy*math.sin(theta)
        y_dot = -dx*math.sin(theta) + dy*math.cos(theta)
        #x,y velocity Controller
        x_dot_pid_output = self.x_dot_pid(x_dot)
        y_dot_pid_output = self.y_dot_pid(y_dot)
        throttle_ = self.z_dot_pid(dz)
        
        self.x_dot_pid.setpoint=x_*0.3
        self.y_dot_pid.setpoint=y_*0.3
        self.z_dot_pid.setpoint=delta[2]
        if x_>0.7:
            self.x_dot_pid.setpoint = 0.4
        elif x_<-0.7:
            self.x_dot_pid.setpoint = -0.4
        if y_>0.7:
            self.y_dot_pid.setpoint = 0.4
        elif y_<-0.7:
            self.y_dot_pid.setpoint = -0.4
        if delta[2]>0.8:
            self.z_dot_pid.setpoint = 10
        elif delta[2]<-0.8:
            self.z_dot_pid.setpoint = -10
        print("x_dot_pid.setpoint = ",self.x_dot_pid.setpoint,"y_dot_pid.setpoint = ",self.y_dot_pid.setpoint,"z_dot_pid.setpoint = ",self.z_dot_pid.setpoint)

        self.roll_pid.setpoint =  y_dot_pid_output       #setpoint of the roll angle controller
        self.pitch_pid.setpoint = -x_dot_pid_output      #setpoint of the pitch angle controller

        #print("pitch_command = ",pitch_)
        #print("roll_command = ",roll_)
        #print("throttle_command = ",throttle_)
        #print("yaw_command = ",yaw_)
        
        print("Error:","(x,y,z):",delta,"(roll,pitch,yaw):",self.goal_point.orientation.euler_zyx()-self.current_state.orientation.euler_zyx())

        control_commands=Quaternion()
        control_commands.x = pitch_
        control_commands.y = roll_
        control_commands.z = throttle_
        control_commands.w = yaw_
        print("Control Commands: ",control_commands)
        self.pub.publish(control_commands)



def main(args=None):
    try:
        rclpy.init(args=args)
        node=controller_cleaned()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
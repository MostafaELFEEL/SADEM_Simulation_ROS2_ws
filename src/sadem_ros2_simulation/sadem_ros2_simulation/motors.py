import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Quaternion



class motors(Node):
    def __init__(self):
        super().__init__('motors')
        self.pub=self.create_publisher(Quaternion,'/Motor_Speeds',10)
        self.sub=self.create_subscription(Quaternion,'/control_commands',self.control_commands_to_motor_speeds,10)

        #############################  PARAMETERS  #########################################
        self.gains=np.array([1,1,1,1])
        self.biases=np.array([0,0,23.0915,0])

        
    def control_commands_to_motor_speeds(self,commands):
        #print(commands)
        commands = np.array([commands.x,commands.y,commands.z,commands.w])
        #print(commands)
        pitch,roll,thrust,yaw = (commands+self.biases)*self.gains
        print('commands',pitch,roll,thrust,yaw)

        FR= thrust*(1 - pitch - roll + yaw)
        FL= thrust*(1 - pitch + roll - yaw)
        BR= thrust*(1 + pitch - roll - yaw)
        BL= thrust*(1 + pitch + roll + yaw)
        print('speeds',FR,FL,BR,BL)
        

        msg=Quaternion()
        msg.x=FR
        msg.y=FL
        msg.z=BR
        msg.w=BL
        self.pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = motors()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

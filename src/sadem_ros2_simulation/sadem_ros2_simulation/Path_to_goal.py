import rclpy
from rclpy.node import Node
from math import dist
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path


class path_to_goal(Node):
	def __init__(self):
		super().__init__('path_to_goal')
		###################################
		#initialize subscribers and publishers
		self.point_pub=self.create_publisher(Quaternion,'/map/state',2000)
		self.pub= self.create_publisher(PoseStamped,'/Control_Pose',10)
		self.sub=self.create_subscription(Path,'/path',self.get_path,10)
		###################################
		#initializations
		self.path=[]
		self.current_pose=[]
		self.i=0
		self.current_sub=None
	
		###################################
		#loop
		

	def get_path(self,data):
		self.path=data.poses
		self.i=0
		self.current_sub=self.create_subscription(PoseStamped,'/Pos',self.get_current_pose,10)
		print("path: ", self.path)





	def get_current_pose(self,data):
		self.current_pose = data
		current_point=[ self.current_pose.pose.position.x,
						self.current_pose.pose.position.y,
						self.current_pose.pose.position.z]
		#print("i: ", self.i)
		try:
			next_point=[self.path[self.i].pose.position.x,
						self.path[self.i].pose.position.y,
						self.path[self.i].pose.position.z]
			#print("current_point: ", current_point)
			#print("next_point: ", next_point)
			#print("i: ", self.i)
			distance=dist(next_point,current_point)
			if distance < 0.5:
				#print("distance: ", distance, " i: ", self.i)
				self.i+=1
			#print("path",len(self.path))
			self.pub.publish(self.path[self.i])
			x_,y_,z_ = self.path[self.i].pose.position.x, self.path[self.i].pose.position.y, self.path[self.i].pose.position.z
			
			msg=Quaternion()
			msg.x=x_
			msg.y=y_
			msg.z=z_
			msg.w=6.0
			self.point_pub.publish(msg)

		except:
			print("Last point reached")
			self.pub.publish(self.path[-1])
			x_,y_,z_ = self.path[-1].pose.position.x, self.path[-1].pose.position.y, self.path[-1].pose.position.z
			
			msg1=Quaternion()
			msg1.x=x_
			msg1.y=y_
			msg1.z=z_
			msg1.w=6.0
			self.point_pub.publish(msg1)


def main(args=None):
	try:
		rclpy.init(args=args)
		node=path_to_goal()
		rclpy.spin(node)
		node.destroy_node()
		rclpy.shutdown()
	except KeyboardInterrupt:
		pass










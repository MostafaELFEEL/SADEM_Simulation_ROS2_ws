import rclpy 
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import dist
from scipy.interpolate import splprep, splev
from time import sleep
import matplotlib.pyplot as plt
import os

class node_class:
	def __init__(self, position,g,h,f,parent):
		self.parent = parent
		self.position = position
		self.g =g 
		self.h =h
		self.f =f


class pathplanning(Node):
	def __init__(self):
		super().__init__('path_planning')
		self.sub_current_pose = self.create_subscription(PoseStamped,'/Pos',  self.set_current_pose,10)
		self.sub_goal_pose = self.create_subscription(PoseStamped,'/Target_Pos',  self.set_goal_pose,10)
		self.pub = self.create_publisher(Quaternion,'/map/state',2)
		self.reset_map_pub=self.create_publisher(Bool,'/reset',2)
		self.reset_pub=self.create_publisher(Bool,'/map/reset',10)
		self.path_pub=self.create_publisher(Path,'/path',10)
		self.timer=self.create_timer(0.2,self.timer_callback)



		# Get a parameter named 'your_parameter_name' with a default value of 'default_value'
		self.declare_parameter('map_number',1)
		your_parameter_value = self.get_parameter('map_number').value
		
		self.rate=1
		self.ratemap=1/20
			
		self.goal_point2=None
		self.start_point2=None
		self.goal_point_prev=[0,0,0]
		
		self.current_point=None
		self.current_posestamped=None
		self.goal_point=None
		self.goal_posestamped=None
		self.current_orientation=None

		self.counter=0
		self.goal_changed=False

		sleep(self.rate)
		msg=Bool()
		msg.data=True
		self.reset_map_pub.publish(msg)
		sleep(self.rate)

		path=os.path.abspath('')+'/sadem_simulation_ros2_ws/src/sadem_ros2_simulation/sadem_ros2_simulation/maps/map'+str(your_parameter_value)+'.png'
		self.map=np.array(plt.imread(path))   #<-------------------------------------
		self.z=5
		self.map_scale=2
		
		non_white_pixels = (self.map[:, :, :3] != 1.0).any(axis=-1)
		self.map=np.ones((self.map.shape[0],self.map.shape[1],self.z))
		self.map[non_white_pixels] = 0
		self.map_to_coppeliasim()
############################
	#functions
	
	def timer_callback(self):
		#print("Waiting for goal to be changed")
		if self.goal_changed and self.counter >=10:
			self.timer.cancel()
			msg=Bool()
			msg.data=True
			self.reset_pub.publish(msg)


			self.path_planning(self.map,self.current_point,self.goal_point)
			self.counter=0
			self.goal_changed=False
			self.sub_goal_pose = self.create_subscription( PoseStamped,'/Target_Pos', self.set_goal_pose,10)
			self.timer.reset()



	def map_to_coppeliasim(self):
		msg = Quaternion()
		for i in range(self.map.shape[0]):
			for j in range(self.map.shape[1]):
				for k in range(self.z): 
					if self.map[i,j,k]==0:  #send boundaries to coppeliasim
						sleep(self.ratemap)
						msg.x = i/self.map_scale
						msg.y = j/self.map_scale
						msg.z = k/self.map_scale
						msg.w = 0.0
						self.pub.publish(msg)
						
		

	

	
	def convert_copelia_to_map(self,copelia_pose,round_value=True):
	
		map_pose=[]
		#print("copelia_pose",copelia_pose)
		x=copelia_pose[0]*self.map_scale
		y=copelia_pose[1]*self.map_scale
		z=copelia_pose[2]*self.map_scale
		if round_value:
			x=round(x)
			y=round(y)
			z=round(z)
		map_pose.append(x)
		map_pose.append(y)
		map_pose.append(z)
	
		return map_pose

	def convert_map_to_copelia(self,map_pose,round_value=True):
		
		copelia_pose=[]
		#print("map_pose",map_pose)
		x=map_pose[0]/self.map_scale
		y=map_pose[1]/self.map_scale
		z=map_pose[2]/self.map_scale
		if round_value:
			x=round(x,2)
			y=round(y,2)
			z=round(z,2)
		copelia_pose.append(x)
		copelia_pose.append(y)
		copelia_pose.append(z)
	
		return copelia_pose

	
	def successors(self,node):
		List_successors=[]
		for i in range(-1,2):
			for j in range(-1,2):
				for k in range(-1,2):
					if i==0 and j==0 and k==0:
						continue
					if node.position[0]+i<0 or node.position[1]+j<0 or node.position[2]+k<0 or \
					   node.position[0]+i>=self.map.shape[0] or node.position[1]+j>=self.map.shape[1] or node.position[2]+k>=self.map.shape[2]:
						continue
					if self.map[node.position[0]+i,node.position[1]+j,node.position[2]+k]==0:
						continue

					List_successors.append(node_class([node.position[0]+i,node.position[1]+j,node.position[2]+k],None,None,None,node))
						
		return List_successors
		


	def A_star(self,start,goal):
		open_list = []
		closed_list = []
		open_list.append(start)

		while len(open_list) > 0:
			open_list.sort(key=lambda x: x.f, reverse=False)
			parent = open_list.pop(0)
			closed_list.append(parent)

			if parent.position == goal.position:
				path=[]
				while parent.parent is not None:
					path.append(parent.position)
					parent = parent.parent

				path.append(start)
				path.reverse()

				return path
			
			childern=self.successors(parent)

			for child in childern:
				child.g=parent.g+dist(parent.position,child.position)
				child.h=dist(child.position,goal.position)
				z_cost=abs(child.position[2]-goal.position[2])*10
				
				child.f=child.g+child.h+z_cost
				
				flag=False

				for i,closed_child in enumerate(closed_list):
					if child.position==closed_child.position:
						flag=True
						if child.f<closed_child.f:
							closed_list[i]=child
						break

				if flag:
					continue

				for i,open_child in enumerate(open_list):
					if child.position==open_child.position:
						flag=True
						if child.f<open_child.f:
							open_list[i]=child
						break
				
				if flag:
					continue

				open_list.append(child)




	def set_current_pose(self,data):
		#print("Current Pose Set")
		self.current_posestamped=data
		self.start_point2=[data.pose.position.x,data.pose.position.y,data.pose.position.z]
		self.current_point=self.convert_copelia_to_map([data.pose.position.x,data.pose.position.y,data.pose.position.z])
		

	def set_goal_pose(self,data):
		
		self.goal_posestamped=data
		self.goal_point2=[data.pose.position.x,data.pose.position.y,data.pose.position.z]
		self.goal_point=self.convert_copelia_to_map([data.pose.position.x,data.pose.position.y,data.pose.position.z])
		if dist(self.goal_point,self.goal_point_prev)>0.1:
			self.goal_changed=True
			self.counter=0
		else:
			if self.counter>=10 and self.goal_changed==True:
				self.destroy_subscription(self.sub_goal_pose)
			self.counter+=1
		self.goal_point_prev=self.goal_point
		
		

	def spline(self,path):
		path=np.array(path)
		x = path[:, 0]
		y = path[:, 1]
		z = path[:, 2]
		# Fit a spline to the data
		tck, u = splprep([x, y, z], s=0, per=False)
		# Define new range for the spline parameter
		u_new = np.linspace(u.min(), u.max(), 200)
		# Evaluate the spline at the new parameter values
		x_spline, y_spline, z_spline = splev(u_new, tck)
		path=np.array([x_spline,y_spline, z_spline]).T
		return path

	def path_planning(self,Map,current_point,goal_point):
		
		start=node_class(current_point,0,0,0,None)  #position,g,h,f,parent
		start_x,start_y,start_z=self.start_point2#convert_map_to_copelia(current_point)
		sleep(self.ratemap)
		msg=Quaternion()
		msg.x=start_x
		msg.y=start_y
		msg.z=start_z
		msg.w=2.0
		self.pub.publish(msg)


		#print(start.position)
		#goal out of bounds

		if goal_point[0]>=self.map.shape[0] or goal_point[1]>=self.map.shape[1] or goal_point[2]>=self.map.shape[2]\
			  or goal_point[0]<0 or goal_point[1]<0 or goal_point[2]<0:
			print("Goal out of bounds")
			return
			#exit()
		if Map[goal_point[0],goal_point[1],goal_point[2]]==0:
			print("Goal is in obstacle")
			return
		

			#exit()
		goal=node_class(goal_point,0,0,0,None)
		goal_x,goal_y,goal_z=self.goal_point2#convert_map_to_copelia(goal_point)
		sleep(self.ratemap)
		msg=Quaternion()
		msg.x=goal_x
		msg.y=goal_y
		msg.z=goal_z
		msg.w=3.0
		self.pub.publish(msg)


		#print(goal.position)
		

		final_path = self.A_star(start,goal)

		#print("len1final_path",len(final_path))
		final_path.pop(0)
		final_path.insert(0,self.convert_copelia_to_map(self.start_point2,round_value=False))
		final_path.pop(-1)
		final_path.append(self.convert_copelia_to_map(self.goal_point2,round_value=False))
		#print("len2final_path",len(final_path))
		

		if len(final_path)>3:
			final_path=self.spline(final_path)

		print("final_path",final_path)
		orientations=[]
		current_orientation=[self.current_posestamped.pose.orientation.x,
					   self.current_posestamped.pose.orientation.y,
					   self.current_posestamped.pose.orientation.z,
					   self.current_posestamped.pose.orientation.w]
		
		current_orientation=euler_from_quaternion(current_orientation)[2]

		#print("current",current_orientation)
		goal_orientation=[self.goal_posestamped.pose.orientation.x,
					self.goal_posestamped.pose.orientation.y,
					self.goal_posestamped.pose.orientation.z,
					self.goal_posestamped.pose.orientation.w]
		
		goal_orientation=euler_from_quaternion(goal_orientation)[2]

		#print("goal",goal_orientation)
		orientation_diff=goal_orientation-current_orientation
		orientation_step=orientation_diff/len(final_path)

		#print("here",orientation_diff,orientation_step,len(final_path))
		for i in range(len(final_path)):
			#orientations.append(math.atan2(final_path[i][1]-final_path[i-1][1],final_path[i][0]-final_path[i-1][0])) #angle between two points
			#print("i",i,orientation_step*i)
			orientation=current_orientation+orientation_step*i
			#print(orientation)
			orientations.append(orientation)


		msg = Path()
		msg.header.frame_id = "map"
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.poses.append(self.current_posestamped)
		for i in final_path:
			pose = PoseStamped()
			x_,y_,z_=self.convert_map_to_copelia(i)
			pose.header.frame_id = "map"
			pose.pose.position.x = x_
			pose.pose.position.y = y_
			pose.pose.position.z = z_
			quaternion=quaternion_from_euler(0,0,orientations.pop(0))
			pose.pose.orientation.x = quaternion[0]
			pose.pose.orientation.y = quaternion[1]
			pose.pose.orientation.z = quaternion[2]
			pose.pose.orientation.w = quaternion[3]
			msg.poses.append(pose)
		msg.poses.append(self.goal_posestamped)
		self.path_pub.publish(msg)
		for i in final_path:
			x_,y_,z_=self.convert_map_to_copelia(i)
			sleep(self.ratemap)
			msg=Quaternion()
			msg.x=x_
			msg.y=y_
			msg.z=z_
			msg.w=4.0
			self.pub.publish(msg)
			
def main(args=None):
	try:
		rclpy.init(args=args)
		node = pathplanning()
		rclpy.spin(node)
		node.destroy_node()
		rclpy.shutdown()
	except KeyboardInterrupt:
		pass

#!/usr/bin/env python

from copy import deepcopy
import math
import numpy
import random
from threading import Thread, Lock
import sys

import actionlib
import control_msgs.msg
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import sensor_msgs.msg
import tf
import trajectory_msgs.msg
from random import randrange, uniform


def convert_to_message(T):
	t = geometry_msgs.msg.Pose()
	position = tf.transformations.translation_from_matrix(T)
	orientation = tf.transformations.quaternion_from_matrix(T)
	t.position.x = position[0]
	t.position.y = position[1]
	t.position.z = position[2]
	t.orientation.x = orientation[0]
	t.orientation.y = orientation[1]
	t.orientation.z = orientation[2]
	t.orientation.w = orientation[3]		
	return t

def convert_from_message(msg):
	R = tf.transformations.quaternion_matrix((msg.orientation.x,
											  msg.orientation.y,
											  msg.orientation.z,
											  msg.orientation.w))
	T = tf.transformations.translation_matrix((msg.position.x, 
											   msg.position.y, 
											   msg.position.z))
	return numpy.dot(T,R)

def convert_from_trans_message(msg):
	R = tf.transformations.quaternion_matrix((msg.rotation.x,
											  msg.rotation.y,
											  msg.rotation.z,
											  msg.rotation.w))
	T = tf.transformations.translation_matrix((msg.translation.x, 
											   msg.translation.y, 
											   msg.translation.z))
	return numpy.dot(T,R)
   



class MoveArm(object):

	def __init__(self):
		print "Motion Planning Initializing..."
		# Prepare the mutex for synchronization
		self.mutex = Lock()
		self.test = []

		# Some info and conventions about the robot that we hard-code in here
		# min and max joint values are not read in Python urdf, so we must hard-code them here
		self.num_joints = 7
		self.q_min = []
		self.q_max = []
		self.q_min.append(-3.1459);self.q_max.append(3.1459)
		self.q_min.append(-3.1459);self.q_max.append(3.1459)
		self.q_min.append(-3.1459);self.q_max.append(3.1459)
		self.q_min.append(-3.1459);self.q_max.append(3.1459)
		self.q_min.append(-3.1459);self.q_max.append(3.1459)
		self.q_min.append(-3.1459);self.q_max.append(3.1459)
		self.q_min.append(-3.1459);self.q_max.append(3.1459)
		# How finely to sample each joint
		self.q_sample = [0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.1]
		self.joint_names = ["lwr_arm_0_joint",
							"lwr_arm_1_joint",
							"lwr_arm_2_joint",
							"lwr_arm_3_joint",
							"lwr_arm_4_joint",
							"lwr_arm_5_joint",
							"lwr_arm_6_joint"]

		# Subscribes to information about what the current joint values are.
		rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, 
						 self.joint_states_callback)

		# Subscribe to command for motion planning goal
		rospy.Subscriber("/motion_planning_goal", geometry_msgs.msg.Transform,
						 self.move_arm_cb)

		# Publish trajectory command
		self.pub_trajectory = rospy.Publisher("/joint_trajectory", trajectory_msgs.msg.JointTrajectory, 
											  queue_size=1)		

		# Initialize variables
		self.joint_state = sensor_msgs.msg.JointState()

		# Wait for moveit IK service
		rospy.wait_for_service("compute_ik")
		self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
		print "IK service ready"

		# Wait for validity check service
		rospy.wait_for_service("check_state_validity")
		self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
													  moveit_msgs.srv.GetStateValidity)
		print "State validity service ready"

		# Initialize MoveIt
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group_name = "lwr_arm"
		self.group = moveit_commander.MoveGroupCommander(self.group_name) 
		print "MoveIt! interface ready"

		# Options
		self.subsample_trajectory = True
		print "Initialization done."

	def get_joint_val(self, joint_state, name):
		if name not in joint_state.name:
			print "ERROR: joint name not found"
			return 0
		i = joint_state.name.index(name)
		return joint_state.position[i]

	def set_joint_val(self, joint_state, q, name):
		if name not in joint_state.name:
			print "ERROR: joint name not found"
		i = joint_state.name.index(name)
		joint_state.position[i] = q

	""" Given a complete joint_state data structure, this function finds the values for 
	our arm's set of joints in a particular order and returns a list q[] containing just 
	those values.
	"""
	def q_from_joint_state(self, joint_state):
		q = []
		for i in range(0,self.num_joints):
			q.append(self.get_joint_val(joint_state, self.joint_names[i]))
		return q

	""" Given a list q[] of joint values and an already populated joint_state, this 
	function assumes that the passed in values are for a our arm's set of joints in 
	a particular order and edits the joint_state data structure to set the values 
	to the ones passed in.
	"""
	def joint_state_from_q(self, joint_state, q):
		for i in range(0,self.num_joints):
			self.set_joint_val(joint_state, q[i], self.joint_names[i])

	""" This function will perform IK for a given transform T of the end-effector. It 
	returns a list q[] of 7 values, which are the result positions for the 7 joints of 
	the left arm, ordered from proximal to distal. If no IK solution is found, it 
	returns an empy list.
	"""
	def IK(self, T_goal):
		req = moveit_msgs.srv.GetPositionIKRequest()
		req.ik_request.group_name = self.group_name
		req.ik_request.robot_state = moveit_msgs.msg.RobotState()
		req.ik_request.robot_state.joint_state = self.joint_state
		req.ik_request.avoid_collisions = True
		req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
		req.ik_request.pose_stamped.header.frame_id = "world_link"
		req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
		req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
		req.ik_request.timeout = rospy.Duration(3.0)
		res = self.ik_service(req)
		q = []
		if res.error_code.val == res.error_code.SUCCESS:
			q = self.q_from_joint_state(res.solution.joint_state)
		return q

	""" This function checks if a set of joint angles q[] creates a valid state, or 
	one that is free of collisions. The values in q[] are assumed to be values for 
	the joints of the left arm, ordered from proximal to distal. 
	"""
	def is_state_valid(self, q):
		req = moveit_msgs.srv.GetStateValidityRequest()
		req.group_name = self.group_name
		current_joint_state = deepcopy(self.joint_state)
		current_joint_state.position = list(current_joint_state.position)
		self.joint_state_from_q(current_joint_state, q)
		req.robot_state = moveit_msgs.msg.RobotState()
		req.robot_state.joint_state = current_joint_state
		res = self.state_valid_service(req)
		return res.valid
	
	
	class node:
		def __init__(self):
			self.id = -1
			self.position = numpy.zeros(7)
			self.previous_node_id = -1
			
	def get_node_by_id(self,id1,nlist):
		
		for n in nlist:
			if n.id == id1:
				return n
	
	def print_node(self,n):
		print "node:"
		print n.id
		print n.position
		print n.previous_node_id
		print "end node"
	
	def clear_path(self,final):
		#print "clear_path"
		#print "original length: " + str(len(final))
		final_cleared = []
		final_cleared.append(final[len(final)-1])
		final_cleared.append(final[len(final)-2])
		
		for n in reversed(range(1,len(final)-2)):
			if self.check_colision(final[0],final[n],0) is False:
				
				final_cleared.append(final[n-1])
			else:
				break
	
		final_cleared.append(final[0])
		
		#print "final length"
		#print len(final_cleared)
		#print final_cleared
		final_cleared2 = []
		for n in reversed(range(len(final_cleared))):
			final_cleared2.append(final_cleared[n])
		return final_cleared2

	
	def generate_path(self,node_list,node_goal,node_start):
		j = 0
		#print "goal"
		#self.print_node(node_goal)
		for n in node_list:
			#print ".---."
			#print j
			#self.print_node(n)
			j = j + 1
		
		K = True
		path = []
		nod = node_goal
		while K:
			
			path.append(nod.position)
			nod = self.get_node_by_id(nod.previous_node_id,node_list)
			#nod = node_list[nod.previous_node_id]
			if nod.id == -1:
				K = False
			 
		
		path1 = list(reversed(path))
		
		return path1
	
	def check_colision(self,a,b,t):
		
		punto_no_valido = 0
		
		#print "entering check colision" + str(t)
		#print "point a"
		#print a
		#print "point b"
		#print b
		#print a
		
		c = b - a
		
		#print c
		
		#print self.q_sample
		#raw_input()
		
		max_n = -1
		k = -1
		for j in range(7):
			#print c[j]
			#print self.q_sample[j]
			pepe = c[j]/self.q_sample[j]
			#print abs(pepe)
			if abs(pepe) > max_n:
				max_n = abs(pepe)
				k = j
				#print "minacum"
				#print min_n
			#raw_input()
		samplers_int = []
		samplers = c / self.q_sample
		for g in samplers:
			samplers_int.append(abs(int(g)))
		#print "samplers"
		#print samplers_int
		#print ""
		
		n_sampling = abs(int(max_n))
		#print "sampling number " + str(n_sampling) + " joint: " + str(k)
		#print ""
		#print "vector a"
		#print a
		#print "vector b"
		#print ""
		#print b
		#print ""
		#print "vector c"
		#print c
		#print ""
		#print "samples"
		#print self.q_sample
		#print ""
		#raw_input()
		

		
		distance = self.calculate_distance(a,b)
		#print "distance " + str(distance)
		distance_n = distance / n_sampling
		#print "distance sampling " + str(distance_n)
		#samp_point = a
		hh = numpy.zeros(7)
		sp_previous = a
		sp = numpy.zeros(7)
		#norm_c = numpy.linalg.norm(c)
		hh = c/n_sampling
		kk = hh
		#print "total samplers"
		#print n_sampling
		#print ""

		ss = -1
		for n in range(1,n_sampling):
			
			#print "number of sample"
			#print n
			#print ""
			
			for j in range(7):
				#print "j"
				#print j
				#print ""
				sp[j] = sp_previous[j] + kk[j]
				if a[j] > b[j] and sp[j] <= b[j]:
					sp[j] = b[j]
					continue
				if a[j] < b[j] and sp[j] >= b[j]:
					sp[j] = b[j]
					continue
				
				
				if abs(kk[j]) < self.q_sample[j]:
					sp[j] = sp_previous[j]
					kk[j] = kk[j]+hh[j]
					
				else:
					sp_previous[j] = sp[j]
					kk[j] = hh[j]
					
				#raw_input("jj loop press key")
			#print "sample point at iteration " + str(n)
			#print sp
			#print ""
			#print "previous"
			#print sp_previous
			#print ""
			#vector_c = c * (n+1) / norm_c*distance_n
			#juan = samp_point + vector_c
			
			#print "sampler"
			#print sp
			#print ""
			#print "destination b"
			#print b
			#print ""
		
			#raw_input("sampler loop press")
			
			if self.is_state_valid(sp) is False:
				punto_no_valido = 1
				#print "motherfucker!!!"
				break
			#raw_input()
		if punto_no_valido == 1:
			#print "colision!!!!"
			return False
		
		#raw_input("end check colision")
		
		return True
	
	
	def calculate_distance(self,pos1,pos2):
		a = (pos1[0]-pos2[0])*(pos1[0]-pos2[0]) + (pos1[1]-pos2[1])*(pos1[1]-pos2[1]) + (pos1[2]-pos2[2])*(pos1[2]-pos2[2]) + (pos1[3]-pos2[3])*(pos1[3]-pos2[3]) + (pos1[4]-pos2[4])*(pos1[4]-pos2[4]) + (pos1[5]-pos2[5])*(pos1[5]-pos2[5]) + (pos1[6]-pos2[6])*(pos1[6]-pos2[6])
		return numpy.sqrt(a)
		
	def closest_node(self,listn,pos,pre_dist):
		
		closest_distance = 1000000
		for n in listn:
			dist = self.calculate_distance(n.position,pos)
			if dist < closest_distance:
				closest_distance = dist
				closest_node_return = n
		if closest_distance > pre_dist:
			return closest_node_return
		else:
			return -1
		
	def motion_plan(self, q_start, q_goal, q_min, q_max):
		
		# Replace this with your code

		node_list = []

		if self.check_colision(q_start,q_goal,0) is True:
			ll = [q_start,q_goal]
			return ll
		
		#self.test.append(q_start)
		pre_dist = 0.5
		
		loop = True
		count = 0
		threshold =	 1000000
		#threshold = 0
		
		node_start = self.node()
		node_start.id = -1
		node_start.position = q_start
		
		node_goal = self.node()
		node_goal.id = -2
		node_goal.position = q_goal
		
		node_list.append(node_start)
		
		final_list = []
		#print "init"
		#print q_start
		#print q_goal
		#print ""
		#raw_input()
		while loop:
			#print "loop " + str(count)
			#print ""
			
			#print len(node_list)

			
			if count > threshold:
				break
			
			#1. generate random point
			
			random_point = numpy.zeros(7)
			
			for q in range(7):

				random_point[q] = uniform(q_min[q], q_max[q])
			#print "random point"
			#print random_point
			#print ""
			#self.test.append(random_point)
			
			if self.is_state_valid(random_point) is False:
				#print "random colision"
				#print ""
				
				continue
			
			#2. find closest node

			
			closest_node_temp = self.closest_node(node_list,random_point,pre_dist)
			
			if closest_node_temp == -1:
				#print "closest distance"
				#print ""
				
				continue
			#print "closest"
			#print closest_node_temp.position
			#print ""
			#raw_input('press')
			
			
			#new point at pre_dist
			#vector
			vector = random_point - closest_node_temp.position
			#normalized
			
			norm = numpy.linalg.norm(vector)
			
			vector = vector / norm*pre_dist
			
			new_node_pos = closest_node_temp.position + vector
			
			
			#print "new node"
			#print new_node_pos
			#print self.calculate_distance(new_node_pos,random_point)
			#print ""
			
			#check colision free
			
			
			#result = self.check_colision(cco,closest_node_temp.position,1)
			result = self.check_colision(new_node_pos,closest_node_temp.position,1)
			
			
			if result == False:
				#print "check colision"
				#print ""
				
				continue
			
			#append_node
			
			#raw_input("continue node")
			node_temp = self.node()
			node_temp.id = count
			node_temp.position = new_node_pos
			node_temp.previous_node_id = closest_node_temp.id
			
			#self.print_node(node_temp)
			
			node_list.append(node_temp)
			print len(node_list)
			
			if self.check_colision(node_temp.position,node_goal.position,0) == True:
				node_goal.previous_node_id = node_temp.id
				node_goal.id = count + 1
				node_list.append(node_goal)
				final_list = self.generate_path(node_list,node_goal,node_start)
				#print "final list"
				#print final_list
				#final_list = self.clear_path(final_list)
				#raw_input("pepe")
				#print "final list cleared"
				#print final_list
				
				
				#print "longitud "+str(len(final_list))
				break
			
			
			count = count + 1
			#raw_input()
			
		
		#print "end ------------"
		#pepe = []
		#q_list = [q_start, q_goal]

		return final_list
		
		#return test1
	
	

	

	

	
	def create_trajectory(self, q_list, v_list, a_list, t):
		joint_trajectory = trajectory_msgs.msg.JointTrajectory()
		for i in range(0, len(q_list)):
			point = trajectory_msgs.msg.JointTrajectoryPoint()
			point.positions = list(q_list[i])
			point.velocities = list(v_list[i])
			point.accelerations = list(a_list[i])
			point.time_from_start = rospy.Duration(t[i])
			joint_trajectory.points.append(point)
		joint_trajectory.joint_names = self.joint_names
		return joint_trajectory

	def create_trajectory(self, q_list):
		joint_trajectory = trajectory_msgs.msg.JointTrajectory()
		for i in range(0, len(q_list)):
			point = trajectory_msgs.msg.JointTrajectoryPoint()
			point.positions = list(q_list[i])
			joint_trajectory.points.append(point)
		joint_trajectory.joint_names = self.joint_names
		return joint_trajectory

	def project_plan(self, q_start, q_goal, q_min, q_max):
		q_list = self.motion_plan(q_start, q_goal, q_min, q_max)
		joint_trajectory = self.create_trajectory(q_list)
		return joint_trajectory

	def move_arm_cb(self, msg):
		T = convert_from_trans_message(msg)
		self.mutex.acquire()
		q_start = self.q_from_joint_state(self.joint_state)
		print "Solving IK"
		q_goal = self.IK(T)
		if len(q_goal)==0:
			print "IK failed, aborting"
			self.mutex.release()
			return
		print "IK solved, planning"
		trajectory = self.project_plan(numpy.array(q_start), q_goal, self.q_min, self.q_max)
		if not trajectory.points:
			print "Motion plan failed, aborting"
		else:
			print "Trajectory received with " + str(len(trajectory.points)) + " points"
			self.execute(trajectory)
		self.mutex.release()
		
	def joint_states_callback(self, joint_state):
		self.mutex.acquire()
		self.joint_state = joint_state
		self.mutex.release()

	def execute(self, joint_trajectory):
		self.pub_trajectory.publish(joint_trajectory)

if __name__ == '__main__':
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_arm', anonymous=True)
	ma = MoveArm()
	rospy.spin()
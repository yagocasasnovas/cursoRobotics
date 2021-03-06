#!/usr/bin/env python

import math
import numpy
import time
from threading import Thread, Lock

import rospy
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF

def S_matrix(w):
	S = numpy.zeros((3,3))
	S[0,1] = -w[2]
	S[0,2] =  w[1]
	S[1,0] =  w[2]
	S[1,2] = -w[0]
	S[2,0] = -w[1]
	S[2,1] =  w[0]
	return S

# This is the function that must be filled in as part of the Project.
def cartesian_control(joint_transforms, b_T_ee_current, b_T_ee_desired,red_control, q_current, q0_desired):
	num_joints = len(joint_transforms)
	dq = numpy.zeros(num_joints)
	#-------Fill in your code here ---------------------------

	Kt = 1
	Kr = 1

	b_T_ee_current_inverse = tf.transformations.inverse_matrix(b_T_ee_current)
	
	deltaX = numpy.dot(b_T_ee_desired,b_T_ee_current_inverse)
	
	translation_delta_x = deltaX[:,3]
	
	translation_delta = [translation_delta_x[0],translation_delta_x[1],translation_delta_x[2]]
	
	translation_delta = numpy.array(translation_delta)
	
	norm_X = numpy.linalg.norm(translation_delta)
	
	if norm_X > 0.1:
		
		translation_delta = translation_delta * 0.1 / norm_X
	
	angle,axis = rotation_from_matrix(deltaX)
	rotation_delta_x = tf.transformations.euler_from_matrix(deltaX)

	rotation_delta_x = numpy.array(rotation_delta_x)
	
	norm_W = numpy.linalg.norm(rotation_delta_x)
	
	if norm_W > 1:
		rotation_delta_x = rotation_delta_x / norm_W
	
	#Vee = [translation_delta[0],translation_delta[1],translation_delta[2],rotation_delta_x[0],rotation_delta_x[1],rotation_delta_x[2]]
	
	
	
	Vee = numpy.concatenate((translation_delta,rotation_delta_x),0)
	#Vee = [translation_delta[0],translation_delta[1],translation_delta[2],0,0,0]
	
	
	Vee = numpy.array(Vee)
	#print Vee
	c = numpy.empty([7,6])
	#jacobian
	j = 0
	#print 'ss' + str(num_joints)
	for joint_n in joint_transforms:
		
		
		#joint_n = joint_transforms[2]
		inv_join = tf.transformations.inverse_matrix(joint_n)
		jTee = numpy.dot(b_T_ee_desired,inv_join)
	
		jTee1 = numpy.delete(jTee,3,0)
		jTee2 = numpy.delete(jTee1,numpy.s_[-1:],1)
		jTeeRotInv = tf.transformations.inverse_matrix(jTee2)
	
		jTeeTransf = jTee[:,3]
	
		s_matrix = S_matrix(jTeeTransf)
	
	
		#pepe = tf.transformations.concatenate_matrices(jTeeRotInv,s_matrix)
		pepe = numpy.dot((-1)*jTeeRotInv,s_matrix)

		pepe1 = pepe[:,2]
		pepe2 = jTeeRotInv[:,2]
		pepe3 = numpy.concatenate((pepe1,pepe2),0)
		#print pepe3
		if j == 0:
			jac = numpy.column_stack(pepe3)
			
		else:
			pepe4 = numpy.column_stack(pepe3)
			numpy.append(jac,pepe4,axis=1)

		c[j] = pepe3
		j = j + 1
		
	jac = numpy.column_stack(c)
	#jac_inv = tf.transformations.inverse_matrix(jac)
	jac_inv = numpy.linalg.pinv(jac,rcond=0.0001)
	
	dq = numpy.dot(jac_inv,Vee)
	
	norm_dq = numpy.linalg.norm(dq)
	
	if norm_dq > 1:
		dq = dq / norm_dq
	#print dq1

	#dq = [0.5,0.5,0.5,0.5,0.5,0.5,0.5]
	#----------------------------------------------------------------------
	return dq
    
def convert_from_message(t):
	trans = tf.transformations.translation_matrix((t.translation.x,t.translation.y,t.translation.z))
	rot = tf.transformations.quaternion_matrix((t.rotation.x,
                                                t.rotation.y,
                                                t.rotation.z,
                                                t.rotation.w))
	T = numpy.dot(trans,rot)
	return T

# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
	R = numpy.array(matrix, dtype=numpy.float64, copy=False)
	R33 = R[:3, :3]
	# axis: unit eigenvector of R33 corresponding to eigenvalue of 1
	l, W = numpy.linalg.eig(R33.T)
	i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
	if not len(i):
		raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
	axis = numpy.real(W[:, i[-1]]).squeeze()
	# point: unit eigenvector of R33 corresponding to eigenvalue of 1
	l, Q = numpy.linalg.eig(R)
	i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
	if not len(i):
		raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
	# rotation angle depending on axis
	cosa = (numpy.trace(R33) - 1.0) / 2.0
	if abs(axis[2]) > 1e-8:
		sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
	elif abs(axis[1]) > 1e-8:
		sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
	else:
		sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
	angle = math.atan2(sina, cosa)
	return angle, axis

class CartesianControl(object):

	#Initialization
	def __init__(self):
		#Loads the robot model, which contains the robot's kinematics information
		self.robot = URDF.from_parameter_server()

		#Subscribes to information about what the current joint values are.
		rospy.Subscriber("/joint_states", JointState, self.joint_callback)

		#Subscribes to command for end-effector pose
		rospy.Subscriber("/cartesian_command", Transform, self.command_callback)

		#Subscribes to command for redundant dof
		rospy.Subscriber("/redundancy_command", Float32, self.redundancy_callback)

		# Publishes desired joint velocities
		self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

		#This is where we hold the most recent joint transforms
		self.joint_transforms = []
		self.q_current = []
		self.x_current = tf.transformations.identity_matrix()
		self.R_base = tf.transformations.identity_matrix()
		self.x_target = tf.transformations.identity_matrix()
		self.q0_desired = 0
		self.last_command_time = 0
		self.last_red_command_time = 0

		# Initialize timer that will trigger callbacks
		self.mutex = Lock()
		self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

	def command_callback(self, command):
		self.mutex.acquire()
		self.x_target = convert_from_message(command)
		self.last_command_time = time.time()
		self.mutex.release()

	def redundancy_callback(self, command):
		self.mutex.acquire()
		self.q0_desired = command.data
		self.last_red_command_time = time.time()
		self.mutex.release()        

	def timer_callback(self, event):
		msg = JointState()
		self.mutex.acquire()
		if time.time() - self.last_command_time < 0.5:
			dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_target,
                                   False, self.q_current, self.q0_desired)
			msg.velocity = dq
		elif time.time() - self.last_red_command_time < 0.5:
			dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_current,
                                   True, self.q_current, self.q0_desired)
			msg.velocity = dq
		else:            
			msg.velocity = numpy.zeros(7)
		self.mutex.release()
		self.pub_vel.publish(msg)

	def joint_callback(self, joint_values):
		root = self.robot.get_root()
		T = tf.transformations.identity_matrix()
		self.mutex.acquire()
		self.joint_transforms = []
		self.q_current = joint_values.position
		self.process_link_recursive(root, T, joint_values)
		self.mutex.release()

	def align_with_z(self, axis):
		T = tf.transformations.identity_matrix()
		z = numpy.array([0,0,1])
		x = numpy.array([1,0,0])
		dot = numpy.dot(z,axis)
		if dot == 1: return T
		if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
		rot_axis = numpy.cross(z, axis)
		angle = math.acos(dot)
		return tf.transformations.rotation_matrix(angle, rot_axis)

	def process_link_recursive(self, link, T, joint_values):
		if link not in self.robot.child_map: 
			self.x_current = T
			return
		for i in range(0,len(self.robot.child_map[link])):
			(joint_name, next_link) = self.robot.child_map[link][i]
			if joint_name not in self.robot.joint_map:
				rospy.logerror("Joint not found in map")
				continue
			current_joint = self.robot.joint_map[joint_name]        

			trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0], 
                                                                  current_joint.origin.xyz[1],
                                                                  current_joint.origin.xyz[2]))
			rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0], 
                                                         current_joint.origin.rpy[1],
                                                         current_joint.origin.rpy[2], 'rxyz')
			origin_T = numpy.dot(trans_matrix, rot_matrix)
			current_joint_T = numpy.dot(T, origin_T)
			if current_joint.type != 'fixed':
				if current_joint.name not in joint_values.name:
					rospy.logerror("Joint not found in list")
					continue
				# compute transform that aligns rotation axis with z
				aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
				self.joint_transforms.append(aligned_joint_T)
				index = joint_values.name.index(current_joint.name)
				angle = joint_values.position[index]
				joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                 numpy.asarray(current_joint.axis))
				next_link_T = numpy.dot(current_joint_T, joint_rot_T) 
			else:
				next_link_T = current_joint_T

			self.process_link_recursive(next_link, next_link_T, joint_values)

if __name__ == '__main__':
	rospy.init_node('cartesian_control', anonymous=True)
	cc = CartesianControl()
	rospy.spin()

#!/usr/bin/env python  
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

def transformed(rot,tras,orig):
	
	return numpy.dot(rot,orig) + tras

def normalize(v):
	norm=numpy.linalg.norm(v, ord=1)
	if norm==0:
		norm=numpy.finfo(v.dtype).eps
	return v/norm
	
def get_translation(matrix):
	
	n = numpy.zeros(3)
	
	n.itemset(0,matrix.item(3))
	n.itemset(1,matrix.item(7))
	n.itemset(2,matrix.item(11))
	
	
	return n
	
def get_translation1(matrix):
	n = numpy.ndarray((4,4),dtype=float)
	n.fill(0)
	n.itemset(0,1)
	n.itemset(5,1)
	n.itemset(10,1)
	n.itemset(3,matrix.item(3))
	n.itemset(7,matrix.item(7))
	n.itemset(11,matrix.item(11))
	n.itemset(15,1)
	
	return n
	
def get_rotation(matrix):
	n = numpy.ndarray((3,3),dtype=float)
	n.fill(0)
	n.itemset(0,matrix.item(0))
	n.itemset(1,matrix.item(1))
	n.itemset(2,matrix.item(2))
	n.itemset(3,matrix.item(4))
	n.itemset(4,matrix.item(5))
	n.itemset(5,matrix.item(6))
	n.itemset(6,matrix.item(8))
	n.itemset(7,matrix.item(9))
	n.itemset(8,matrix.item(10))
	
	return n

def get_rotation1(matrix):
	n = numpy.ndarray((4,4),dtype=float)
	n.fill(0)
	n.itemset(0,matrix.item(0))
	n.itemset(1,matrix.item(1))
	n.itemset(2,matrix.item(2))
	
	n.itemset(4,matrix.item(4))
	n.itemset(5,matrix.item(5))
	n.itemset(6,matrix.item(6))
	
	n.itemset(8,matrix.item(8))
	n.itemset(9,matrix.item(9))
	n.itemset(10,matrix.item(10))
	n.itemset(15,1)
	
	return n

def DHMatrix(d,omega,r,alpha):
	n = numpy.ndarray(shape=(4,4), dtype=float)
	n.fill(0)
	
	n.itemset(0,numpy.cos(omega))
	
	n.itemset(1,numpy.sin(omega)*numpy.cos(alpha)*(-1))
	
	n.itemset(2,numpy.sin(omega)*numpy.sin(alpha))
	
	n.itemset(3,numpy.cos(omega)*r)
	
	n.itemset(4,numpy.sin(omega))
	
	n.itemset(5,numpy.cos(omega)*numpy.cos(alpha))
	
	n.itemset(6,numpy.cos(omega)*numpy.sin(alpha)*(-1))
	
	n.itemset(7,numpy.sin(omega)*r)
	
	n.itemset(9,numpy.sin(alpha))
	
	n.itemset(10,numpy.cos(alpha))
	n.itemset(11,d)
	n.itemset(15,1)
	
	return n
		
def message_from_transform(T):
	
	
	
	msg = geometry_msgs.msg.Transform()
	q = tf.transformations.quaternion_from_matrix(T)
	
	translation = tf.transformations.translation_from_matrix(T)
	
	msg.translation.x = translation[0]
	msg.translation.y = translation[1]
	msg.translation.z = translation[2]
	
	msg.rotation.x = q[0]
	msg.rotation.y = q[1]
	msg.rotation.z = q[2]
	msg.rotation.w = q[3]
	
	return msg
	

def publish_transforms():
	
	origin = [0,0,0,1]
	origin2 = [0,0,0]
	
	euler = tf.transformations.quaternion_from_euler(0.79,0.0,0.79)
	
	ROobject = tf.transformations.quaternion_matrix(euler)
	
	TRobject = tf.transformations.translation_matrix((0.0,1.0,1.0))
	
	Tobject = tf.transformations.concatenate_matrices(
		
		ROobject,
		TRobject
		)
	

	"""pepe = tf.transformations.concatenate_matrices(Tobject,origin)
	print pepe
	rot = get_rotation(Tobject)
	print transformed(rot,get_translation(Tobject),origin2)"""
	

	TobjectINV = tf.transformations.inverse_matrix(Tobject)
	
	"print tf.transformations.concatenate_matrices(TobjectINV,origin)"
	
	T1_stamped = geometry_msgs.msg.TransformStamped()
	T1_stamped.header.stamp = rospy.Time.now()
	T1_stamped.header.frame_id = "base_frame"
	T1_stamped.child_frame_id = "object_frame"
	T1_stamped.transform = message_from_transform(Tobject)
	br.sendTransform(T1_stamped)
	
	
	
	about_axis = tf.transformations.quaternion_about_axis(1.5, (0,0,1))
	
	ROrobot = tf.transformations.quaternion_matrix(about_axis)
	
	TRrobot = tf.transformations.translation_matrix((0.0,-1.0,0.0))
	
	Trobot = tf.transformations.concatenate_matrices(
		
		ROrobot,
		TRrobot
		)
	
	
	TrobotRot = get_rotation1(Trobot)
	
	TrobotRot2 = tf.transformations.inverse_matrix(TrobotRot)
	TrobotINV = tf.transformations.inverse_matrix(Trobot)
	
	TrobotINV1 = get_translation1(Trobot)
	TrobotINV2 = tf.transformations.inverse_matrix(TrobotINV1)
	
	"""print "s"
	print Trobot
	print TrobotINV"""
	
	
	
	T2_stamped = geometry_msgs.msg.TransformStamped()
	T2_stamped.header.stamp = rospy.Time.now()
	T2_stamped.header.frame_id = "base_frame"
	T2_stamped.child_frame_id = "robot_frame"
	T2_stamped.transform = message_from_transform(Trobot)
	br.sendTransform(T2_stamped)
	
	
	
	TRcamera = tf.transformations.translation_matrix((0.0,0.1,0.1))
	TRcameraINV = tf.transformations.inverse_matrix(TRcamera)
	
	
	
	
	

	
	Tpepe = tf.transformations.concatenate_matrices(TRcameraINV,TrobotINV)
	Tjuan = tf.transformations.concatenate_matrices(Tpepe,Tobject)
	
	vect = tf.transformations.concatenate_matrices(Tjuan,origin)
	vect1 = numpy.zeros(3)
	
	vect1.itemset(0,vect.item(0))
	vect1.itemset(1,vect.item(1))
	vect1.itemset(2,vect.item(2))
	
	
	ort = numpy.cross([1,0,0],vect1)
	pp = numpy.pi/2
	
	scalar_product = numpy.dot([1,0,0],vect1)
	
	norm = numpy.linalg.norm(vect1)
	
	cosinus = scalar_product/norm
	
	angle = numpy.arccos(cosinus)
	
	about_axis = tf.transformations.quaternion_about_axis(1.573, ort)

	ROcamera = tf.transformations.quaternion_matrix(about_axis )

	TCameraFinal = tf.transformations.concatenate_matrices(TRcamera,ROcamera)
	
	T3_stamped = geometry_msgs.msg.TransformStamped()
	T3_stamped.header.stamp = rospy.Time.now()
	T3_stamped.header.frame_id = "robot_frame"
	T3_stamped.child_frame_id = "camera_frame"
	T3_stamped.transform = message_from_transform(TCameraFinal)
	br.sendTransform(T3_stamped)
	

	"""Tkk = tf.transformations.inverse_matrix(TRcamera)
	
	Tkk1 = tf.transformations.inverse_matrix(get_rotation1(Trobot))
	
	Tkk2 = tf.transformations.concatenate_matrices(Tkk1,Tkk)"""

	

	"print Tob1"

	"""Tpepe = tf.transformations.concatenate_matrices(TRcameraINV,TrobotINV)
	Tjuan = tf.transformations.concatenate_matrices(Tpepe,Tobject)
	
	vect = tf.transformations.concatenate_matrices(Tjuan,origin)
	
	T4_stamped = geometry_msgs.msg.TransformStamped()
	T4_stamped.header.stamp = rospy.Time.now()
	T4_stamped.header.frame_id = "camera_frame"
	T4_stamped.child_frame_id = "pepe_frame"
	T4_stamped.transform = message_from_transform(Tjuan)
	br.sendTransform(T4_stamped)"""


if __name__ == '__main__':
	rospy.init_node('project2_solution')

	br = tf2_ros.TransformBroadcaster()
	rospy.sleep(0.5)

	while not rospy.is_shutdown():
		publish_transforms()
		rospy.sleep(0.05)

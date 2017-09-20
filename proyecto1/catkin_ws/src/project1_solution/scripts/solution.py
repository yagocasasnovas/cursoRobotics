#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts


pub = rospy.Publisher('sum', Int16, queue_size=10)

def talker(p,a,b):
	
	#rospy.init_node('talker', anonymous=True)
	#rate = rospy.Rate(10) # 10hz
	#while not rospy.is_shutdown():
	sum_answer = a + b
	rospy.loginfo("Sum: %s", sum_answer)
	#print str(a) + " " + str(b) + " " + str(sum_answer)
	pub.publish(sum_answer)
	#rate.sleep()


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "A: %s", data.a)
	rospy.loginfo(rospy.get_caller_id() + "B: %s", data.b)

	talker(pub,data.a, data.b)
	
def listener():


    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("two_ints", TwoInts, callback)


    rospy.spin()

if __name__ == '__main__':
	listener()
	
	

		
		
		
		
		
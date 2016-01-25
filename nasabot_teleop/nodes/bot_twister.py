#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int16


#a simple function to map a value from an input range to and output range:
	# x: the value to be mapped
	#in_min  in_max: the input range
	#out_min  out_max: the desrired output range
	
def map(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	
def constrain(x , low, high):
	return min(max(x,low),high)
	
# the listener class is the heart of this code
def callback_twist(msg):
	if msg.linear.x == 0: 
		throttle = 0
	else:
		throttle = abs(msg.linear.x)
		throttle = constrain(throttle,0,twist_range_high)
		throttle = map(throttle,0,twist_range_high,min_speed,max_speed)
		if msg.linear.x < 0:
			throttle = throttle * -1
		
	
	steering = constrain(msg.angular.z,twist_range_low,twist_range_high)
	steering = map(steering,twist_range_low,twist_range_high,0,180)
	
	pub_throttle.publish(throttle)
	pub_steering.publish(steering)
        
if __name__ == '__main__':
	# in ROS, nodes are unique named. If two nodes with the same
	# node are launched, the previous one is kicked off. The 
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'talker' node so that multiple talkers can
	# run simultaenously.
	rospy.init_node('bot_twister', anonymous=True)
	ns = rospy.get_namespace()
	twist_range_low = rospy.get_param('twist_range_low',-1)
	twist_range_high = rospy.get_param('twist_range_high',1)
	
	min_speed = rospy.get_param('~min_speed_out',100)
	max_speed = rospy.get_param('~max_speed_out',255)


	pub_throttle = rospy.Publisher('%sthrottle'%ns, Int16, queue_size=10)
	pub_steering = rospy.Publisher('%ssteering'%ns, Int16, queue_size=10)
	rospy.Subscriber('%stwist'%ns, Twist, callback_twist)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

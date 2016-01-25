#!/usr/bin/env python

from nasabot_teleop.srv import *
import rospy
import sys
from sensor_msgs.msg import CompressedImage

_type = CompressedImage

_input = 'input'
_output = 'output'

publisher = rospy.Publisher(_output, _type, queue_size=10)
	
def handle_add_two_ints(req):
    print "Returning [%s] in under %s"%(req.a,req.b)
    try:
    	msg = rospy.wait_for_message(_input, _type, timeout=req.b)
    	publisher.publish(msg)
    except(rospy.ROSException),e:
    	rospy.logwarn("Topic %s not available..."%_input)
    
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    
    #subscriber = rospy.Subscriber(_input, _type, queue_size=1)
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to publish an int."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()

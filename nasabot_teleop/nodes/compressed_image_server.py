#!/usr/bin/env python

from nasabot_teleop.srv import *
import rospy
import sys
from sensor_msgs.msg import CompressedImage

_type = CompressedImage

_input = 'input'
_output = 'output'

publisher = rospy.Publisher(_output, _type, queue_size=10)
	
def handle_request_compressedimage(req):
    print "Returning [%s] in under %s"%(req.topic,req.timeout)
    success=False
    try:
    	msg = rospy.wait_for_message(req.topic, _type, timeout=req.timeout)
    	publisher.publish(msg)
    	success = True
    except(rospy.ROSException),e:
    	rospy.logwarn("Topic %s not available..."%_input)
    
    return RequestCompressedImageResponse(success)

def request_compressedimage_server():
    rospy.init_node('add_two_ints_server')
    
    #subscriber = rospy.Subscriber(_input, _type, queue_size=1)
    s = rospy.Service('request_compressed_image', RequestCompressedImage, handle_request_compressedimage)
    print "Ready to publish an image."
    rospy.spin()

if __name__ == "__main__":
    request_compressedimage_server()

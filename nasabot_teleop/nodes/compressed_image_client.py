#!/usr/bin/env python

import sys
import rospy
from nasabot_teleop.srv import *

def request_compressedimage_client(topic, timeout):
    rospy.wait_for_service('request_compressed_image')
    try:
        request_compressedimage = rospy.ServiceProxy('request_compressed_image', RequestCompressedImage)
        resp1 = request_compressedimage(topic, timeout)
        return resp1.available
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        topic = sys.argv[1]
        timeout = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s in %s seconds"%(topic, timeout)
    print "%s"%request_compressedimage_client(topic, timeout)

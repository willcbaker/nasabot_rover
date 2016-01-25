#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray

import sys, select, termios, tty

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

webcam = [50,50]
kinect = [50,50]

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }
controlBindings={
	#left - 123 right - 124 down - 125 up - 126
	#left - 100 right - 102 down - 98 up - 104
        '0':'webcam',
        '1':'kinect',
        }
cameraBindings={
        '2':(0,-1),
        '4':(-1,0),
        '6':(1,0),
        '8':(0,1),
        }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1
move = 2

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('nasabot_teleop_joy')
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
    pub_webcam = rospy.Publisher('~webcam/control', UInt16MultiArray, queue_size=5)
    pub_kinect = rospy.Publisher('~kinect/control', UInt16MultiArray, queue_size=5)
    camera_control = 'webcam'
    camera_control_vals = {'webcam':[50,50],'kinect':[50,50]}
    pub_control={'webcam':pub_webcam,'kinect':pub_kinect}

    x = 0
    th = 0
    mv = [0,0]
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    target_move = [0,0]
    control_speed = 0
    control_turn = 0
    control_move = [0,0]
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                mv = [0,0]
                control_speed = 0
                control_turn = 0
                control_move = [0,0]
            elif key in cameraBindings.keys():
            	mv = cameraBindings[key]            
		for j in range(len(target_move)):
		    target_move[j] = move * mv[j]
		    if target_move[j] > control_move[j]:
			control_move[j] = min( target_move[j], control_move[j] + 1 )
		    elif target_move[j] < control_move[j]:
			control_move[j] = max( target_move[j], control_move[j] - 1 )
		    else:
			control_move[j] = target_move[j]
		    current = camera_control_vals[camera_control][j]
		    current +=control_move[j]
		    current = min(current,100)
		    current = max(current,0)
		    camera_control_vals[camera_control][j]=current

		if target_speed > control_speed:
			control_speed = min( target_speed, control_speed + 0.02 )
		elif target_speed < control_speed:
			control_speed = max( target_speed, control_speed - 0.02 )
		else:
			control_speed = target_speed

		if target_turn > control_turn:
			control_turn = min( target_turn, control_turn + 0.1 )
		elif target_turn < control_turn:
			control_turn = max( target_turn, control_turn - 0.1 )
		else:
			control_turn = target_turn
		array = UInt16MultiArray()
		array.data = camera_control_vals[camera_control]
		pub_control[camera_control].publish(array)
		    
            elif key in controlBindings.keys():
            	if key == '0':
            		camera_control = 'webcam'
            	if key == '1':
            		camera_control = 'kinect'
            	print camera_control
            		
            else:
            	control_move = [0,0]
            	mv = [0,0]
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break
	    print x,th
            target_speed = speed * x
            target_turn = turn * th
            
            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)
		
            #print camera_control,':',camera_control_vals[camera_control]
            #control_cam[0] = control_move
            #pub_webcam.publish(webcam)
            #pub_kinect.publish(kinect)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))
    except TypeError as e:
    	print e
    except:
        print sys.exc_info()

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


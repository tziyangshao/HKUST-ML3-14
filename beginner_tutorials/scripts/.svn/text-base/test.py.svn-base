#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import subprocess
##testing
import struct
import binascii
from beginner_tutorials.msg import keyframeMsg
from itertools import count
import sim3
##from IPython.core.debugger import Tracer

##from ctypes import *
##debug_here = Tracer()

pointcloud = None

##class POINT(Structure):
##    _fields_ = [("idepth", c_float),
##		("idepth_var", c_float),
##		("color", c_ubyte * 4 )]
global filename
filename = ("data/pointcloud_%05i" % i for i in count(1))

def callback(data):
    filename_buffer=filename
    fx=data.fx
    fy=data.fy
    cx=data.cx
    cy=data.cy

    sim3.Sophus.Sim3f.camToWorld=data.camToWorld

    pc_width=data.width
    pc_height=data.height
    time=data.time
    pointcloud=data.pointcloud
 ##  pointcloud=POINT(data.pointcloud)

##    debug_here()

    fxi=1/fx
    fyi=1/fy
    cxi=-cx/fx
    cyi=-cy/fy
    
    with open(next(filename),"w") as datafile:
        datafile.write("%f\n%f\n%f\n%f\n%f\n%f\n%f\n"%(camToWorld[0],camToWorld[1],camToWorld[2],camToWorld[3],camToWorld[4],camToWorld[5],camToWorld[6]))
	datafile.write("%f\n%f\n%f\n%f\n"%(fxi,fyi,cxi,cyi))
	for width in range(640):
	    for height in range(480):
	        parse=[]
	        for n in range(4):
		    parse.append(pointcloud[n+height*12+width*480])
		depth = struct.unpack("f","".join(parse))
		if depth[0]>0:	
		    datafile.write("%i\n%i\n"%(width,height))
		    datafile.write("%f\n"%(1/depth[0]))
##Sophus::Vector3f pt = scamToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1) * depth);
	cameraProperty = sim3.Sophus.Vector3f((x*fxi + cxi), (y*fyi + cyi),1)
	pt=sim3.Sophus.Vector3f(0,0,0)
	pt=camToworld*(cameraProperty/depth)
	print(type(camToWorld))
	print(len(camToWorld))
	##working	
	##for width in range(640):
	    ##for height in range(480):
		##for a in range(2):
		    ##parse=[]
		    ##for n in range(4):
			##parse.append(pointcloud[n+a*4+height*2*4+width*2*4*480])
	    	    ##datafile.write("\t%f:\n"%(struct.unpack("f","".join(parse))))	
    
	    ##datafile.write("\t%f\n"%(struct.unpack('f',str.join(parser))))
	##for width in range(640):
	##    for height in range(480):
	##	datafile.write("%r\t"%(pointcloud[1]))
	##	datafile.write("%s\t"%(inputpointdense.unpack(pointcloud[pc_width*width+height])))	
    rospy.loginfo("data appending \n")
    

def listener():

    # In ROS, nodes are uniquely named. If swo nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/lsd_slam/keyframes", keyframeMsg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # subprocess.call("rm pointcloud",shell=True)
    listener()

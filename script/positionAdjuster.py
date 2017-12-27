#!/usr/bin/env python


#Copyright (c) 2017 Ryoichi Ishikawa. All rights reserved.
#
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php


import rospy
import socket
import struct
import sys
import tf
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import *
import math

global adjuster

def adjusterCB(msg):
	global adjuster
	adjuster=msg.data
	

if __name__ == '__main__':
	rospy.init_node('init_pos_pub')
	listener=tf.TransformListener()
	adjuster=True

	if len(sys.argv)<2:
		print "usage:positionAdjuster.py <robot's foot flame name>"
		exit()
		
	footprintFrame="/" + sys.argv[1]
	print "footprintframe: " + footprintFrame
	hololensPos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,queue_size=1)
	sub=rospy.Subscriber('/holo/adjuster',Bool,adjusterCB)
	rot90=np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
	rate = rospy.Rate(10.0)
	br = tf.TransformBroadcaster()
		
	last_update=rospy.get_time()
	firstloop=True
	while not rospy.is_shutdown():
		try:
			if listener.frameExists("/map") and listener.frameExists("/hololens"):
				t = listener.getLatestCommonTime('/map', '/hololens')
				(trans,rot) = listener.lookupTransform('/map', '/hololens', t)
			else:
				continue
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,tf.Exception):
                        print "tf error /map /hololens"
			continue
		try:
			(trans2,rot2) = listener.lookupTransform('/hololens_p', footprintFrame, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,tf.Exception):
                        print "tf error hololens_p footprintframe"
			continue
		
		#check pepper Head position
		#try:
		#	(trans3,rot3) = listener.lookupTransform('/hololens_p', "/Head", rospy.Time(0))
		#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,tf.Exception):
                #        print "tf error hololens_p footprintframe"
	#		continue
		
		
		if not firstloop:
			if last_t==t:
				continue			
			d=(trans[0]-last_trans[0])*(trans[0]-last_trans[0])+(trans[1]-last_trans[1])*(trans[1]-last_trans[1])
			now = rospy.get_time()
			#jump check caused by switching spatial anchor
			if d>1.0 and now-last_update<2.0:
				print "position jumped (ddist , dtime) (" + str(d) + "," + str(now-last_update)
				continue
		#d=(trans[0]-trans2[0])*(trans[0]-trans2[0])+(trans[1]-trans2[1])*(trans[1]-trans2[1])
		#if d<0.01:
		#  rate.sleep()
		#  continue

		#map 2 hololens_p
		map2holo = np.dot(tf.transformations.translation_matrix(trans),tf.transformations.quaternion_matrix(rot))
		holo2foot = np.dot(tf.transformations.translation_matrix(trans2),tf.transformations.quaternion_matrix(rot2))    

		map2foot=np.dot(map2holo,holo2foot)
		map2foot_beforeAdjust=map2foot
		#Head position check
		#holo2head = np.dot(tf.transformations.translation_matrix(trans3),tf.transformations.quaternion_matrix(rot3))    
		#map2head=np.dot(map2holo,holo2head)

		if adjuster:
			foot2holo=np.linalg.inv(holo2foot)

			#calc adjust rotation matrix 
			foot2map=np.linalg.inv(map2foot)
			rz=np.array([foot2map[0][2],foot2map[1][2],foot2map[2][2]])

			#print "init_map2foot-angle-axis"
			#print map2foot
			to_z=np.array([0,0,1.0])
			axis=np.cross(rz,to_z)
			angle=math.acos(np.dot(rz,to_z))
			#print angle
			#print axis
			adjustMatrixA=tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(angle,axis))
			adjustMatrixA=np.linalg.inv(adjustMatrixA)
			#debug
			debugMat=np.dot(map2foot,adjustMatrixA)
			#print "map2foot-adjustA"
			#print adjustMatrixA
			#print debugMat
			#
			adjustMatrixB=np.dot(np.dot(holo2foot,adjustMatrixA),foot2holo)
			adjustMatrixB[0][3]=0
			adjustMatrixB[1][3]=0
			adjustMatrixB[2][3]=0
			#print adjustMatrixB
			#print adjustMatrixA
			#print adjustMatrixB
			map2foot=np.dot(np.dot(map2holo,adjustMatrixB),holo2foot)
		
		brtrans=(map2foot[0][3], map2foot[1][3], map2foot[2][3])
		brrot=	tf.transformations.quaternion_from_matrix(map2foot)		
		br.sendTransform(brtrans,
			brrot,
			rospy.Time.now(),
			"/localized_footprint",
			"/map")
		
		brtrans=(map2foot_beforeAdjust[0][3], map2foot_beforeAdjust[1][3], map2foot_beforeAdjust[2][3])
		brrot=	tf.transformations.quaternion_from_matrix(map2foot_beforeAdjust)		
		br.sendTransform(brtrans,
			brrot,
			rospy.Time.now(),
			"/localized_footprint_nAdj",
			"/map")
		
		#brtrans=(map2head[0][3], map2head[1][3], map2head[2][3])
		#brrot=	tf.transformations.quaternion_from_matrix(map2head)		
		#br.sendTransform(brtrans,
	#		brrot,
		#	rospy.Time.now(),
		#	"/localized_neck",
		#	"/map")
		
		#print "adjusted"
		#print map2foot
		cmd = PoseWithCovarianceStamped()
		cmd.pose.pose.position.x=map2foot[0][3]
		cmd.pose.pose.position.y=map2foot[1][3]
		cmd.pose.pose.position.z=0#map2foot[2][3]    
		q=tf.transformations.quaternion_from_matrix(map2foot)
		invq=tf.transformations.quaternion_inverse(q)

		dirq=  np.zeros((4, ), dtype=np.float64)
		dirq[0]=1  
		q1=tf.transformations.quaternion_multiply(dirq,invq)
		q2=tf.transformations.quaternion_multiply(q,q1)

		qz=q2[0]
		qw=q2[1]
		rad=math.sqrt(qz*qz+qw*qw)
		qz=qz/rad;
		qw=qw/rad;

		theta=math.acos(qz);
		if qw<0:
			theta=-theta;

		cmd.pose.pose.orientation.x=0
		cmd.pose.pose.orientation.y=0
		cmd.pose.pose.orientation.z=math.sin(theta/2)
		cmd.pose.pose.orientation.w=math.cos(theta/2)
		hololensPos.publish(cmd)
		last_trans=trans
		last_update=rospy.get_time()
		firstloop=False
		last_t=t
		rate.sleep()

#!/ usr/bin/env python
import roslib
import rospy
import tf
import os

def cls () :
    os.system('clear')
import numpy as np

if __name__ == '__main__':
    rospy.init_node('kinect_listener', anonymous = True )
    listener = tf.TransformListener()
    rate = rospy.Rate(30.0) 
    
    while not rospy.is_shutdown() : 
        try: 
			cls()
			(trans,rot) = listener.lookupTransform('/ torso_1 ','/ openni_link ', rospy.Time(0) )
        
			rospy.loginfo(" Torso coordinates :\n\n")
			rospy.loginfo("\t\tX = {0:f}".format(trans[0]))
			rospy.loginfo("\t\tY = {0:f}".format(trans[1]))
			rospy.loginfo("\t\tZ = {0:f}".format(trans[2]))
		except (tf.LookupException , tf.ConnectivityException , tf.ExtrapolationException ):
			continue
    
    rate.sleep ()
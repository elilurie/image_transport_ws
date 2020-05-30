#!/usr/bin/env python
import numpy as np
import cv2
import sys
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import findlines_algo
sys.path.append(os.path.abspath('./src/image_transport_tutorial/scripts/common'));
import myerror

Probabilistic = False
is_camera=False


class ROSFindLines():
    def __init__(self):
        self.classname='ROSFindLines'
        fn='{}::__init_()'.format(self.classname)
        try:
            self.bridge = CvBridge()
            self.algo=findlines_algo.AlgoFindLines(isfromros=True)
            #
            # Declare node
            #
            self.init_node()
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err)            
            rospy.logerr(msg)
            raise ValueError(msg);
    ####################
    # callback
    ####################
    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        try:
            #rospy.loginfo(rospy.get_caller_id() + "I heard image: {}".format(self.count))
            
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")

            #
            # Find lines and publish
            #
            imgout=self.algo.find_line(img)
            imgmsg = self.bridge.cv2_to_imgmsg(imgout, "bgr8")
            self.pub.publish(imgmsg)
 
            
            #cv.imwrite("{}/frame-{}.png".format('.', self.count), cv_image)
            self.count+=1
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err)            
            rospy.logerr(msg)
            raise ValueError(msg);
    ####################
    # init_node
    ####################
    def init_node(self):
        fn='{}::init_node()'.format(self.classname)
        try:
            rospy.loginfo("{}: BR init_node()".format(fn))
            rospy.init_node('findlines', anonymous=True)
            #
            # Reroute the log
            #
            rospy.loginfo("{}: AR init_node()".format(fn))
            self.count=0;
            rospy.loginfo('{}: BR Subscriber to /camera/image'.format(fn))
            #
            # Subscriber
            #
            rospy.Subscriber("camera/image", Image, self.callback)

            rospy.loginfo('{}: BR Publisher of /findlines/image'.format(fn))
            self.pub = rospy.Publisher('findlines/image', Image, queue_size=10)

            # spin() simply keeps python from exiting until this node is stopped
            rospy.loginfo('{}: BR spin'.format(fn))
            rospy.spin()
 
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err)            
            rospy.logerr(msg)
            raise ValueError(msg);
    
if __name__ == '__main__':
    try:
        obj=ROSFindLines()
    except rospy.ROSInterruptException:
        pass
 




#!/usr/bin/env python
import numpy as np
import cv2
import sys
import os
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from cv_bridge import CvBridge, CvBridgeError
import detectobj_algo
sys.path.append(os.path.abspath('./src/image_transport_tutorial/scripts/common'));
import myerror

is_camera=False


class ROSDetectObj():
    def __init__(self):
        self.classname='ROSDetectObj'
        fn='{}::__init_()'.format(self.classname)
        try:
            self.bridge = CvBridge()
            self.algo=detectobj_algo.AlgoDetectObj(isfromros=True)
            self.pubtopic='detectobj/image'
            self.cmd='PAUSEPUB'
            #
            # Declare node
            #
            self.init_node()
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err)            
            rospy.logerr(msg)
            raise ValueError(msg);
    ####################
    # callback_newtopic
    #  1. New message on /rpiwebserver/newtopic/ was recieved 
    #       It is a command to mjpeg subscriber to switch a video topic 
    #       (For ex: unsubscribe from /findlines/image and subscribe to /findlines/image 
    #       It is sent from rossdk.js 
    #       This means that if findlines is not requested anymore we can pause publish it
    ####################
    def callback_newtopic(self, data):
        fn='{}::callback_newtopic()'.format(self.classname)
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        try:
            rospy.loginfo('{}: get_caller_id:{} heard:{}'\
                    .format(fn, rospy.get_caller_id(), data.data))
            pubtopic=data.data
            if pubtopic != self.pubtopic:
                self.cmd='PAUSEPUB'
                rospy.loginfo('{}: PAUSEPUB'.format(fn))
            else:
                self.cmd='RESUMEPUB'
                rospy.loginfo('{}: RESUMEPUB'.format(fn))
 
            
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err)            
            rospy.logerr(msg)
            raise ValueError(msg);
 
    ####################
    # callback
    ####################
    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        fn='{}::callback()'.format(self.classname)

        try:
            #rospy.loginfo(rospy.get_caller_id() + "I heard image: {}".format(self.count))
            
            if self.cmd=='RESUMEPUB':
                img = self.bridge.imgmsg_to_cv2(data, "bgr8")

                #
                # Find lines and publish
                #
                imgout=self.algo.detect_objects(img)
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
            rospy.init_node('detectobj', anonymous=True)
            #
            # Reroute the log
            #
            rospy.loginfo("{}: AR init_node()".format(fn))
            self.count=0;
            rospy.loginfo('{}: BR Subscriber to /camera/image'.format(fn))
            #
            # Subscriber
            #  queu_size=1 and buff_size=2**24 are necessary for retrieve only latest message(image)
            #
            #
            rospy.Subscriber("camera/image", Image, self.callback, queue_size=1, buff_size=2**24)

            rospy.loginfo('{}: BR Publisher of topic: {}'.format(fn, self.pubtopic))
            self.pub = rospy.Publisher(self.pubtopic, Image, queue_size=1)
            #
            # Subscribe also to a topic that switches video input to MJPEG
            # If the message on /rpiwebserver/newtopic/ request video NOT from /findlines/image
            # then stop publishing the video
            #
            self.subnewtopic=rospy.Subscriber("rpiwebserver/newtopic", String, self.callback_newtopic)
 
            # spin() simply keeps python from exiting until this node is stopped
            rospy.loginfo('{}: BR spin'.format(fn))
            rospy.spin()
 
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err)            
            rospy.logerr(msg)
            raise ValueError(msg);
    
if __name__ == '__main__':
    try:
        obj=ROSDetectObj()
    except rospy.ROSInterruptException:
        pass
 




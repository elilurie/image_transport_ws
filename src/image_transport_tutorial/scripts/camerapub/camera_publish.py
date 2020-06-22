#!/usr/bin/env python
import sys
import os
import os.path
from os import path

import cv2 as cv
import argh
import rospy
import threading
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
sys.path.append(os.path.abspath('./src/image_transport_tutorial/scripts/common'));
import myerror
#############################
# class GrabFrameThread
#############################
class CameraGrabFrameThread():
    def __init__(self, fname):
        self.classname='CameraGrabFrameThread()'
        fn='{}::__init__()'.format(self.classname)
        try:
            if fname.isdigit():
                rospy.loginfo('{}: isdigit=True'.format(fn))
                devid=int(fname)
                self.devid=devid
                self.camera = cv.VideoCapture(devid)
            else:
                rospy.loginfo('{}: isdigit=False'.format(fn))
                if path.exists(fname)==False:
                    raise ValueError('{}: failed!!! {} does not exist'.format(fn, fname))
                self.camera = cv.VideoCapture(fname)

            self.frame=None
        except Exception as err:
            msg='{}: Failed!!! err:{}'.format(fn, err)
            rospy.logerr(msg)
            raise ValueError(msg)
    ###############
    # process
    ###############
    def process(self):
        fn='{}::process()'.format(self.classname)
        try:
            while not rospy.is_shutdown():
                ret, self.frame = self.camera.read()
                if ret==False:
                    msg='{}: Failed!!! camera read Failed!!! ret:{}'\
                            .format(fn, ret)
                    rospy.logerr('{}: failed!!! ret=False'.format(fn))
                    continue
                
                time.sleep(5.0 / 1000)
                #rospy.loginfo('{}: count:{}'.format(fn, self.count))
        except Exception as err:
            msg='{}: Failed!!! err:{}'.format(fn, err)
            rospy.logerr(msg)
            raise ValueError(msg)

#############################
# class CameraPublisher
#############################
class CameraPublish():
    def __init__(self, fname='0'):

        self.classname="CameraPublish"
        fn='{}::__init__()'.format(self.classname)
        try:
            self.count=0
            self.fname=None
            self.frate=30.0
            #self.frate=4.0
            self.start_node()
            rospy.loginfo('{}: fname:{}'.format(fn, fname))
            self.videothread=None            
            self.videothread=CameraGrabFrameThread(fname)
            thread = threading.Thread(target=self.videothread.process);
            thread.start();

            rospy.loginfo('{}: AR create VideoCapture'.format(fn))
            self.bridge = CvBridge()
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err);
            rospy.logerr(msg)
            raise ValueError(msg)
       
    #########################
    # start_node
    #########################    
    def start_node(self):
        fn='{}::start_node()'.format(self.classname)
        try:
            rospy.init_node('camerapub')
            rospy.loginfo('image_pub node started')
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err)            
            rospy.logerr(msg)
            raise ValueError(msg);
    ##########################
    # publish
    ##########################
    def publish(self):
        fn='{}::publish()'.format(self.classname)
        try:
            
            start_time = time.time()
            pub = rospy.Publisher('camera/image', Image, queue_size=1)
            while not rospy.is_shutdown():
                if self.videothread!=None:
                    ret=False
                    img=self.videothread.frame
                    if (type(img) is np.ndarray): 
                        ret=True                        

                else:                                
                    ret, img = self.camera.read()
                if ret==False:
                    msg='{}: Failed!!! camera read Failed!!! ret:{}'\
                            .format(fn, ret)
                    if time.time() - start_time > 5.0:
                        rospy.logerr('{}: failed!!! ret=False'.format(fn))
                    rospy.Rate(30).sleep()  # 30 Hz
                    continue
                #rospy.loginfo('{}: count:{}'.format(fn, self.count))
                self.count+=1
                img[0,0,0]=self.count
                imgmsg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                #rospy.loginfo('{}: img[0,0,0]:{}'.format(fn, img[0,0,0]))
                pub.publish(imgmsg)
                rospy.Rate(self.frate).sleep()  # 30 Hz
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err)            
            rospy.logerr(msg)
            raise ValueError(msg);
           
if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print("usage: camera_publish.py arg1 ")
            print("\targ1 - 0,1.. for /dev/video# or full path for video file")
        else:
            obj=CameraPublish(fname=sys.argv[1])
            obj.publish()
    except rospy.ROSInterruptException:
        pass
 


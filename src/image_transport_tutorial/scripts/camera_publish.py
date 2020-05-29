#!/usr/bin/env python
import cv2 as cv
import argh
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import myerror
#############################
# class CameraPublisher
#############################
class CameraPublish():
    def __init__(self):

        self.classname="CameraPublish"
        fn='{}::__init__()'.format(self.classname)
        try:
            self.count=0
            self.start_node()
            self.camera = cv.VideoCapture(0)
            rospy.loginfo('{}: AR create VideoCapture'.format(fn))
            self.bridge = CvBridge()
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err);
            rospy.logerror(msg)
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
            while not rospy.is_shutdown():
                ret, img = self.camera.read()
                if ret==False:
                    msg='{}: Failed!!! camera read Failed!!! ret:{}'\
                            .format(fn, ret)
                    rospy.logerr('{}: failed!!! ret=False'.format(fn))
                #rospy.loginfo('{}: count:{}'.format(fn, self.count))
                self.count+=1
                imgmsg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                pub = rospy.Publisher('camera/image', Image, queue_size=10)
                pub.publish(imgmsg)
                rospy.Rate(30.0).sleep()  # 30 Hz
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err)            
            rospy.logerr(msg)
            raise ValueError(msg);
           
if __name__ == '__main__':
    try:
        obj=CameraPublish()
        obj.publish()
    except rospy.ROSInterruptException:
        pass
 


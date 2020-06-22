#!/usr/bin/env python
import os
import sys
import cv2
import time
import argh
from argh import arg
import rospy
sys.path.append(os.path.abspath('./src/image_transport_tutorial/scripts/common'));
sys.path.append(os.path.abspath('../common'));
import mylog
import myerror

# Pretrained classes in the model
classNames = {0: 'background',
              1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus',
              7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light', 11: 'fire hydrant',
              13: 'stop sign', 14: 'parking meter', 15: 'bench', 16: 'bird', 17: 'cat',
              18: 'dog', 19: 'horse', 20: 'sheep', 21: 'cow', 22: 'elephant', 23: 'bear',
              24: 'zebra', 25: 'giraffe', 27: 'backpack', 28: 'umbrella', 31: 'handbag',
              32: 'tie', 33: 'suitcase', 34: 'frisbee', 35: 'skis', 36: 'snowboard',
              37: 'sports ball', 38: 'kite', 39: 'baseball bat', 40: 'baseball glove',
              41: 'skateboard', 42: 'surfboard', 43: 'tennis racket', 44: 'bottle',
              46: 'wine glass', 47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon',
              51: 'bowl', 52: 'banana', 53: 'apple', 54: 'sandwich', 55: 'orange',
              56: 'broccoli', 57: 'carrot', 58: 'hot dog', 59: 'pizza', 60: 'donut',
              61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant', 65: 'bed',
              67: 'dining table', 70: 'toilet', 72: 'tv', 73: 'laptop', 74: 'mouse',
              75: 'remote', 76: 'keyboard', 77: 'cell phone', 78: 'microwave', 79: 'oven',
              80: 'toaster', 81: 'sink', 82: 'refrigerator', 84: 'book', 85: 'clock',
              86: 'vase', 87: 'scissors', 88: 'teddy bear', 89: 'hair drier', 90: 'toothbrush'}

def id_class_name(class_id, classes):
    for key, value in classes.items():
        if class_id == key:
            return value
#########################
# Mylog to ROS
#########################
class MyLogForROS():
    def __init__(self):
        self.info=rospy.loginfo
        self.error=rospy.logerr
        self.warning=rospy.logwarn

############################
#
############################
class AlgoDetectObj():
    def __init__(self, log=None, isfromros=True):
        self.classname="AlgoDetectObj" 
        fn='{}::__init__()'.format(self.classname)
        self.log=log

        if isfromros:
            self.log=MyLogForROS()
        self.maximgwidth=800
        self.maximgheight=600
        self.realpath=os.path.dirname(os.path.realpath(__file__))
        self.log.info('{}: realpath:{}'.format(fn, self.realpath))
        self.model=self.load_model()

    ###############################
    # resize image
    ###############################
    def resize_image(self, image):
        image_height, image_width, _ = image.shape
        if image_height > self.maximgheight or image_width > self.maximgwidth: 
            f=float(image_width)/float(image_height)
            if image_height > image_width:
                dst_height=600    
                dst_width=int(dst_height*f);
            else:
                dst_width=800    
                dst_height=int(dst_width/f);
            img=cv2.resize(image, (dst_width, dst_height))
            return img
        return image

    ######################
    # load_model
    ######################
    def load_model(self):
        fn='{}::load_model()'.format(self.classname)
        try:
            modelfile='{}/models/frozen_inference_graph.pb'.format(self.realpath)
            modelcfg='{}/models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt'.format(self.realpath)

            model = cv2.dnn.readNetFromTensorflow(modelfile,
                                                    modelcfg)
            return model
        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err);
            print 'aaa'            
            self.log.error(msg)
            print 'bbb'            
            raise ValueError(msg)
    ############################
    #
    ############################
    def display_detected_objects(self, image):
        fn='display_detected_objects()'
        try:
            cv2.imshow('image', image)
            # cv2.imwrite("image_box_text.jpg",image)

            cv2.waitKey(0)
            cv2.destroyAllWindows()

        except Exception as err:
            msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(), err);
            self.log.error(msg)
            raise ValueError(msg)
       
    #####################
    # detect_objects
    #####################
    def detect_objects(self, image):
       
        image=self.resize_image(image)
        image_height, image_width, _ = image.shape
        blobsize=300

        self.model.setInput(cv2.dnn.blobFromImage(image, size=(300, 300), swapRB=True))
        fx=float(image_width)/blobsize
        fy=float(image_height)/blobsize
        output = self.model.forward()
        # print(output[0,0,:,:].shape)

        count=0
        for detection in output[0, 0, :, :]:
            confidence = detection[2]
            if confidence > .5:
                class_id = detection[1]
                class_name=id_class_name(class_id,classNames)
                #print(str(str(class_id) + " " + str(detection[2])  + " " + class_name))
                box_x = detection[3] * blobsize * fx
                box_y = detection[4] * blobsize * fy
                #box_x = detection[3] * image_width
                #box_y = detection[4] * image_height
                #box_width = detection[5] * image_width
                #box_height = detection[6] * image_height
                box_width = detection[5] * blobsize*fx
                box_height = detection[6] * blobsize*fy
                 #cv2.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (23, 230, 210), thickness=1)
                cv2.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (23, 230, 210), thickness=1)
                #cv2.putText(image,class_name ,(int(box_x), int(box_y+.05*image_height)),cv2.FONT_HERSHEY_SIMPLEX,(.005*image_width),(0, 0, 255))

                cv2.putText(image,class_name,
                                #(int(box_x), int(box_y+.05*image_height)),
                                (int(box_x), int(box_y)+30),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                #(.005*image_width),(0, 0, 255))
                                (1.0),(0, 0, 255))

        return image



####################################
# PROCEDURES
####################################
#######################################
# detect
#   usage: ./main.py detect FNAME
#######################################
@arg('--fname', default='0', help='File name (0, 1, 2... in case of camera)' )
@arg('--ftype', default=2, help='0-frame file, 1-video file, 2-camera' )
#@arg('-d', '--daysback', default=90, help='daysback' )
def detect(fname=0, ftype=2):
    '''Detect objects'''
    fn='detect()'
    log = mylog.configure_logging('detectobj', "/tmp/detectobj.log")
    try:
        log.info('{}: ##############################'.format(fn))
        log.info('{}: fname:{} ftype:{}'\
                .format(fn,fname, ftype))
        log.info('{}: ##############################'.format(fn))
        log.info('{}: __file__:{}'.format(fn,__file__));
        if ftype==0:
            image = cv2.imread(fname)
        elif ftype==1:
            camera = cv2.VideoCapture(fname)
        elif ftype==2:
            camera = cv2.VideoCapture(int(fname))
        counter=0
        algoobj=AlgoDetectObj(log=log, isfromros=False)
        #model=algoobj.load_model()
        start_time = time.time()
        if ftype==0:
            #for i in range(0,100):
            for i in range(0,1):
                image=algoobj.detect_objects(image)
                counter+=1
        elif ftype==1:
            while 1:
                ret, img = camera.read()
                if ret:
                    image=algoobj.detect_objects(img)
                    counter+=1
                    # Display the resulting frame
                    cv2.imshow('Frame',image)
                    # Press Q on keyboard to  exit
                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        break
                else:
                    log.info('{}: ret=False=>break'.format(fn))
                    break;
        elif ftype==2:
            while True:
                ret, img = camera.read()
                if ret:
                    image=algoobj.detect_objects(img)
                    counter+=1
                    # Display the resulting frame
                    cv2.imshow('Frame',image)
                    # Press Q on keyboard to  exit
                    if time.time() - start_time > 10:
                        log.info("{}: FPS:{}".format(fn, counter / (time.time() - start_time)))
                        start_time=time.time()
                        counter=0;
                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        break
                else:
                    log.info('{}: ret=False=>break'.format(fn))
                    break;


        log.info("{}: FPS:{}".format(fn, counter / (time.time() - start_time)))
        if ftype==0:
            display_detected_objects(log, image)
        

        log.info('{}: FINISH'.format(fn))

    except Exception as err:
        log.error("{}: err:{}".format(fn, err))
####################################
# MAIN
####################################
arg_parser = argh.ArghParser()
arg_parser.add_commands([
                            detect
                        ])

if __name__ == '__main__':
    arg_parser.dispatch()

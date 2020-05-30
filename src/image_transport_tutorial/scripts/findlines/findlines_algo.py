#!/usr/bin/env python
import sys
import os
import argh
import numpy as np
import cv2 as cv
import rospy
sys.path.append(os.path.abspath('./src/image_transport_tutorial/scripts/common'));
import mylog
import myerror

class MyLogForROS():
    def __init__(self):
        self.info=rospy.loginfo
        self.error=rospy.logerr
        self.warning=rospy.logwarn

class AlgoFindLines():
    def __init__(self, log=None, isfromros=True):
        self.classname="AlgoFindLines" 
        fn='{}::__init__()'.format(self.classname)
        self.log=log

        if isfromros:
            self.log=MyLogForROS()
        self.isprobabilistic=False

    def get_strong_lines(self, lines,rho_min_dist):

        """A function used to extract distinct lines from a HoughTransform lines list

        Attributes:
        ----------
        lines - a list of (rho, theta)
        rho_min_dist - the min dist between distinct lines

        Output:
        ------
        strong_lines - a structure including up to 4 distinct lines
        """

        # Init strong lines structure
        strong_lines = np.zeros([4,1,2])

        n2 = 0
        max_lines_to_analyze = 10
        N = min(max_lines_to_analyze,len(lines))

        for n1 in range(0,N):
            for rho,theta in lines[n1]:
                if n1 == 0:
                    strong_lines[n2] = lines[n1]
                    #print(lines[n1])
                    n2 = n2 + 1
                else:
                    # Handle singularity point
                    if rho < 0:
                       rho*=-1
                       theta-=np.pi

                    # Check if line is close to previous lines
                    closeness_rho = np.isclose(rho,strong_lines[0:n2,0,0],atol = rho_min_dist)
                    closeness_theta = np.isclose(theta,strong_lines[0:n2,0,1],atol = np.pi/36)
                    closeness = np.all([closeness_rho,closeness_theta],axis=0)
                    if not any(closeness) and n2 < 4:
                        strong_lines[n2] = lines[n1]
                        n2 = n2 + 1
                        #print(lines[n1])
        return strong_lines
    ##########################
    # find_line() 
    #   find line in the frame
    ##########################
    def find_line(self, frame):

        """A function used to find distinct lines in an images stream

        Attributes:
        ----------
        cap -
        rho_min_dist - the min dist between distinct lines

        Output:
        ------
        strong_lines - a structure including up to 4 distinct lines
        """

        # Normalize the input image
        height, width = frame.shape[:2]
        #print(width, height, 'width', 'height')
        normalizedFrame = np.zeros((height, width))
        normalizedFrame = cv.normalize(frame, normalizedFrame, 0, 255, cv.NORM_MINMAX)
        #cv.imshow('dst_rt', normalizedImg)

        # Translate from CV2 BGR to gray scale
        gray = cv.cvtColor(normalizedFrame, cv.COLOR_BGR2GRAY)
        # crop_img = gray[(height * 0.1:200, 0:-1]

        # Canny Edge Detection:
        C_Threshold1 = 20; # 200 # pixels with lower gradient are declared not-edge
        C_Threshold2 = 50; # 500 # pixels with higher gradient are declared edge
        edges = cv.Canny(gray, C_Threshold1, C_Threshold2, apertureSize=3)
        #cv.imshow('dst_rt', edges)

        # Hough Transform
        if self.isprobabilistic:
            # Probabilistic Hough Transform:
            lines = cv.HoughLinesP(edges, rho=1, theta=1 * np.pi / 400, threshold=200, minLineLength=200, maxLineGap=50)
            #lines = cv.HoughLinesP(edges, rho=1, theta=1 * np.pi / 180, threshold=100, minLineLength=100, maxLineGap=50)
            N = lines.shape[0]
            for i in range(N):
                x1 = lines[i][0][0]
                y1 = lines[i][0][1]
                x2 = lines[i][0][2]
                y2 = lines[i][0][3]
                cv.line(normalizedFrame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        else:
            # Regular Transform
            Rres = 1
            Thetares = 1 * np.pi / 400
            lines = cv.HoughLines(edges, Rres, Thetares, 300)

            if not(lines is None):
                N = min(len(lines),5)
                print('lines num =', N)

                strong_lines = self.get_strong_lines(lines,100)
                for my_lines in strong_lines:
                    rho = my_lines[0][0]
                    theta = my_lines[0][1]
                    if (rho==0) and (theta==0):
                        break;
                    else:
                        print(rho, theta)
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))

                        cv.line(normalizedFrame, (x1, y1), (x2, y2), (0, 0, 255), 2)

                '''
                for rho, theta in lines[0:N,0]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))
                    print(rho, theta)
                    
                    cv.line(normalizedFrame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    '''
        return normalizedFrame



#def main():
#    #cap = cv.VideoCapture('deck vid.mp4')
#    if is_camera:
#        cap = cv.VideoCapture('gals deck.mp4')
#
#        fourcc = cv.VideoWriter_fourcc(*'XVID')
#        out = cv.VideoWriter('output.avi', fourcc, 20.0, (640, 480))
#        find_line(cap=cap, frame=None, out_file=out)
#    else:
#        #frame = cv.imread("../images/frame-1.png")
#
#        #find_line(cap=None, frame=frame, out_file=None)
#
#    if is_camera:
#        cap.release()
#    cv.destroyAllWindows()
#
#if __name__ == '__main__':
#    main()
##############################
##############################
#   PROCEDURES
##############################
##############################
##############################
# fromcamera 
#   Find lines for image that was captured from camera  
##############################
#@argh.arg('fnamearr', nargs='+', type=str,  help='Array of TIF file names of the images' )
#@argh.arg('--numofrects', help='Number of grid rectangles required for vertical alignment' )
def fromcamera():
    ''' Capture image from camera and find lines'''
    fn='fromcamera()'
    log = mylog.configure_logging('fromcamera', "/tmp/findlines_algo.log")
    try:
        log.info('{}: START'.format(fn))
        #log.info('{}: ##############################'.format(fn))
        #log.info('{}: \tfnamearr:{}'.format(fn, fnamearr))
        #log.info('{}: ##############################'.format(fn))
        obj=AlgoFindLines(log=log, isfromros=False)
        
        cap = cv.VideoCapture(0)
        while True:
            ret, img = cap.read() 
            if ret!=True:
                log.error('{}: cap.read() Failed!!!'.format(fn))
                continue;
            imgout=obj.find_line(img)

        #if out_file !=None:
        #    out_file.write(normalizedFrame)
            cv.imshow('frame', imgout)

            if cv.waitKey(5) & 0xFF == ord('q'):
                break
    except Exception as err:
        msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(),err)
        log.error(msg)

##############################
# fromframefile
#   Find lines for image that was captured from one frame on disk 
##############################
#@argh.arg('fnamearr', nargs='+', type=str,  help='Array of TIF file names of the images' )
argh.arg('fname', help='File name to capture the image from' )
def fromframefile(fname):
    ''' Capture image from frame and find lines'''
    fn='fromframefile()'
    log = mylog.configure_logging('fromframefile', "/tmp/findlines_algo.log")
    try:
        log.info('{}: START'.format(fn))
        #log.info('{}: ##############################'.format(fn))
        #log.info('{}: \tfnamearr:{}'.format(fn, fnamearr))
        #log.info('{}: ##############################'.format(fn))
        
        obj=AlgoFindLines(log=log, isfromros=False)
        frame = cv.imread(fname)

        imgout = obj.find_line(frame)
        cv.imshow('frame', imgout)

        #if cv.waitKey(5) & 0xFF == ord('q'):
        #    return
        cv.waitKey(0) 
       
    except Exception as err:
        msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(),err)
        log.error(msg)

##############################
# fromvideo
#   Find lines for image that was captured from video file
##############################
#@argh.arg('fnamearr', nargs='+', type=str,  help='Array of TIF file names of the images' )
argh.arg('fname', help='File name to capture the image from' )
def fromvideofile(fname):
    ''' Capture image from video file and find lines'''
    fn='fromframe()'
    log = mylog.configure_logging('fromvideofile', "/tmp/findlines_algo.log")
    try:
        log.info('{}: START'.format(fn))
        #log.info('{}: ##############################'.format(fn))
        #log.info('{}: \tfnamearr:{}'.format(fn, fnamearr))
        #log.info('{}: ##############################'.format(fn))
        
        obj=AlgoFindLines(log=log, isfromros=False)
        cap = cv.VideoCapture(fname)
        while True:
            ret, img = cap.read() 
            if ret!=True:
                log.error('{}: cap.read() Failed!!!'.format(fn))
                continue;
            imgout=obj.find_line(img)

        #if out_file !=None:
        #    out_file.write(normalizedFrame)
            cv.imshow('frame', imgout)

            if cv.waitKey(5) & 0xFF == ord('q'):
                break

      
    except Exception as err:
        msg='{}: Failed!!! line:{} err:{}'.format(fn, myerror.lineno(),err)
        log.error(msg)


####################################
# MAIN
####################################
arg_parser = argh.ArghParser()
arg_parser.add_commands([
                            fromcamera,
                            fromframefile,
                            fromvideofile
                        ])
if __name__ == '__main__':
    arg_parser.dispatch()


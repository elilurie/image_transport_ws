2020-05-30
----------
    Create subscriber to /camera/image and publisher after find lines on /findlines/image
    ---------------------------------------------------------------------------
    How to run
    -----------
    terminal1
        $ source devel/setup.bash
        $ rosrun image_transport_tutorial camera_publish.py fname
            fname - 0 for camera 
                    other for video file name
                
     treminal2:
        $ source devel/setup.bash
        $ rosrun image_transport_tutorial findlines_sub_pub.py
 
2020-05-29
----------
    Create publisher of Video Camera with OpenCV VidCap in python
    -------------------------------------------------------------
        Topic: /camera/image
    How to run
    -----------
    treminal1:
        $ source devel/setup.bash
        $ rosrun image_transport_tutorial camera_publish.py
 
 2020-05-27
----------
    Create learning_image_transport package (http://wiki.ros.org/image_transport/Tutorials)
    ---------------------------------------
    cd ~/robodeck/dev
    $ mkdir -p ./image_transport_ws/src
    $ cd ./image_transport_ws/src
    $ source /opt/ros/melodic/setup.bash
    $ catkin_create_pkg learning_image_transport image_transport cv_bridge 
    $ cd ..
    $ rosdep install --from-paths src -i -y --rosdistro melodic
    $ catkin_make
    $ source devel/setup.bash

    Create a publisher
    ------------------
    Publishes video stream

    $ git clone https://github.com/ros-perception/image_common.git
    $ cd src
    $ cp -r ../image_common/image_transport/tutorial/ image_transport_tutorial
    (remove or copy the image_common project to other directory we dont need it anymore) 
    $ cd ..
    ~/robodeck/dev/image_transport_ws$ catkin_make

    Run publisher
    -------------
    terminal1: 
        $ source devel/setup.bash
        $ roscore
    treminal2:
        $ source devel/setup.bash
        $ rosrun image_transport_tutorial my_publisher 0 
    Run subcriber
    -------------
        $ source devel/setup.bash
        $ rosrun image_transport_tutorial my_subscriber
                


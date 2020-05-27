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
    $ git clone https://github.com/ros-perception/image_common.git
    $ cd src
    $ cp -r ../image_common/image_transport/tutorial/ image_transport_tutorial
    (remove or copy the image_common project to other directory we dont need it anymore) 
    $ cd ..
    ~/robodeck/dev/image_transport_ws$ catkin_make
 

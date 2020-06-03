#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <ros/console.h>


int main(int argc, char** argv)
{
    const char* fn="main()";
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

#ifdef PUBLISH_FILE
    
    cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Rate loop_rate(5);
    while (nh.ok()) {
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
#else
    ROS_INFO("%s: START", fn);
    // Convert the passed as command line parameter index for the video device to an integer
    std::istringstream video_sourceCmd(argv[1]);
    int video_source;
    // Check if it is indeed a number
    if(!(video_sourceCmd >> video_source)) return 1;

    cv::VideoCapture cap(video_source);
    // Check if video device can be opened with the given index
    if(!cap.isOpened())
    {
        ROS_ERROR("%s: cap.isOpened Failed!!!", fn);
        return 1;
    }
    ROS_INFO("%s: Open capture on video source %d Success", fn, video_source);
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(30);
    while (nh.ok()) {
      cap >> frame;
      // Check if grabbed frame is actually full with some content
      if(!frame.empty()) {
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        cv::waitKey(1);
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
#endif
}


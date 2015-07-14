#ifndef _PEOPLE_DETECTOR_HPP_
#define _PEOPLE_DETECTOR_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class PeopleDetector
{
private:

    ros::NodeHandle n;
    ros::Subscriber camera_sub;

public:
    PeopleDetector();
    ~PeopleDetector(){;}

    void hog_detector(cv::Mat current_frame);
    void cameraSubscriber(const sensor_msgs::ImageConstPtr& _camera);

};

#endif

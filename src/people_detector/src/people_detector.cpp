

#include "people_detector.hpp"


PeopleDetector::PeopleDetector():n("~")
{
    camera_sub = n.subscribe/*<sensor_msgs::ImageConstPtr>*/("/camera/rgb/image_raw", 1, &PeopleDetector::cameraSubscriber, this);
}


void PeopleDetector::hog_detector(cv::Mat current_frame)
{

    cv::HOGDescriptor hog;
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    cv::namedWindow("video capture", CV_WINDOW_AUTOSIZE);

    if (!current_frame.data)
        return;

    std::vector<cv::Rect> found, found_filtered;
    hog.detectMultiScale(current_frame, found, 0, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);

    size_t i, j;
    for (i=0; i<found.size(); i++)
    {
        cv::Rect r = found[i];
        for (j=0; j<found.size(); j++)
            if (j!=i && (r & found[j])==r)
                break;
        if (j==found.size())
            found_filtered.push_back(r);
    }
    for (i=0; i<found_filtered.size(); i++)
    {
        cv::Rect r = found_filtered[i];
        r.x += cvRound(r.width*0.1);
        r.width = cvRound(r.width*0.8);
        r.y += cvRound(r.height*0.06);
        r.height = cvRound(r.height*0.9);
        cv::rectangle(current_frame, r.tl(), r.br(), cv::Scalar(0,255,0), 2);
    }

    cv::imshow("view", current_frame);
    if (cv::waitKey(20) >= 0)
        return;
}

void PeopleDetector::cameraSubscriber(const sensor_msgs::ImageConstPtr& _camera)
{
    cv_bridge::CvImageConstPtr cv_ptr_rgb;

    try
    {
        cv_ptr_rgb = cv_bridge::toCvCopy(_camera, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", _camera->encoding.c_str());
    }

    cv::Mat tmp_frame = cv_ptr_rgb->image;
    cv::Mat frame(cv::Size(320,240), CV_8UC3);
    cv::resize(tmp_frame, frame, frame.size());

    hog_detector(frame);

//    cv::waitKey(30);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "people_detector");

    PeopleDetector people_detector;
    ros::Rate loop_rate(5); // [Hz]

    ros::spin();
//    while(ros::ok())
//    {
//        ros::spinOnce();
//        loop_rate.sleep();
//    }

    return 0;
}

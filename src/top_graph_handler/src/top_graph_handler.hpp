#ifndef _TOP_GRAPH_HANDLER_
#define _TOP_GRAPH_HANDLER_

#include <topological_graph.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <opencv2/highgui/highgui.hpp>


#define VISUALIZER

class TopGraphHandler
{
private:
    ros::NodeHandle n;
    tf::TransformListener* listener;
    std::string out_lp_file;

//    /// CONTROLLER
    TopGraph* theTopGraph;
    int path_id;
    std::map<int, std::vector<cv::Point3f> > path_to_target;

//    /// ROBOT ----------------------------
    std::string robotname;
    cv::Point3f robot_pose;
    std::string move_task_state;

//    /// MAP -------------------------------
    cv::Mat map;
    float resolution;
    float origin_x;
    float origin_y;

//    /// PUB -------------------------------
    ros::Publisher path_pub;
    std_msgs::String path_to_publish;

    ros::Publisher controller_state_pub;
    std_msgs::String state_to_publish;

//    /// SUB -------------------------------
    ros::Subscriber map_sub;
    ros::Subscriber move_sub;

//    /// VIS -------------------------------
    cv::Mat vis;
    void visualize();

    void init();
    void getRobotPose();

public:

    TopGraphHandler();
    ~TopGraphHandler();

    void mapSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& _map);
    void moveSubscriber(const std_msgs::String::ConstPtr& _msg);
    void dialogSubscriber(const std_msgs::String::ConstPtr& _msg);

    // main method
    void update();
};

#endif

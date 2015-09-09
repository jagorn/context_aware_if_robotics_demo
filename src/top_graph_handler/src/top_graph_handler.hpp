#ifndef _TOP_GRAPH_HANDLER_
#define _TOP_GRAPH_HANDLER_

#include <topological_graph.h>

#include <adaptive_controller/ContextROIs.h>
#include <adaptive_controller/ContextROI.h>
#include <adaptive_controller/ActionRequest.h>
#include <adaptive_controller/ActionFeedback.h>

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
    int current_path;
    std::map<int, std::vector<cv::Point3f> > path_to_target;

//    /// ROBOT ----------------------------
    std::string robotname;
    cv::Point3f robot_pose;
    std::string move_task_state;
    cv::Point3f destination;

    bool task_assigned;
    bool moving;
    std::string task;

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

    ros::Publisher task_pub;

//    /// SUB -------------------------------
    ros::Subscriber map_sub;
    ros::Subscriber move_sub;
    ros::Subscriber task_sub;
    ros::Subscriber context_middleware_sub;

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
    void contextMidSubscriber(const adaptive_controller::ContextROIs::ConstPtr& _msg);
    void taskSubscriber(const adaptive_controller::ActionRequest::ConstPtr& _msg);

    // main method
    void update();
};

#endif

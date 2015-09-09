
#include "top_graph_handler.hpp"

TopGraphHandler::TopGraphHandler(): n("~")
{
    n.param<std::string>("robot_name", robotname, "robot0");
    n.param<std::string>("out_lp_file", out_lp_file, "./test.lp");

    map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &TopGraphHandler::mapSubscriber, this);
    move_sub = n.subscribe<std_msgs::String>("/" + robotname + "/way_point_navigation/feedbackMotion", 1000, &TopGraphHandler::moveSubscriber, this);

    task_sub = n.subscribe<adaptive_controller::ActionRequest>("/" + robotname + "/actions/request", 1000, &TopGraphHandler::taskSubscriber, this);
    context_middleware_sub = n.subscribe<adaptive_controller::ContextROIs>("/" + robotname + "/context/ROIs", 1000, &TopGraphHandler::contextMidSubscriber, this);

    task_pub = n.advertise<adaptive_controller::ActionFeedback>("/" + robotname + "/actions/feedback", 1000);
    path_pub = n.advertise<std_msgs::String>("targetPose", 1000);
    controller_state_pub = n.advertise<std_msgs::String>("MoveControllerState", 1000);

    // initialize the listener
    while(ros::Time::now() == ros::Time(0));
    this->listener = new tf::TransformListener();

    usleep(1e6);
    ros::spinOnce();

    init();
}

TopGraphHandler::~TopGraphHandler()
{
    delete theTopGraph;
}

void TopGraphHandler::init()
{
    theTopGraph = new TopGraph(map, out_lp_file);

    path_id = 0;
    current_path = -1;

    task_assigned = false;
    task = "";

    moving = false;

    move_task_state = "INIT";
    state_to_publish.data = "INIT";

    controller_state_pub.publish(state_to_publish);
}

void TopGraphHandler::visualize()
{
    cv::Mat vis;
    cv::cvtColor(map, vis, CV_GRAY2RGB);

    theTopGraph->vis(vis);

    if(path_to_target.size() != 0 )
        Utils::drawPath(vis, &(path_to_target.begin()->second), CV_RGB(255,0,0), 1);
    Utils::drawRobot(vis, 1, robot_pose);

    cv::imshow("vis", vis);
    cv::waitKey(1);
}

void TopGraphHandler::mapSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& _map)
{
    int width = _map->info.width;
    int height = _map->info.height;

    map = cv::Mat(height, width, CV_8U);
    resolution = _map->info.resolution;
    origin_x = _map->info.origin.position.x;
    origin_y = _map->info.origin.position.y;

    ROS_INFO("Map received.");

    for(int i = 0, i_rev = height - 1; i < height;++i, --i_rev)
    {
        for(int j = 0; j < width; j++)
        {
            switch(_map->data[i_rev*width + j])
            {
            default:
            case -1:
                this->map.data[i*width + j] = 150;
                break;
            case 0:
                this->map.data[i*width + j] = 255;
                break;
            case 100:
                this->map.data[i*width + j] = 0;
                break;
            }
        }
    }

    ROS_INFO("Image extracted from map.");
}

void TopGraphHandler::moveSubscriber(const std_msgs::String::ConstPtr& _msg)
{
    move_task_state = _msg->data;
    state_to_publish.data = _msg->data;

    if (_msg->data == "PENDING") {
        moving = true;
    }

    if ((_msg->data == "FINISHED" || _msg->data == "SUCCEEDED" ) && moving) {

        ROS_INFO("task completed - %s", _msg->data.c_str());

        moving = false;
        adaptive_controller::ActionFeedback feedback;
        feedback.robot = robotname;
        feedback.action = task;
        feedback.value = "success";
        task_pub.publish(feedback);
    }
}

void TopGraphHandler::taskSubscriber(const adaptive_controller::ActionRequest::ConstPtr& _msg)
{
    ROS_INFO("New task received");

    if (robotname == _msg->robot) {
        ROS_INFO("ROBOT MATCHING");

        if( _msg->action.substr (0,4) == "goTo") {
            ROS_INFO("ACTION MATCHING");

            std::string parameters = _msg->action.substr(5, _msg->action.size()-6);

            ROS_INFO("task = %s", _msg->action.c_str());
            ROS_INFO("parameters = %s", parameters.c_str());

            int delimiter = parameters.find(",");
            int paramX = atoi(parameters.substr(0,delimiter).c_str());
            int paramY = atoi(parameters.substr(delimiter+1, parameters.size() - (delimiter + 1)).c_str());


            int x = (paramX - origin_x)/resolution;
            int y = map.size().height - (paramY - this->origin_y)/resolution;

            ROS_INFO("x = %d, y = %d", x, y);


            destination = cv::Point3f(x, y, 0);
            task = _msg->action;
            task_assigned = true;
        }
    }
}

void TopGraphHandler::contextMidSubscriber(const adaptive_controller::ContextROIs::ConstPtr& _msg)
{
    ROS_INFO("Context message received");

    std::vector<TopGraph::Area> areas;
    std::vector<adaptive_controller::ContextROI> rois = _msg->ROIs;

    for (int i = 0; i < rois.size(); i++) {

        adaptive_controller::ContextROI roi = rois[i];

        int centreX = (roi.centreX - origin_x)/resolution;
        int centreY = map.size().height - (roi.centreY - this->origin_y)/resolution;
        float weight = roi.weight / 10.0;

        std::vector<float> dimensions;
        if (roi.shape == "circle") {
            dimensions.push_back((float(roi.radius)/2.0)/resolution);
        }
        else {
            dimensions.push_back(roi.side/resolution);
            dimensions.push_back(roi.height/resolution);
        }

        areas.push_back( TopGraph::Area(roi.id, roi.shape, cv::Point2f(centreX, centreY), weight, dimensions) );
    }

    theTopGraph->updateContextAreas(&areas);
    task_assigned = true;
}


void TopGraphHandler::getRobotPose()
{
    tf::StampedTransform transform;

    if(resolution != -1)
    {
        try
        {
            listener->waitForTransform("/" + robotname + "/map", "/" + robotname + "/base_link", ros::Time(0), ros::Duration(1.0));
            listener->lookupTransform("/map", "/" + robotname + "/base_link", ros::Time(0), transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        robot_pose = cv::Point3f((transform.getOrigin().x() - origin_x)/resolution,
                                 map.size().height - (transform.getOrigin().y() - this->origin_y)/resolution,
                                 tf::getYaw(transform.getRotation()));
    }
}

void TopGraphHandler::update()
{
    getRobotPose();

    std::vector<cv::Point3f> waypoints;
    theTopGraph->computePath(robot_pose, destination, &waypoints );
    if(task_assigned)
    {
        task_assigned = false;
        path_to_target.clear();
        path_to_target.insert(std::make_pair<int, std::vector<cv::Point3f> >( path_id++, waypoints) );
    }

    if(path_id != current_path && path_id != 0)
    {
        current_path = path_id;
        path_to_publish.data = std::string("Path_-"+Utils::to_string(current_path)+" 0");
        for(unsigned int i=0; i<path_to_target.begin()->second.size(); ++i)
        {
            path_to_publish.data +=
                    + " " + Utils::to_string( (path_to_target.begin()->second.at(i).x *resolution) + origin_x)
                    + " " + Utils::to_string( -(path_to_target.begin()->second.at(i).y -map.size().height) *resolution + origin_y)
                    + " " + Utils::to_string( path_to_target.begin()->second.at(i).z);
        }

        path_pub.publish(path_to_publish);
    }

    controller_state_pub.publish(state_to_publish);

#ifdef VISUALIZER
    visualize();
#endif
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "topological_graph");

    TopGraphHandler tgh;
    ros::Rate loop_rate(5); //[Hz]

    while(ros::ok())
    {
        tgh.update();

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

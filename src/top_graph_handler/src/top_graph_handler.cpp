

#include "top_graph_handler.hpp"

TopGraphHandler::TopGraphHandler(): n("~")
{
    n.param<std::string>("robot_name", robotname, "robot0");
    n.param<std::string>("out_lp_file", out_lp_file, "./test.lp");
    map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &TopGraphHandler::mapSubscriber, this);
    move_sub = n.subscribe<std_msgs::String>("/" + robotname + "/way_point_navigation/feedbackMotion",
                                                 1000, &TopGraphHandler::moveSubscriber, this);
    context_middleware_sub = n.subscribe<std_msgs::String>("/" + robotname + "/contextROI",
                                                           1000, &TopGraphHandler::contextMidSubscriber, this);

    path_pub = n.advertise<std_msgs::String>("targetPose", 1000);
    controller_state_pub = n.advertise<std_msgs::String>("MoveControllerState", 1000);

    // initialize the listner
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
    move_task_state = "INIT";

    state_to_publish.data = "INIT";
    controller_state_pub.publish(state_to_publish);
}

void TopGraphHandler::visualize()
{
    cv::Mat vis;
    cv::cvtColor(map, vis, CV_GRAY2RGB);

    theTopGraph->vis(vis);

    Utils::drawPath(vis, &path_to_target[1], CV_RGB(255,0,0), 1);
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
}

void TopGraphHandler::contextMidSubscriber(const std_msgs::String::ConstPtr& _msg)
{
    // "id centroid p1 p2"
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
    if(path_id == 0)
    {
        float weight = 100;
        std::vector<TopGraph::Area> areas;

        std::vector<float> param_circle(1,30);
        areas.push_back( TopGraph::Area(areas.size(), "circle", cv::Point2f(500,350), weight, param_circle) );
        areas.push_back( TopGraph::Area(areas.size(), "circle", cv::Point2f(600,350), weight, param_circle) );

        std::vector<float> param_rect;
        param_rect.push_back(50);
        param_rect.push_back(30);
        areas.push_back( TopGraph::Area(areas.size(), "rect", cv::Point2f(380,390), weight, param_rect) );


        theTopGraph->updateContextAreas(&areas);
    }

//    if() //new task to start
    std::vector<cv::Point3f> waypoints;
    theTopGraph->computePath(robot_pose, cv::Point3f(760,80,-M_PI+M_PI/4), &waypoints );

    path_to_target.insert(std::make_pair<int, std::vector<cv::Point3f> >( 1, waypoints) );

    if(path_id == 0) //NOT GOOD
    {
        ++path_id;
        path_to_publish.data = std::string("Path_-"+Utils::to_string(path_id)+" 0");
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

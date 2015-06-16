#ifndef COORD_BASIC_DRAWING
#define COORD_BASIC_DRAWING

#include <opencv2/highgui/highgui.hpp>
#include <cmath>

namespace Utils
{

static cv::Scalar robot_color = CV_RGB(255,0,0);
static cv::Scalar door_color = CV_RGB(0,0,255);
static cv::Scalar event_color = CV_RGB(0,180,0);
static float ARROW_MODULE = 8;

void arrow(cv::Mat img, int x, int y, int u, int v, cv::Scalar color, int size, int thickness);

void number(cv::Mat img, int _num, cv::Point2f _p, cv::Scalar color = CV_RGB(0,0,0), cv::Point2f displacement=cv::Point2f(-10,-5));

void drawPath(cv::Mat img, std::vector<cv::Point3f>* path, cv::Scalar color = CV_RGB(0,255,0), float thickness = 1.f);

void drawTask(cv::Mat img, unsigned int t_id, cv::Scalar color, std::vector<cv::Point3f>* path, float thickness = 1.f);

void drawRobot(cv::Mat img, unsigned int r_id, cv::Point3f pose, cv::Scalar color = robot_color);

void drawDoor(cv::Mat img, unsigned int d_id, cv::Point3f pose, cv::Scalar color = door_color);

void drawEvent(cv::Mat img, unsigned int e_id, cv::Point2f pose, cv::Scalar color = event_color);

void drawTarget(cv::Mat img, unsigned int t_id, cv::Point2f pose);

void drawProxPoint(cv::Mat img, cv::Point3f pose, float arrow_module = ARROW_MODULE);

void drawProxemicsArea(cv::Mat img, cv::Point3f pose, bool moving = true);

}

#endif

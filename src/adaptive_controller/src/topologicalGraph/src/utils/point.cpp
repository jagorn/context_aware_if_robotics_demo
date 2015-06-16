#include "point.h"


Utils::Point2f::Point2f(const Point2f& _p)
{
    x = _p.x;
    y = _p.y;
}

Utils::Point3f::Point3f(const Point3f& _p)
{
    x = _p.x;
    y = _p.y;
    z = _p.z;
}

float Utils::norm2D(Utils::Point2f p1, Utils::Point2f p2)
{
    Utils::Point2f delta;
    delta.x = p2.x-p1.x;
    delta.y = p2.y-p1.y;

    return sqrt(delta.x*delta.x + delta.y*delta.y);
}

Utils::Point3f Utils::cv2co_point(cv::Point3f cv_p)
{
    return Point3f(cv_p.x, cv_p.y, cv_p.z);
}

Utils::Point2f Utils::cv2co_point(cv::Point2f cv_p)
{
    return Point2f(cv_p.x, cv_p.y);
}

cv::Point3f Utils::co2cv_point(Utils::Point3f co_p)
{
    return cv::Point3f(co_p.x, co_p.y, co_p.z);
}

cv::Point2f Utils::co2cv_point(Utils::Point2f co_p)
{
    return cv::Point2f(co_p.x, co_p.y);
}


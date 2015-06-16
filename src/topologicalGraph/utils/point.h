#ifndef COORD_POINT
#define COORD_POINT

#include <cmath>
#include <opencv2/highgui/highgui.hpp>


namespace Utils
{

class Point2f
{
public:
    float x,y;
    Point2f():x(0),y(0){}
    Point2f(float _x,float _y): x(_x),y(_y){}
    Point2f(const Point2f& _p);
    void operator=(const Point2f& p)
    {
       x = p.x;
       y = p.x;
    }

    inline float atan2(){ return std::atan2(y,x); }
    inline float norm(){ return std::sqrt(x*x + y*y); }
};

class Point3f
{
public:
    float x,y,z;
    Point3f():x(0),y(0),z(0){}
    Point3f(float _x,float _y): x(_x),y(_y),z(0){}
    Point3f(float _x,float _y,float _z): x(_x),y(_y),z(_z){}
    Point3f(const Point3f& _p);
    void operator=(const Point3f& p)
    {
       x = p.x;
       y = p.y;
       z = p.z;
    }

    inline Utils::Point2f translation(){ return Utils::Point2f(x,y); }
    inline float atan2(){ return std::atan2(y,x); }
    inline float norm(){ return std::sqrt(x*x + y*y); }
    inline float norm3(){ return std::sqrt(x*x + y*y + z*z); }

};

float norm2D(Utils::Point2f p1, Utils::Point2f p2);

Utils::Point3f cv2co_point(cv::Point3f cv_p);
Utils::Point2f cv2co_point(cv::Point2f cv_p);
cv::Point3f co2cv_point(Utils::Point3f co_p);
cv::Point2f co2cv_point(Utils::Point2f co_p);


}

#endif

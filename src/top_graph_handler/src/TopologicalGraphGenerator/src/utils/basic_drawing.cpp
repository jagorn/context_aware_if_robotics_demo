#include "basic_drawing.h"

namespace Utils
{

void arrow(cv::Mat img, int x, int y, int u, int v, cv::Scalar color, int size, int thickness)
{
    if(u == 0 && v == 0) cv::circle(img, cv::Point2f(x, y), thickness, color, CV_FILLED);
    else if(!(u == 0 && v == 0)){
        float l = size;

        float theta = 0;
        if(u == 0 && v > 0) theta = 1.57;
        else if(u == 0 && v < 0) theta = -1.57;

        float alpha = 2.30;
        if(u != 0) theta = atan2(v,u);

        cv::Point2f p0( x +u, y +v);
        cv::Point2f p1( x +u +(l*2)*cos(theta), y +v +(l*2)*sin(theta) );
        cv::Point2f p2( x +u +l*cos(theta-alpha) , y +v +l*sin(theta-alpha) );
        cv::Point2f p3( x +u +l*cos(theta+alpha) , y +v +l*sin(theta+alpha) );

        cv::line( img, cv::Point2f( x, y ), cv::Point2f( x+ u, y+ v ), color, thickness, 8);

        cv::line( img, p1, p2, color, thickness, 1);
        cv::line( img, p1, p3, color, thickness, 1);
        cv::line( img, p0, p3, color, thickness, 1);
        cv::line( img, p0, p2, color, thickness, 1);
    }
}

void text(cv::Mat img, std::string _text, cv::Point2f _p, cv::Scalar color, cv::Point2f displacement)
{
    float fontScale = 0.3f;
    int thickness = 1;

    cv::putText(img,_text,_p+displacement,cv::FONT_HERSHEY_SCRIPT_SIMPLEX,fontScale,color,thickness);
}

void drawCircleArea(cv::Mat img, std::string e_id, cv::Point2f pose, float radius, float weight)
{
    float green = 255 * (1 - weight/10.0);
    float red = 255;
    float blue = 50;
    cv::circle( img, pose, radius, CV_RGB(int(red),int(green),int(blue)), CV_FILLED);
    text(img, e_id, pose, cv::Scalar::all(0));
}

void drawRectArea(cv::Mat img, std::string e_id, cv::Point2f pose, float base, float height, float weight)
{
    float green = 255 * (1 - weight/10.0);
    float red = 255;
    float blue = 50;
    cv::rectangle( img, pose - cv::Point2f(base/2,-height/2), pose + cv::Point2f(base/2,-height/2), CV_RGB(int(red),int(green),int(blue)), 4);
    text(img, e_id, pose, cv::Scalar::all(0));
}

void drawRobot(cv::Mat img, unsigned int r_id, cv::Point3f pose, cv::Scalar color)
{
    int robot_radius = 6;

    cv::Point2f r_pos;
    r_pos.x = pose.x;
    r_pos.y = pose.y;

    cv::Point2f r_rot;
    r_rot.x = r_pos.x + (robot_radius-2)*cos(-pose.z);
    r_rot.y = r_pos.y + (robot_radius-2)*sin(-pose.z);

    cv::circle( img, r_pos, robot_radius +1, cv::Scalar::all(0), CV_FILLED );
    cv::circle( img, r_pos, robot_radius, color, CV_FILLED );

    cv::circle( img, r_rot, (robot_radius/3) +1, cv::Scalar::all(0), CV_FILLED);
    cv::circle( img, r_rot, robot_radius/3, CV_RGB(255,255,0), CV_FILLED);

    std::stringstream ss;
    ss << r_id;
    std::string string_id = ss.str();

    text(img, string_id, r_pos, cv::Scalar::all(0));
}

void drawPath(cv::Mat img, std::vector<cv::Point3f>* path, cv::Scalar color, float thickness)
{
    if(path->size() == 1)
    {
        cv::Point2f p;
        p.x = path->at(0).x;
        p.y = path->at(0).y;

        cv::circle( img, p, 2, CV_RGB(0,0,255), CV_FILLED );
    }
    else
    {
        for(unsigned int tn=1; tn<path->size(); ++tn)
        {
            cv::Point2f p1,p2;
            p1.x = path->at(tn-1).x;
            p1.y = path->at(tn-1).y;
            p2.x = path->at(tn).x;
            p2.y = path->at(tn).y;

            cv::circle( img, p1, 2, CV_RGB(0,0,255), CV_FILLED );

            if(tn == (path->size()-1))
                cv::circle( img, p2, 2, CV_RGB(0,0,255), CV_FILLED );

            cv::line( img, p1, p2, CV_RGB(100,50,0), 3, 1 );
        }
    }
}

void drawTask(cv::Mat img, unsigned int t_id, cv::Scalar color, std::vector<cv::Point3f>* path, float thickness)
{
    std::stringstream ss;
    ss << t_id;
    std::string string_id = ss.str();

    if(path->size() == 1)
    {
        cv::Point2f p;
        p.x = path->at(0).x;
        p.y = path->at(0).y;

        cv::circle( img, p, 5, color, CV_FILLED );
        text(img, string_id, p, cv::Scalar::all(0));
    }
    else
    {
        for(unsigned int tn=1; tn<path->size(); ++tn)
        {
            cv::Point2f p1,p2;
            p1.x = path->at(tn-1).x;
            p1.y = path->at(tn-1).y;
            p2.x = path->at(tn).x;
            p2.y = path->at(tn).y;

            cv::circle( img, p1, 2, CV_RGB(0,0,0), CV_FILLED );
            if(tn == (path->size()-1))
            {
                cv::circle( img, p2, 5, color, CV_FILLED );
                text(img, string_id, p2, cv::Scalar::all(0));
            }

            cv::line( img, p1, p2, color, thickness, 1 );
        }
    }
}

void drawDoor(cv::Mat img, unsigned int d_id, cv::Point3f pose, cv::Scalar color)
{
    cv::Point2f d_pos_lower, d_pos_upper;
    if(pose.z != 0.f)
    {
        d_pos_lower.x = pose.x-10;
        d_pos_lower.y = pose.y;
        d_pos_upper.x = pose.x;
        d_pos_upper.y = pose.y+20;
    }
    else
    {
        d_pos_lower.x = pose.x-10;
        d_pos_lower.y = pose.y;
        d_pos_upper.x = pose.x+10;
        d_pos_upper.y = pose.y+10;
    }

    std::stringstream ss;
    ss << d_id;
    std::string string_id = ss.str();

    cv::rectangle( img, d_pos_lower, d_pos_upper, cv::Scalar::all(0), CV_FILLED);
    cv::rectangle( img, d_pos_lower+cv::Point2f(1,1), d_pos_upper+cv::Point2f(-1,-1), color, CV_FILLED);
    text(img, string_id, d_pos_upper, cv::Scalar::all(255), cv::Point2f(-10,-1));
}

void drawEvent(cv::Mat img, unsigned int e_id, cv::Point2f pose, cv::Scalar color)
{
    float angle = 360.0f/(2*(float)6);

    for(float i=0; i<360; i+=2*angle)
        cv::ellipse(img, pose, cv::Size(9,9), 0, i, i+angle, cv::Scalar::all(0), -1, CV_AA);

    for(float i=1; i<360; i+=2*angle)
        cv::ellipse(img, pose, cv::Size(8,8), 0, i+5, i+angle-5, color, -1, CV_AA);

    std::stringstream ss;
    ss << e_id;
    std::string string_id = ss.str();

    text(img, string_id, pose, cv::Scalar::all(255), cv::Point2f(0,-20));
}

void drawTarget(cv::Mat img, unsigned int t_id, cv::Point2f pose)
{
    cv::circle( img, pose, 9, CV_RGB(0,0,0), CV_FILLED );
    cv::circle( img, pose, 8, CV_RGB(255,0,0), CV_FILLED );
    cv::circle( img, pose, 6, CV_RGB(255,255,255), CV_FILLED );
    cv::circle( img, pose, 4, CV_RGB(255,0,0), CV_FILLED );
}

void drawProxPoint(cv::Mat img, cv::Point3f pose, float arrow_module)
{

    cv::circle( img, cv::Point2f(pose.x,pose.y), 3, CV_RGB(0,0,255), CV_FILLED );
    arrow(img,
          pose.x, pose.y,
          arrow_module*cos(pose.z),
          arrow_module*sin(pose.z),
          CV_RGB(0,0,255), 1, 1);
}

void drawProxemicsArea(cv::Mat img, cv::Point3f pose, bool moving)
{
    cv::Point2f position(pose.x,pose.y);
    float radius = 10;
    int rK = 3;

    for (float r=4; r>=0; --r)
    {
        if(moving)
        {
            cv::ellipse(img, position, cv::Size(radius*(1 + r/2),radius*(1 + r/2)),
                        0, -pose.z*(180/M_PI)+90, -pose.z*(180/M_PI)-90, cv::Scalar(r*50,r*50,r*50), CV_FILLED, 0);
        }
        else
        {
            if(r==4)
                cv::ellipse(img, position, cv::Size(radius*(1 + r/2),radius*(1 + r/2)),
                            0, -pose.z*(180/M_PI)+90, -pose.z*(180/M_PI)-90, cv::Scalar(r*50,r*50,r*50), CV_FILLED, 0);
            else if(r==3)
                cv::ellipse(img, position, cv::Size(radius*(1 + r/2),radius*(1 + r/2)),
                            0, -pose.z*(180/M_PI)+90, -pose.z*(180/M_PI)-90, cv::Scalar(0,250,50), CV_FILLED, 0);
            else if(r==2)
                cv::ellipse(img, position, cv::Size(radius*(1 + r/2),radius*(1 + r/2)),
                            0, -pose.z*(180/M_PI)+90, -pose.z*(180/M_PI)-90, cv::Scalar(0,250,250), CV_FILLED, 0);
            else if(r==1)
                cv::ellipse(img, position, cv::Size(radius*(1 + r/2),radius*(1 + r/2)),
                            0, -pose.z*(180/M_PI)+90, -pose.z*(180/M_PI)-90, cv::Scalar(0,0,255), CV_FILLED, 0);


            cv::line( img, position, cv::Point2f(position.x + radius*rK*cos(0.75f-pose.z),
                                                 position.y + radius*rK*sin(0.75f-pose.z)),
                      cv::Scalar::all(0), 1.5, 1);
            cv::line( img, position, cv::Point2f(position.x + radius*rK*cos(-0.75f-pose.z),
                                                 position.y + radius*rK*sin(-0.75f-pose.z)),
                      cv::Scalar::all(0), 1.5, 1);
        }

    }
}

}


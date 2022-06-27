#include <ros_func.h>

namespace ros_func
{

std_msgs::ColorRGBA ColorRGBA(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA ret;
    ret.b = b;    ret.g = g;    ret.r = r;    ret.a = a;
    return ret;
}



}
/**
 * @file ros_func.h
 * @brief ROSメッセージ型の生成
 */
#ifndef __ROS_FUNC__
#define __ROS_FUNC__
#include "pose.h"

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>

namespace ros_func
{
using namespace kinematics;

std_msgs::ColorRGBA ColorRGBA(double r, double g, double b, double a);

template <typename T>
geometry_msgs::Point Point(vec3<T> p)
{
    geometry_msgs::Point ret;
    ret.x = p.x;    ret.y = p.y;    ret.z = p.z;
    return ret;
}

template <typename T>
geometry_msgs::Quaternion Quaternion(vec4<T> p)
{
    geometry_msgs::Quaternion ret;
    ret.x = p.x;    ret.y = p.y;    ret.z = p.z;    ret.w = p.w;
    return ret;
}

template <typename T>
geometry_msgs::Pose Pose(pose<T> pos)
{
    geometry_msgs::Pose ret;
    ret.position = Point(pos.p);
    ret.orientation = Quaternion(pos.q);
    return ret;
}

}

#endif

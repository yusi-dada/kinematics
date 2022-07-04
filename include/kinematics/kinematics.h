#pragma once
#include <kinematics/vec3.h>
#include <kinematics/vec4.h>
#include <kinematics/pose.h>
#include <kinematics/pose_array.h>
#include <kinematics/camera.h>
#include <ros_func/ros_marker.h>

typedef kinematics::vec3<double> vec3d;
typedef kinematics::vec4<double> vec4d;
typedef kinematics::pose<double> posed;
typedef kinematics::pose_array<double> pad;
typedef kinematics::camera<double> camd;

typedef kinematics::vec3<float> vec3f;
typedef kinematics::vec4<float> vec4f;
typedef kinematics::pose<float> posef;
typedef kinematics::pose_array<float> paf;
typedef kinematics::camera<float> camf;

typedef std::vector<kinematics::vec3<double>> mat3d;
typedef std::vector<kinematics::vec3<float>> mat3f;

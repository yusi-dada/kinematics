/**
 * @file kinematics.h
 * @brief ライブラリインクルード用
 */
#pragma once
#include <kinematics/camera.h>      // camera, pose, vec4, vec3
#include <ros_func/ros_marker.h>    // ros_func

typedef kinematics::vec3<double> vec3d;
typedef kinematics::vec4<double> vec4d;
typedef kinematics::pose<double> posed;
typedef kinematics::camera<double> camd;

typedef kinematics::vec3<float> vec3f;
typedef kinematics::vec4<float> vec4f;
typedef kinematics::pose<float> posef;
typedef kinematics::camera<float> camf;

typedef std::vector<kinematics::vec3<double>> mat3d;
typedef std::vector<kinematics::vec3<float>> mat3f;

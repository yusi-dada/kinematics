#include "ros_marker.h"
#include "pose_array.h"
#include "camera.h"
#include "robot.h"
using namespace kinematics;
using namespace ros_func;
typedef pose<double> posed;
typedef vec4<double> vec4d;
typedef vec3<double> vec3d;
typedef pose_array<double> pad;



int main(int argc, char **argv)
{
    // init ROS node
    ros::init (argc, argv, "test_pose");
    rviz_publisher rp("test");
    ros::Rate rate (100);

    double jnt[6]={0,0,M_PI/2,0,M_PI/2,0};

    pose_array<double> robo(posB(), alfa(), jnt);
    std::cerr << robo << std::endl;

    auto tool = robo[-1];

    return 0;
}
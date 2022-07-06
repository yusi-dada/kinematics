#include <ros_func/ros_marker.h>
#include <kinematics/pose_array.h>
#include <kinematics/camera.h>
#include <robot/robot.h>
using namespace kinematics;
using namespace ros_func;
typedef pose<double> posed;
typedef vec4<double> vec4d;
typedef vec3<double> vec3d;

int main(int argc, char **argv)
{
    // init ROS node
    ros::init (argc, argv, "test_pose");
    rviz_publisher rp("test");
    PathMarker pm("path", 50);

    ros::Rate rate (100);

    double jnt[6]={0,0,M_PI/2,0,M_PI/2,0};
    float t=0;
    while (ros::ok ())
    {
        t+=1.0/100.0;
        jnt[3] = M_PI/6 * sin(t);
        jnt[5] = M_PI/6 * sin(t+M_PI/4);

        robot<double> robo(jnt);
        pm.append(robo.links[-1].p);
        
        robo.rviz(rp);

        rate.sleep ();
    }
    return 0;
}
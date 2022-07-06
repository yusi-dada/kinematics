#include <ros_func/ros_marker.h>
#include <kinematics/pose.h>
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
    //PathMarker pm("test_path",300);
    ros::Rate rate (100);

    std::unique_ptr<posed> ptr;
    {
        posed pos(vec3d(0,0,0), vec4d(0,0,0));
        posed pos1(vec3d(1,2,3), vec4d(0,0,0));
        std::cerr << "pos=" << pos << std::endl;
        ptr.reset(&pos);
        std::cerr << "ptr=" << ptr.get() << std::endl;
        pos = pos1;
        std::cerr << "ptr=" << ptr.get() << std::endl;
        std::cerr << "pos=" << pos << std::endl;
    }
        std::cerr << "ptr=" << ptr.get() << std::endl;


    double T=0;
    while(ros::ok())
    {
        rate.sleep ();
    }

    return 0;
}
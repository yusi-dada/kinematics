#include <ros_func/ros_marker.h>
#include <robot/robot.h>
using namespace kinematics;
using namespace ros_func;

int main(int argc, char **argv)
{
    // init ROS node
    ros::init (argc, argv, "test_pose");

    rviz_publisher rp("test");
    PathMarker pm("test_path",300);
    ros::Rate rate (100);

    kinematics::joint<double> jnt = {0,0,M_PI/2, 0, M_PI/2, 0};
    kinematics::pose<double> base;


    double T=0;
    while(ros::ok())
    {
        auto ma = InitMarker();

        std::vector<fpose<double>> pose_array = kinematics::to_pose_array(jnt, base);
        rp.setAxis("world", "link", to_pose_array(pose_array));
        LinkMarker(ma, "world", to_pose_array(pose_array), ColorRGBA(1,1,1,1), "link");

        std::cerr << pose_array.back() << std::endl;

        rp.publish(ma);

        rate.sleep ();
    }

    return 0;
}
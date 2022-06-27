#include "ros_marker.h"
#include "pose_array.h"
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

    // 座標系S0を基準座標系に設定
    vec3d S0_p(0,0,0);
    vec4d S0_q;

    // 変換先の座標系S1を設定
    vec3d S1_p(0,0,0);
    vec4d S1_q(vec3d(1,0,1), 3*M_PI); // 180deg回転だとRotationTo関数が演算できない

    // S0→S1にするための回転軸axisと回転角thetaを導出
    auto p = S0_q.RotationTo2(S1_q);
    auto axis = p.first;
    auto theta = p.second;

    std::cerr << "axis = " << axis << std::endl;
    std::cerr << "theta = " << theta * 180.0/M_PI << std::endl;


    float t=0;
    while (ros::ok ())
    {
        t+=1.0/100.0;

        // 変換前後の座標系
        rp.setAxis("world", "S0", posed(S0_p, S0_q));
        rp.setAxis("world", "S1", posed(S1_p, S1_q));

        // 変換量生成
        double ratio = 0.5*(1+sin(-M_PI/2 + t));
        vec4d rot(axis, theta*ratio);

        // 変換中の座標系
        // S0から変換するのでS0*rot
        rp.setAxis("world", "S2", posed(S0_p, S0_q*rot));

        vec4d rot2 = S0_q.slerp(S1_q, ratio);
        rp.setAxis("world", "S3", posed(S0_p, S0_q*rot2));



        rate.sleep ();
    }
    return 0;
}
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
    PathMarker pm("test_path",300);
    ros::Rate rate (100);

    // 座標系p1とp2を生成
    vec3d rpy = vec3d(12.345,23.456,34.567)*M_PI/180;
    posed p1(vec3d(0,0,0), vec4d(rpy.x,rpy.y,rpy.z));
//    posed p1;
    posed p2(vec3d(0.2,0,0), vec4d(0,M_PI/2,0));

    // p2からp1への変換量p12の演算(p2座標系)
    auto p12 = p1/p2;
    std::cerr << "p12= \n" << p12 << std::endl;

    // 変換量p12を用いてp1（p1_）を生成
    auto p1_ = p2*p12;
    std::cerr << "p1_= \n" << p1_ << std::endl;
    std::cerr << (p1==p1_) << std::endl;

    // p1からp2への変換量p21の演算
    auto p21 = p2/p1;
    std::cerr << "p21= \n" << p21 << std::endl;

    // 変換量p21を用いてp2（p2_）を生成
    auto p2_ = p1*p21;
    std::cerr << "p2_= \n" << p2_ << std::endl;
    std::cerr << (p2==p2_) << std::endl;


    double T=0;
    while(ros::ok())
    {
        T+=0.01;
        auto ma = InitMarker();
        
        // 座標系を回転
        p1 = p1.rotate(2, 0.01);
        p2 = p2.rotate(2, 0.01);

        rp.setAxis("world","p1", p1);
        rp.setAxis("world","p2", p2);

        // 位置座標を2つの座標系表現で表示
        vec3d pnt1_p1 = vec3d(0.1,0,0);
        vec3d pnt1_p2 = p1.Trans_pnt(pnt1_p1, p2);
        ArrowMarker(ma, "p1", vec3d(0,0,0), pnt1_p1, ColorRGBA(1,0,0,0.5), "pnt1_p1");
        ArrowMarker(ma, "p2", vec3d(0,0,0), pnt1_p2, ColorRGBA(0,1,0,0.5), "pnt1_p2");

        // ベクトルを2つの座標系表現で表示
        vec3d vec1_p2 = p1.Trans_vec(pnt1_p1, p2);
        ArrowMarker(ma, "p2", vec3d(0,0,0), vec1_p2, ColorRGBA(0,0,1,0.5), "vec1_p2");

        // p1をp2座標系周りで回転(変換後p1座標系原点を表示)
        auto p1_tmp = p1.rotate(1,30*M_PI/180*sin(0.5*T), &p2);
        rp.setAxis("world","p1_tmp", p1_tmp);
        pm.append(p1_tmp.p);

        rp.publish(ma);
        rate.sleep ();
    }

    return 0;
}
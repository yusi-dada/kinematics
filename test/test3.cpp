#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>

#include "ros_marker.h"
#include "camera.h"
using namespace kinematics;
using namespace ros_func;
typedef pose<double> posed;
typedef vec4<double> vec4d;
typedef vec3<double> vec3d;

std::vector<posed> jnt2pos_array(double jnt[])
{
    std::vector<posed> ret;
    int N = 6;
    std::vector<vec3d> pos, alfa;
    pos.resize(N);
    alfa.resize(N);

    pos[0] = vec3d(0,0,0.295);
    pos[1] = vec3d(0,0.0797,0);
    pos[2] = vec3d(0,-0.0367,0.230);
    pos[3] = vec3d(-0.050,-0.043,0.0725);
    pos[4] = vec3d(0,0,0.1975);
    pos[5] = vec3d(0,0,0.07);

    alfa[0] = vec3d(0,0,1);
    alfa[1] = vec3d(0,1,0);
    alfa[2] = vec3d(0,1,0);
    alfa[3] = vec3d(0,0,1);
    alfa[4] = vec3d(0,1,0);
    alfa[5] = vec3d(0,0,1);

    posed posB;
    posed posI;
    ret.push_back(posI);
    for(int i=0;i<N;i++)
    {
        posB.p = pos[i];
        posB.q = vec4d(alfa[i], jnt[i]);
        posI = posI * posB;
        ret.push_back(posI);
    }
    return ret;

}

// pose クラステスト

int main(int argc, char **argv)
{
    // init ROS node
    ros::init (argc, argv, "test3");
    rviz_publisher rp("test");
    ros::Rate rate (100);

    std::cerr << "INFINITY>0 : " << (-INFINITY>0) << std::endl;
    double jnt[6]={0,0,M_PI/2,0,M_PI/2,0};

    float t=0;
    while (ros::ok ())
    {
        t+=1.0/100.0;
        //jnt[0] = jnt[0]+0.001;
        //jnt[1] = jnt[1]+0.001;
        //jnt[2] = jnt[2]+0.001;
        std::vector<posed> pos_array = jnt2pos_array(jnt);
        for(int i=0;i<pos_array.size();i++)
        {
            rp.setAxis("world", "link"+std::to_string(i), pos_array[i]);
        }

        auto ma = InitMarker();

        // 各座標系原点を白点で表示
        std::vector<vec3d> pos;
        for(auto p:pos_array) pos.push_back(p.p);
        PointMarker(ma, "world", pos, ColorRGBA(1,1,1,1), "link");
        LineChainMarker(ma, "world", pos, ColorRGBA(1,1,0,1), "arrow");

        // link5からの相対位置(tmp)をlink1からの相対位置に変換
        // link5を基準とした点(赤)とlink1を基準とした点(緑)で表示
//        vec3d tmp(0.1, 0, 0);
        vec3d tmp(cos(t), sin(t), 0);
        tmp = tmp*0.1;
        PointMarker(ma, "link5", tmp, ColorRGBA(1,0,0,1), "test1-1");
        PointMarker(ma, "link1", pos_array[5].Trans_pnt(tmp, pos_array[1]), ColorRGBA(0,1,0,1), "test1-2");

        // worldからの相対位置(tmp)をlink5からの相対位置に変換
        // worldを基準とした点(赤)とlink5を基準とした点(青)で表示
        PointMarker(ma, "world", tmp, ColorRGBA(1,0,0,1), "test2-1");
        PointMarker(ma, "link5", posed().Trans_pnt(tmp, pos_array[5]), ColorRGBA(0,0,1,1), "test2-2");

        // world原点と位置(tmp)の相対関係をlink5原点に平行移動
        // 相対位置をlink5の座標系表現へ変換
        ArrowMarker(ma, "world", vec3d(0,0,0), tmp, ColorRGBA(1,0,1,1), "test3");
        ArrowMarker(ma, "link5", vec3d(0,0,0), posed().Trans_vec(tmp, pos_array[5]), ColorRGBA(1,0,1,1), "test3");

        // link5原点と位置(tmp)の相対関係をlink1原点に平行移動
        // 相対位置をlink1の座標系表現へ変換
        ArrowMarker(ma, "link5", vec3d(0,0,0), tmp, ColorRGBA(0,0,1,1), "test4");
        ArrowMarker(ma, "link1", vec3d(0,0,0), pos_array[5].Trans_vec(tmp, pos_array[1]), ColorRGBA(0,0,1,1), "test4");


/*
        std::vector<vec3d> pos1;
        pos1.push_back(vec3d(0,0,1));
        pos1.push_back(vec3d(0,1,1));
        pos1.push_back(vec3d(1,1,1));
        pos1.push_back(vec3d(1,0,1));
        PyramidMarker(ma, "world", vec3d(), pos1, ColorRGBA(1,1,0,1), "arrow2");
*/


        posed pos1;
        pos1.p.x = 1.0;
        rp.setAxis("world", "test1", pos1);
        rp.setAxis("world", "test2", pos1.rotate(1, M_PI/3*sin(t)));
        //CameraMarker(ma, "world", pos1, tan(20*M_PI/180.0), tan(10*M_PI/180.0), ColorRGBA(1,1,1,1), "camera");

        posed pos2;
        pos2.p.x = 1.0;
        pos2.p.y = 0.3;
        pos2.p.z = 1.0;
        pos2 = pos2 * posed(vec3d(), vec4d(vec3d(cos(t),sin(t),1), M_PI/180*10));
        rp.setAxis("world", "test3", pos2);
//        ImageSurfaceMarker(ma, "world", pos1, pos2, tan(20*M_PI/180.0), tan(10*M_PI/180.0), ColorRGBA(1,1,1,1), "surface");

        vec3d ray(0,0,1);
        double n = pos1.projection(ray, pos2, vec3d(0,0,1));
        if(!isinf(n))
            PointMarker(ma, "test2", n*ray, ColorRGBA(1,1,1,1), "ray-point");

        CubeMarker(ma, "world", pos2, vec3d(1.1,1.1,0.0), ColorRGBA(1,0,0,0.1), "test5");
        CubeWireMarker(ma, "world", pos2, vec3d(1.1,1.1,0.0), ColorRGBA(1,0,0,1), "test5");
        rp.publish(ma);

        rate.sleep ();
    }


    return 0;
}
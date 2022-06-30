#include "ros_marker.h"
#include "camera.h"
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

    // カメラ設定
    posed flange2cam(vec3d(-0.05,0,0), vec4d(0, 10*M_PI/180, 0));
    double cameraYaw = -M_PI/2;
    double tanH = tan(20*M_PI/180.0);
    double tanV = tan(10*M_PI/180.0);

    // フランジ初期位置
    posed flange(vec3d(0.02, 0.05, 0.2), vec4d(M_PI*1.01,0,0));

    // 床面
    posed surf1;

    double T=0;
    while(ros::ok())
    {
        T+=0.01;
        auto ma = InitMarker();

        // フランジ移動
        flange.p.z = 0.2+0.1*sin(T);

        // フランジ
        rp.setAxis("world", "flange", flange);
        
        // カメラ生成
        camera<double> cam1(tanH, tanV, flange*flange2cam, cameraYaw);
        rp.setAxis("world", "cam1", cam1.p0);
        CameraMarker(ma, "world", cam1, ColorRGBA(1,1,1,1), "cam1");
        ImageSurfaceMarker(ma, "world", cam1, surf1, ColorRGBA(1,1,1,1), "surf1");

        // ハンド平面
        auto hand = flange*posed(vec3d(0,0,0.1), vec4d());
        rp.setAxis("world", "hand", hand);        
        auto surf_hand = hand.surface(vec3d(0.0,0,1));
        ImageSurfaceMarker(ma, "world", cam1, surf_hand, ColorRGBA(0,1,1,1), "surf_hand");

        // world座標系から
        vec3d ray(0,0,0);
        ArrowMarker(ma,"world", cam1.p0.p, ray, ColorRGBA(1,1,1,0.5), "ray_world");
        if (cam1.pos2image(ray))
        {
            if (cam1.image2camera(ray))
                ArrowMarker(ma,"cam1", vec3d(0,0,0), ray, ColorRGBA(0,1,0,0.5), "ray_cam1");
        }

        // 画像比率から
        vec3d ray2(0.5,0.5,0);
        if (cam1.image2pos(ray2, surf1))
        {
            ArrowMarker(ma,"world", cam1.p0.p, ray2, ColorRGBA(1,1,1,0.5), "ray2_world");

            // カメラ座標系へ変換
            if (cam1.pos2image(ray2))
            {
                if (cam1.image2camera(ray2))
                    ArrowMarker(ma,"cam1", vec3d(0,0,0), ray2, ColorRGBA(0,0,1,0.5), "ray2_cam1");
            }

        }


        rp.publish(ma);
        rate.sleep ();
    }

    return 0;
}
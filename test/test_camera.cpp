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
    double tanV = tan(15*M_PI/180.0);

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
        //flange.p.z = 0.2+0.1*sin(T);

        // フランジ
        rp.setAxis("world", "flange", flange);

        // カメラ生成
        camera<double> cam1(tanH, tanV, flange*flange2cam, cameraYaw);
        rp.setAxis("world", "cam1", cam1.p0);
        CameraMarker(ma, "world", cam1, ColorRGBA(1,1,1,1), "cam1");
        ImageSurfaceMarker(ma, "world", cam1, surf1, ColorRGBA(1,1,0,1), "surf1");

        // ハンド平面
        auto hand = flange*posed(vec3d(0,0,0.1), vec4d(0,0,0));
        rp.setAxis("world", "hand", hand);        
        auto surf_hand = hand.surface(vec3d(0.0,-0.2,1));
        ImageSurfaceMarker(ma, "world", cam1, surf_hand, ColorRGBA(0,1,1,1), "surf_hand");


        // 画面上の２点から座標系を生成(ハンド)
        std::vector<vec3d> ray3(2);
        ray3[1] = vec3d(0.5,0.5,0);
        ray3[0] = vec3d(0.1,0.2,0);
//        ray3[1] = 0.5*(vec3d(cos(T),sin(T),0)+1);
        posed c1;
        if(cam1.coordinate_by_2(ray3[0],ray3[1],surf_hand,c1))
        {
            std::cerr <<"surf= "<<surf_hand << std::endl;
            std::cerr <<"cam1= "<<cam1.p0 << std::endl;
            std::cerr <<"c1= "<<c1 << std::endl;
            rp.setAxis("world", "hand_coor", c1);        
            if (cam1.image2pos(ray3, surf_hand))
                LineMarker(ma, "world", ray3[0], ray3[1], ColorRGBA(1,1,1,0.5), "line");
            ArrowMarker(ma,"world", cam1.p0.p, c1.p, ColorRGBA(0,1,1,0.5), "hand");
        }
        else
            assert(false);


        // 基準座標系の位置から画面座標系を生成(ターゲット)
        std::vector<vec3d> ray4(2);
//        ray4[0] = vec3d(-9.0423e-03, +7.0935e-02, +0.0000e+00);
//        ray4[1] = vec3d(+3.5756e-02, +1.1757e-01, +2.7756e-17);
        ray4[0] = vec3d(0.01, 0.05, 0);
        ray4[1] = vec3d(0, 0.06, 0);
        posed c2;
        if(cam1.pos2image(ray4))
        {
            // 画面上の２点から座標系を生成
            cam1.coordinate_by_2(ray4[0],ray4[1],surf1,c2);
            rp.setAxis("world", "target_coor", c2);        
            if (cam1.image2pos(ray4, surf1))
                LineMarker(ma, "world", ray4[0], ray4[1], ColorRGBA(1,1,1,0.5), "line");
            ArrowMarker(ma,"world", cam1.p0.p, c2.p, ColorRGBA(1,1,0,0.5), "target");


            // 2つの座標系を一致させる補正量を演算           
            auto tmp = c1.q.RotationTo(c2.q);
            auto axis = tmp.first;
            auto theta = tmp.second;
            std::cerr <<"theta= "<<theta << std::endl;
            if(theta!=0)
            {
                ArrowMarker(ma,"hand_coor", vec3d(), axis, ColorRGBA(1,1,0,0.5), "rotate");
                theta = std::min(std::max(theta,-0.01),0.01);
                flange = flange.rotate(axis, theta, &c1);
            }

            assert(flange.isnum());
        }
        else
            assert(false);


        rp.publish(ma);
        rate.sleep ();
    }

    return 0;
}
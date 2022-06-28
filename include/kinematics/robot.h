#pragma once
#include <array>
#include "camera.h"
#include "pose_array.h"
#include "ros_marker.h"
using namespace ros_func;

#define Naxis (6)

namespace kinematics
{

inline std::vector<vec3<double>> posB()
{
    std::vector<vec3<double>> pos(Naxis);
    pos[0] = vec3<double>(0,0,0.295);
    pos[1] = vec3<double>(0,0.0797,0);
    pos[2] = vec3<double>(0,-0.0367,0.230);
    pos[3] = vec3<double>(-0.050,-0.043,0.0725);
    pos[4] = vec3<double>(0,0,0.1975);
    pos[5] = vec3<double>(0,0,0.07);
    return pos;
}

inline std::vector<vec3<double>> alfa()
{
    std::vector<vec3<double>> alfa(Naxis);
    alfa[0] = vec3<double>(0,0,1);
    alfa[1] = vec3<double>(0,1,0);
    alfa[2] = vec3<double>(0,1,0);
    alfa[3] = vec3<double>(0,0,1);
    alfa[4] = vec3<double>(0,1,0);
    alfa[5] = vec3<double>(0,0,1);
    return alfa;
}

/**
 * @brief 関節角からリンク生成
 * @param [in] jnt 関節角
 * @param [in] base ベース姿勢
 */
template <typename T>
pose<T> jnt2pos_array(std::array<T, Naxis> jnt, pose<T> base=pose<T>())
{
    return pose_array<T>(posB(), alfa(), jnt, base);   // リンク生成
}


template <typename T>
class robot
{

    public:
        pose_array<T> links;
        pose<T> *flange2hand;
        
        bool handeye;
        pose<T> *ref2cam;
        T cameraYaw;
        T tanH;
        T tanV;

        camera<T> cam;

        pose<T> hand_surf;
        std::vector<vec3<T>> hand_points;
        pose<T> target_surf;
        std::vector<vec3<T>> target_points;

        /**
         * @brief 関節角からリンク生成
         * @param [in] jnt 関節角
         * @param [in] base ベース姿勢
         */
        pose<T> jnt2pos(std::array<T, Naxis> jnt, pose<T> base=pose<T>())
        {
            links = pose_array<T>(posB(), alfa(), jnt, base);   // リンク生成
            if(flange2hand) links << *flange2hand;              // 相対変換によるリンク伸長
            return links.pa.back();
        }

        /**
         * @brief 姿勢から関節角を生成
         * @param [in] pos 姿勢
         */
        std::array<T, Naxis> pos2jnt(pose<T> pos)
        {

        }

        robot(T jnt[])
        {
            flange2hand = nullptr;

            handeye = false;
            ref2cam = nullptr;

            *flange2hand = pose<T>(vec3<T>(0,0,0.1),vec4<T>(M_PI/2,0,0));    // ハンド設置
            *ref2cam  = pose<T>(vec3<T>(-0.07,0,0),vec4<T>(0,M_PI/18,0));  // カメラモデル生成

            T cameraYaw = -M_PI/2;
            T tanH = tan(20*M_PI/180.0);
            T tanV = tan(10*M_PI/180.0);


            auto hand = links[-1];                                  // ハンド座標系
            auto flange = links[-2];                                // フランジ座標系

            // カメラモデル生成
            cam = camera<T>(tanH, tanV, flange*(*ref2cam), cameraYaw);

            // 制御平面設定
            hand_surf   = hand.surface(vec3<T>(1,1,0), M_PI);       // ハンド平面
            target_surf = pose<T>();                                // 目標平面
        }

        bool command(std::vector<vec3<T>> hand_points_, std::vector<vec3<T>> target_points_)
        {
//            cam.image2pos(hand_points_, hand_surf);
 //           cam.image2pos(target_points_, target_surf);
        }


        void rviz(rviz_publisher rp)
        {
            auto ma = InitMarker();

            // リンク構造描画
            rp.setAxis("world", "link", links.pa);  // 座標系配信

            auto edge_points = links.p();                                           // 各座標系原点取得
            LineChainMarker(ma, "world", edge_points, ColorRGBA(1,1,0,1), "link");  // リンク描画
            PointMarker(ma, "world", edge_points, ColorRGBA(1,1,1,1), "joint");     // 座標系原点描画

            // カメラモデル生成
            rp.setAxis("world", "camera", cam.p0);  // 座標系配信
            CameraMarker(ma, "world", cam, ColorRGBA(0,1,1,1), "camera");

            // ハンド平面
            rp.setAxis("world", "hand_surf", hand_surf);  // 座標系配信
            ImageSurfaceMarker(ma, "world", cam, hand_surf, ColorRGBA(0,0,1,1), "hand_surf");
            
            // 平面モデル
            ImageSurfaceMarker(ma, "world", cam, target_surf, ColorRGBA(0,1,0,1), "target_surface");

            rp.publish(ma);

        }
    

};

}
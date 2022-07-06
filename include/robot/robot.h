#pragma once
#include <robot/joint.h>    // kinematics
#include <robot/fpose.h>    // kinematics
#include <robot/bit.h>

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
 * @brief ジョイント関節から姿勢生成(基準座標)
 * @param [in] posI ベース姿勢
 */
template <typename T>
std::vector<pose<T>> to_pose_array(joint<T> jnt, pose<T> posI=pose<T>())
{
    std::vector<vec3<T>> pos = posB();
    std::vector<vec3<T>> alfa = alfa();

    int linksize = jnt->val.size();
    std::vector<pose<T>> pa(linksize+1);
    pa[0] = posI;            
    for(int i=0; i<linksize; i++)
        pa[i+1] = pa[i] * pose<T>(pos[i], vec4<T>(alfa[i], jnt[i]));
    return pa;
}

/**
 * @brief ジョイント関節から手先姿勢生成(基準座標)
 * @param [in] posI ベース姿勢
 */
template <typename T>
pose<T> to_pose(joint<T> jnt, pose<T> posI=pose<T>())
{
    std::vector<pose<T>> pa = to_pose_array(jnt, posI);
    return pa.back();
}

/**
 * @brief 手先姿勢からジョイント関節生成
 * @param [in] posI ベース姿勢
 */
template <typename T>
joint<T> to_joint(pose<T> pos, pose<T> posI=pose<T>())
{
    // 手先姿勢の表現を基準座標からベース座標に変換
    pose<T> pos_from_base = pos/posI;
}

}
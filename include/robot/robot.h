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

inline std::vector<vec3<double>> alfaB()
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
 * @brief 角度制約チェック
 * @retval true 制約外
 * @retval false 制約内
 */
template <typename T>
bool LimChk(const joint<T> jnt)
{
    return false;
}

/**
 * @brief フラグ生成
 */
template <typename T>
SFLG FlgChk(joint<T> jnt)
{
    SFLG ret;
    int flg[8] = {0};
    for(int i=0; i<6; i++)
    {
        T Lmin = M_PI;
        while(abs(jnt[i]) > Lmin)
        {
            flg[i] = (jnt[i]>0) ? (flg[i]+1) : (flg[i]-1);
            Lmin += 2.0*M_PI;

            if(Lmin > 10.0*M_PI)
            {
                std::cerr << "[FlgChk] out of range." << std::endl;
                assert(false);
            }
        }
    }
    for(int i=0; i<6; i++)
        ret.flg2.SET(i, flg[i]);

    std::vector<vec3<double>> pos = posB();
    T D2  = abs(pos[3].x);
    T D1  = abs(pos[2].x);
    T L2  = abs(pos[2].x);
    T L3  = abs(pos[3].z + pos[4].z);
    T C23 = cos(jnt[1] + jnt[2]);
    T S23 = sin(jnt[1] + jnt[2]);
    T S2  = cos(jnt[1]);
    T M41 = -D2*C23+L3*S23+L2*S2+D1;
    ret.flg1.bit.RL = (M41>=0) ? (1) : (0);
    ret.flg1.bit.AB = (jnt[2]>=atan2(D2,L3)) ? (1) : (0);
    ret.flg1.bit.NF = (jnt[4]>=0) ? (1) : (0);

    return ret;
}




/**
 * @brief ジョイント関節から姿勢生成(基準座標)
 * @param [in] posI ベース姿勢
 */
template <typename T>
std::vector<fpose<T>> to_pose_array(joint<T> jnt, pose<T> posI=pose<T>())
{
    std::vector<vec3<T>> pos = posB();
    std::vector<vec3<T>> alfa = alfaB();

    int linksize = jnt.val.size();
    std::vector<fpose<T>> pa(linksize+1);
    pa[0] = posI;
    for(int i=0; i<linksize; i++)
        pa[i+1] = pa[i] * fpose<T>(pos[i], vec4<T>(alfa[i], jnt[i]));

    SFLG flg = FlgChk(jnt);
    pa[linksize].flg1 = flg.flg1;
    pa[linksize].flg2 = flg.flg2;    
    return pa;
}

/**
 * @brief ジョイント関節から手先姿勢生成(基準座標)
 * @param [in] posI ベース姿勢
 */
template <typename T>
fpose<T> to_pose(joint<T> jnt, pose<T> posI=pose<T>())
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
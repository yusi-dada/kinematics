#pragma once
#include "pose_array.h"
#include <array>
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


template <typename T>
pose_array<T> jnt2pos_array(std::array<T, Naxis> jnt, pose<T> base=pose<T>())
{
    return pose_array<T>(posB(), alfa(), jnt, base);       // リンク生成
}

template <typename T>
std::array<T, Naxis> pos2jnt(pose<T> pos)
{

}

}
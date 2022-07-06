/**
 * @file fpose.h
 * @brief フラグ付き座標系クラス
 */
#pragma once
#include <kinematics/kinematics.h>
#include <robot/bit.h>

namespace kinematics
{
/**
 * @brief フラグ付き座標系クラス
 */
template <typename T>
class fpose : public pose<T>
{
    public:
        using pose<T>::pose;
};

}
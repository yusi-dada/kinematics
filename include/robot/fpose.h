/**
 * @file fpose.h
 * @brief フラグ付き座標系クラス
 */
#pragma once
#include <kinematics/kinematics.h>
#include <robot/sflg.h>
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
        SFLG1 flg1;
        SFLG2 flg2;

        fpose() : pose<T>()
        {
            flg1.val = flg2.val = 0;
        }

        fpose(const vec3<T> p_, const vec4<T> q_, unsigned int _flg1=0, unsigned int _flg2=0)
         : pose<T>(p_, q_)
        {
            flg1.val = _flg1;
            flg2.val = _flg2;
        }

        fpose<T> operator=(const pose<T>& obj)
        {
            this->p = obj.p;
            this->q = obj.q;
            this->flg1.val = this->flg2.val = 0;
            return(*this);
        }

        fpose<T> operator=(const fpose<T>& obj)
        {
            this->p    = obj.p;
            this->q    = obj.q;
            this->flg1 = obj.flg1;
            this->flg2 = obj.flg2;
            return(*this);
        }        
};

template <typename T>
std::ostream& operator<<(std::ostream& stream, fpose<T>& obj)
{
    char cData[512];
    sprintf(cData,"  flg = (%d, %d)", obj.flg1.val, obj.flg2.val);
    pose<T> tmp(obj.p, obj.q);
    return( stream << tmp << cData);
}

template <typename T>
std::vector<pose<T>> to_pose_array(std::vector<fpose<T>> obj)
{
    std::vector<pose<T>> ret(obj.size());
    for(int i=0; i<obj.size(); i++)
    {
        ret[i].p = obj[i].p;
        ret[i].q = obj[i].q;
    }
    return ret;

}

}
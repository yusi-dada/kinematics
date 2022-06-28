#pragma once
#include "pose.h"
#include <memory>

namespace kinematics
{


template <typename T>
class pose_array
{
    public:
        std::vector<pose<T>> pa;

        pose_array(){}

        pose_array(std::vector<vec3<T>> pos, std::vector<vec3<T>> alfa, T theta[], pose<T> posI=pose<T>())
        {
            (*this)(pos, alfa, theta, posI);
        }

        /**
         * @brief ベース座標系から相対変換によるリンク構造生成
         * @param [in] pos 相対位置
         * @param [in] alfa リンク回転軸
         * @param [in] theta 回転角
         * @param [in] posI ベース座標系
         */
        void operator()(std::vector<vec3<T>> pos, std::vector<vec3<T>> alfa, T theta[], pose<T> posI=pose<T>())
        {
            // 入力サイズ確認
            int linksize = pos.size();
            assert((linksize==alfa.size()) && (linksize>0));

            // 配列初期化
            if (pa.size() != linksize+1)
                pa.resize(linksize+1);

            pa[0] = posI;            
            for(int i=0; i<linksize; i++)
                pa[i+1] = pa[i] * pose<T>(pos[i], vec4<T>(alfa[i], theta[i]));
        }

        /**
         * @brief 代入
         */
        pose_array operator=(const pose_array<T>& obj)
        {
            this->pa = obj.pa;
            return (*this);
        }

        /**
         * @brief インデックスによるリンク取得（python方式）
         */
        pose<T> operator[](int n)
        {
            int pa_size = this->pa.size();
            n = (n>=0) ? (n) : (pa_size+n);           
            assert(0<=n && n<pa_size);
            return this->pa[n];
        }

        /**
         * @brief 座標系原点位置を抽出
         */
        std::vector<vec3<T>> p()
        {
            int N = pa.size();
            std::vector<vec3<T>> ret(N);
            for(int i=0; i<N; i++)
                ret[i]=pa[i].p;
            return ret;            
        }

        /**
         * @brief 座標系姿勢を抽出
         */
        std::vector<vec4<T>> q()
        {
            int N = pa.size();
            std::vector<vec4<T>> ret(N);
            for(int i=0; i<N; i++)
                ret[i]=pa[i].q;
            return ret;            
        }
};

/**
 * @brief 相対変換によるリンク伸長
 */
template <typename T>
pose_array<T>& operator<<(pose_array<T>& obj1, const pose<T>& obj2)
{
    pose<T> tmp = obj1.pa.back();
    obj1.pa.push_back(tmp*obj2);
    return (obj1);
}

template <typename T>
std::ostream& operator<<(std::ostream& stream, pose_array<T>& obj)
{
    char cData[512];
    for (int i=0; i<obj.pa.size(); i++)
    {
        sprintf(cData, "<link%d>\n",i);
        stream << cData << obj.pa[i] << std::endl;
    }
    return( stream );
}

}
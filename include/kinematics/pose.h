/**
 * @file pose.h
 * @brief 座標系クラス
 */
#pragma once
#include "vec4.h"

namespace kinematics
{

template <typename T>
class pose
{
    public:
        vec3<T> p;
        vec4<T> q;

        pose()
        {
            this->p = vec3<T>(0,0,0);
            this->q = vec4<T>(0,0,0,1);
        }

        pose(const vec3<T> p_, const vec4<T> q_)
        {
            this->p = p_;
            this->q = q_;
        }

        /**
         * @brief 代入
         */
        pose<T> operator=(const pose<T>& obj)
        {
            this->p = obj.p;
            this->q = obj.q;
            return(*this);
        }

        /**
         * @brief 指定軸で回転させて姿勢変更
         * @param axis 回転軸(this座標系)
         * @param angle 回転角[rad]
         */
        pose<T> rotate(int axis, T angle)
        {
            assert(0<=axis && axis<=2);
            vec3<T> alfa;
            T *p = &alfa.x;
            p[axis] = 1;
            this->q = this->q * vec4<T>(alfa, angle);
            return(*this);
        }

        /**
         * @brief 相対姿勢で座標系変換
         * @details 結合規則は左から右へ
         * @param [in] obj 相対姿勢(this座標系表現)
         */
        pose<T> operator*(const pose<T>& obj)
        {
            pose<T> ret(this->p, this->q);
            ret.p = ret.p + ret.q.Trans(obj.p);
            ret.q = ret.q * obj.q;
            return ret;
        }

        /**
         * @brief 2座標系間の相対姿勢取得
         * @details 入力姿勢(obj)から相対姿勢(ret)をかけたときに現在姿勢(this)となる(this=obj*ret)
         * @param [in] obj 入力姿勢
         * @return 相対姿勢(obj座標系表現)
         */
        pose<T> operator/(pose<T>& obj)
        {
            pose<T> ret(this->p, this->q);
            ret.q = obj.q.conj()*ret.q;
            ret.p = obj.q.conj().Trans(ret.p-obj.p);
            return ret;
        }

        /**
         * @brief 有効判定
         */
        bool isnum()
        {
            return(this->p.isnum() && this->q.isnum());
        }

        /**
         * @brief 点の座標系表現(pnt)を別座標系(obj)表現へ変換
         * @param pnt [in] 3次元点位置(this座標系)
         * @param obj [in] 変換先座標系
         * @return 変換後3次元点位置(obj座標系)
         */
        vec3<T> Trans_pnt(const vec3<T>& pnt, const pose<T>& obj=pose<T>())
        {
            // クラス座標系表現(pnt)を基準座標系表現(ret)に変換
            vec3<T> ret = this->p + this->q.Trans(pnt); 

            // 基準座標系表現(ret)を別座標系(obj)表現に変換
            pose<T> tmp = obj;
            return tmp.q.conj().Trans(ret - tmp.p);
        }

        /**
         * @brief ベクトルの座標系表現(pnt)を別座標系(obj)表現へ変換
         * @param pnt [in] 3次元点位置(this座標系)
         * @param obj [in] 変換先座標系
         * @return 変換後ベクトル(obj座標系)
         */
        vec3<T> Trans_vec(const vec3<T>& pnt, const pose<T>& obj=pose<T>())
        {
            // クラス座標系表現(pnt)を基準座標系表現(ret)に変換
            vec3<T> ret = this->q.Trans(pnt); 

            // 基準座標系表現(ret)を別座標系(obj)表現に変換
            pose<T> tmp = obj;
            return tmp.q.conj().Trans(ret);
        }

        /**
         * @brief 法線方向指定による平面オブジェクト生成
         * @brief [in] normal 法線ベクトル（this座標系）
         * @brief [in] yaw 法線ベクトルを合わせたあとのZ軸回転[rad]
         */
        pose<T> surface(vec3<T> normal, T yaw=0)
        {
            T nrm = normal.nrm();
            assert(nrm > 1e-9);
            vec3<T> axisZ = vec3<T>(0,0,1);
            vec3<T> alfa = axisZ % normal;      // 回転軸
            T theta = acos(axisZ*normal/nrm);   // 回転角
            pose<T> ret = (*this);
            ret.q = ret.q * vec4<T>(alfa,theta);
            if(yaw!=0) ret.q = ret.q * vec4<T>(0,0,yaw);
            return ret;
        }

        /**
         * @brief ベクトルの平面への写像
         * @param ray [in,out] 射影ベクトル
         * @param surf [in] 平面座標系
         * @param normal [in] 平面の法線指定（平面座標系表現） default:z軸
         * @return 射影ベクトルの延伸倍率
         */
        double projection(vec3<T> ray, pose<T> surf, vec3<T> normal)
        {
            // 基準座標表現に変換
            vec3<T> N = surf.Trans_vec(normal);
            ray = this->Trans_vec(ray);

            double tmp = ray*N;
            if (abs(tmp) <= 1e-6) return INFINITY;
            return (surf.p - this->p)*N / tmp;
        }

        /**
         * @brief 一致判定
         */
        bool operator==(const pose<T>& obj)
        {
            return (this->p == obj.p)&&(this->q == obj.q);
        }

};

template <typename T>
std::ostream& operator<<(std::ostream& stream, pose<T>& obj)
{
    return( stream << "pos[ m ] = "<< obj.p << "\nrpy[deg] = " << obj.q.rpy()*180.0/M_PI );
}


}
/**
 * @file pose.h
 * @brief 座標系クラス
 */
#pragma once
#include <kinematics/vec4.h>

namespace kinematics
{

/**
 * @brief 座標系クラス
 */
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
         * @brief 要素アクセス
         */
        T& operator[](int n)
        {
            switch (n)
            {
                case 0: return(this->p.x);
                case 1: return(this->p.y);
                case 2: return(this->p.z);
                case 3:
                case 4:
                case 5:
                {
                    vec3<T> rpy = this->q.rpy();
                    return rpy[n-3];
                }
                default:
                {
                    std::cerr << "[pose] wrong index" << std::endl;
                    assert(false);
                }
            }
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
         * @brief 相対姿勢で座標系変換
         * @details 結合規則は左から右へ
         * @param [in] obj 相対姿勢(this座標系表現)
         */
        pose<T> operator*(const pose<T>& obj)
        {
            pose<T> ret(this->p, this->q);
            ret.p = ret.p + ret.q.Trans(obj.p, true);
            ret.q = ret.q * obj.q;
            return ret;
        }

        /**
         * @brief 2座標系間の相対姿勢取得
         * @details 現在姿勢(this)は入力姿勢(obj)から相対姿勢(ret)をかける(this=obj*ret)
         * @param [in] obj 入力姿勢
         * @return 相対姿勢(obj座標系表現)
         */
        pose<T> operator/(const pose<T>& obj)
        {
            pose<T> ret(this->p, this->q);
            ret.q = obj.q.conj()*ret.q;
            ret.p = obj.q.conj().Trans(ret.p-obj.p, true);
            return ret;
        }

        /**
         * @brief (this/p座標系の)回転軸で回転させて姿勢変更
         * @param axis 回転軸(this座標系 or p座標系)
         * @param angle 回転角[rad]
         * @param pt 座標系
         * @return 回転処理後のthis座標系
         */
        pose<T> rotate(vec3<T> alfa, T angle, pose<T> *pt = nullptr)
        {
            if(pt)   // 基準座標系の指定がある
            {
                pose<T> base = (*pt);
                base.q.normalize();
                pose<T> tmp = (*this)/base; // 回転前での相対姿勢取得
                base.q = base.q * vec4<T>(alfa, angle);
                return base*tmp;
            }
            else    // 基準座標系の指定がない
            {
                pose<T> base = (*this);
                base.q = base.q * vec4<T>(alfa, angle);
                return base;
            }
        }

        /**
         * @brief (this/p座標系の)指定軸で回転させて姿勢変更
         * @param axis 回転軸(this座標系 or p座標系)
         * @param angle 回転角[rad]
         * @param p 座標系
         * @return 回転処理後のthis座標系
         */
        pose<T> rotate(int axis, T angle, pose<T> *p = nullptr)
        {
            // 回転軸生成（回転基準となる座標系表現）
            assert(0<=axis && axis<=2);
            vec3<T> alfa;
            T *alfap = &alfa.x;
            alfap[axis] = 1;
            return rotate(alfa, angle, p);
        }

        /**
         * @brief 有効判定
         */
        bool isnum()
        {
            return(this->p.isnum() && this->q.isnum());
        }

        /**
         * @brief 一致判定
         */
        bool operator==(const pose<T>& obj)
        {
            return (this->p == obj.p)&&(this->q == obj.q);
        }

        /**
         * @brief 点の座標系表現(pnt)を別座標系(obj)表現へ変換
         * @param pnt [in] 3次元点位置(this座標系)
         * @param obj [in] 変換先座標系
         * @param [in] normalize 正規化フラグ
         * @return 変換後3次元点位置(obj座標系)
         */
        vec3<T> Trans_pnt(const vec3<T>& pnt, const pose<T>& obj=pose<T>(), bool normalize=true)
        {
            // クラス座標系表現(pnt)を基準座標系表現(ret)に変換
            vec3<T> ret = this->p + this->q.Trans(pnt, normalize); 

            // 基準座標系表現(ret)を別座標系(obj)表現に変換
            pose<T> tmp = obj;
            return tmp.q.conj().Trans(ret - tmp.p, normalize);
        }

        /**
         * @brief ベクトルの座標系表現(pnt)を別座標系(obj)表現へ変換
         * @param [in] pnt 3次元点位置(this座標系)
         * @param [in] obj 変換先座標系
         * @param [in] normalize 正規化フラグ
         * @return 変換後ベクトル(obj座標系)
         */
        vec3<T> Trans_vec(const vec3<T>& pnt, const pose<T>& obj=pose<T>(), bool normalize=true)
        {
            // クラス座標系表現(pnt)を基準座標系表現(ret)に変換
            vec3<T> ret = this->q.Trans(pnt, normalize); 

            // 基準座標系表現(ret)を別座標系(obj)表現に変換
            pose<T> tmp = obj;
            return tmp.q.conj().Trans(ret, normalize);
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
            pose<T> ret = (*this);
            vec3<T> axisZ = vec3<T>(0,0,1);
            T theta = acos(axisZ*normal/nrm);       // 回転角
            if(abs(theta)>1e-9) // 回転角があれば
            {
                vec3<T> alfa = axisZ % normal;      // 回転軸
                ret.q = ret.q * vec4<T>(alfa,theta);
            }
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
            if (abs(tmp) <= 1e-9) return INFINITY;
            return (surf.p - this->p)*(N / tmp);
        }
};

template <typename T>
std::ostream& operator<<(std::ostream& stream, pose<T>& obj)
{
//    return( stream << "pos[ m ] = "<< obj.p << "  rpy[deg] = " << obj.q.rpy()*180.0/M_PI );
    return( stream << "p = "<< obj.p << "  q = " << obj.q );
}


}
/**
 * @file vec4.h
 * @brief クォータニオンクラス
 */
#pragma once
#include <kinematics/vec3.h>
#include <bits/stdc++.h> // M_PI

namespace kinematics
{

/**
 * @brief クォータニオンクラス
 */
template <typename T>
class vec4
{
    public:
        T err = 1e-9;
        T x;    ///< ベクトル部 x
        T y;    ///< ベクトル部 y
        T z;    ///< ベクトル部 z
        T w;    ///< スカラ部
    
        /**
         * @brief 基準クォータニオン
         */
        vec4()
        {
            this->x = this->y = this->z = 0;
            this->w = 1;
        }

        /**
         * @brief 要素直接指定
         */
        vec4(T x_, T y_, T z_, T w_)
        {
            this->x = x_;
            this->y = y_;
            this->z = z_;
            this->w = w_;
        }

        /**
         * @brief 回転軸と回転角度から生成
         * @note 回転角範囲を[0, pi]に設定
         */
        vec4(vec3<T> alfa, T theta)
        {
            T nrm = alfa.nrm();
            T s = sin(theta/2.0) / nrm;
            this->x = alfa.x * s;
            this->y = alfa.y * s;
            this->z = alfa.z * s;
            this->w = cos(theta/2.0);
            if(this->w<0) (*this) = -(*this);
        }

        /**
         * @brief 3-2-1-オイラー角から生成
         */
        vec4(T roll, T pitch, T yaw)
        {
            vec4<T> Rx(vec3<T>(1,0,0), roll);
            vec4<T> Ry(vec3<T>(0,1,0), pitch);
            vec4<T> Rz(vec3<T>(0,0,1), yaw);
            vec4<T> R = Rz * Ry * Rx;
            this->x = R.x;
            this->y = R.y;
            this->z = R.z;
            this->w = R.w;
        }

        /**
         * @brief 方向余弦行列から生成
         * @param [in] C 基準座標系からthis座標系への変換行列
         */
        vec4(std::vector<vec3<T>> C)
        {
            assert(C.size()==3);
            auto tmp = 1+C[0][0]+C[1][1]+C[2][2];
            if (tmp>this->err)
            {
                this->w = 0.5*sqrt(tmp);
                auto den = 1.0/(4.0*this->w);
                this->x = -(C[2][1]-C[1][2])*den;
                this->y = -(C[0][2]-C[2][0])*den;
                this->z = -(C[1][0]-C[0][1])*den;
            } 
            else
            {
                // 回転角=pi
                this->x = sqrt(0.5*(1.0+C[0][0]));
                this->y = sqrt(0.5*(1.0+C[1][1]));
                this->z = sqrt(0.5*(1.0+C[2][2]));
                this->w = 0;

                // x>0とする
                if(C[0][1]<0) this->y = -this->y;
                if(C[0][2]<0) this->z = -this->z;
            }
            this->normalize();
        }

        /**
         * @brief 要素アクセス
         */
        T operator[](int n)
        {
            switch (n)
            {
                case 0: return(this->x);
                case 1: return(this->y);
                case 2: return(this->z);
                case 3: return(this->w);
                default:
                {
                    std::cerr << "[vec4] wrong index" << std::endl;
                    assert(false);
                }
            }
        }

        vec4<T> operator+()
        {
        	return(vec4<T>(this->x,this->y,this->z,this->w));
        }

        vec4<T> operator-()
        {
        	return(vec4<T>(-this->x,-this->y,-this->z,-this->w));
        }

        /**
         * @brief 共役クォータニオン
         */
        vec4<T> conj()
        {
            return vec4<T>(-this->x,-this->y,-this->z,this->w);
        }

        /**
         * @brief ノルム
         */
        T nrm()
        {
            return sqrt(this->x*this->x + this->y*this->y + this->z*this->z + this->w*this->w);
        }

        /**
         * @brief 正規化
         */
        vec4<T> normalize()
        {
            double nrm = this->nrm();
            assert(abs(nrm) > this->err);
            this->x = this->x / nrm;
            this->y = this->y / nrm;
            this->z = this->z / nrm;
            this->w = this->w / nrm;
            return (*this);
        }

        /**
         * @brief 代入
         */
        template<typename U>
        vec4<T> operator=(const vec4<U>& obj)
        {
            this->x = (T)obj.x;
            this->y = (T)obj.y;
            this->z = (T)obj.z;
            this->w = (T)obj.w;
            return(*this);
        }

        /**
         * @brief 各要素の一致判定（数値誤差をerrだけ許容）
         */
        bool operator==(const vec4<T>& obj)
        {
            if(abs(this->x-obj.x)>this->err) return false;
            if(abs(this->y-obj.y)>this->err) return false;
            if(abs(this->z-obj.z)>this->err) return false;
            if(abs(this->w-obj.w)>this->err) return false;
            return true;
        }

        /**
         * @brief 同回転のクォータニオン判定
         */
        bool eq(const vec4<T>& obj)
        {
            return ((*this) == obj) || (-(*this) == obj);
        }

        /**
         * @brief 不定値判定
         */
        bool isnan()
        {
            if(std::isnan(this->x)) return true;
            if(std::isnan(this->y)) return true;
            if(std::isnan(this->z)) return true;
            if(std::isnan(this->w)) return true;
            return false;
        }

        /**
         * @brief 無限大値判定
         */
        bool isinf()
        {
            if(std::isinf(this->x)) return true;
            if(std::isinf(this->y)) return true;
            if(std::isinf(this->z)) return true;
            if(std::isinf(this->w)) return true;
            return false;
        }

        /**
         * @brief 有効値判定
         */
        bool isnum()
        {
            return(!isnan() && !isinf());
        }

        /**
         * @brief クォータニオン積
         */
        vec4<T> operator*(const vec4<T>& obj)
        {
            T s1 = this->w;
            vec3<T> v1(this->x, this->y, this->z);
            T s2 = obj.w;
            vec3<T> v2(obj.x, obj.y, obj.z);
            T s = (s1 * s2) - (v1 * v2);
            vec3<T> v = (s1 * v2) + (s2 * v1) + (v1 % v2);
            return( vec4<T>(v.x, v.y, v.z, s) );
        }

        /**
         * @brief ベクトルの基準座標への変換
         * @param [in] v 変換前位置（this座標系）
         */
        vec3<T> Trans(const vec3<T>& v, bool normalize=true)
        {
            if(normalize) this->normalize();
            vec4<T> p_in(v.x, v.y, v.z, 0);
            vec4<T> p_out = (*this) * p_in * (*this).conj();
            return vec3<T>(p_out.x, p_out.y, p_out.z);
        }

        /**
         * @brief 方向余弦行列の生成
         * @return 基準座標系からthis座標系からの変換行列
         */
        std::vector<vec3<T>> C(bool normalize=true)
        {
            if(normalize) this->normalize();
            std::vector<vec3<T>> ret;
            ret.push_back( vec3<T>(1-2*(y*y+z*z), 2*(x*y+w*z)  , 2*(x*z-w*y))  );   // 1行目
            ret.push_back( vec3<T>(2*(x*y-w*z)  , 1-2*(x*x+z*z), 2*(y*z+w*x))  );
            ret.push_back( vec3<T>(2*(x*z+w*y)  , 2*(y*z-w*x)  , 1-2*(x*x+y*y)));
            return (ret);
        }



        /**
         * @brief 3-2-1-オイラー角の生成
         */
        vec3<T> rpy(bool normalize=true)
        {
            if(normalize) this->normalize();
            T roll, pitch, yaw;
            T flg = -2*(x*z-y*w);
            if (abs(flg-1) < 1.e-6)
            {
                roll  = atan2(2.0*(x*y-w*z), 2.0*(x*z+w*y));
                pitch = M_PI/2.0;
                yaw   = 0;
            }
            else if(abs(flg+1) < 1.e-6)
            {
                roll  = -atan2(2.0*(x*y-w*z), 2.0*(x*z+w*y));
                pitch = -M_PI/2.0;
                yaw   = 0;
            }
            else
            {
                T tmp = -2.0*(x*z-y*w);
                if(tmp>1)  tmp = 1;
                if(tmp<-1) tmp = -1;
                roll  = atan2(2.0*(y*z+w*x), 1.0-2.0*(x*x+y*y));
                pitch = asin(tmp);
                yaw   = atan2(2.0*(x*y+w*z), 1.0-2.0*(y*y+z*z));
            }

            return( vec3<T>(roll, pitch, yaw) );
        }



        /**
         * @brief this姿勢からobj姿勢への等価回転軸と回転角を算出
         */
        std::pair<vec3<T>, T> RotationTo(vec4<T> obj)
        {
            if((*this)==obj)    return{vec3<T>(NAN,NAN,NAN), 0};

            vec4<T> p = obj.conj()*(*this);
            if(abs(p.w)<1e-9)
            {
                // 回転角が0 or pi
            }

            std::vector<vec3<T>> C = p.C();
            //Transpose(C);
            vec3<T> knum(C[2][1]-C[1][2], C[0][2]-C[2][0], C[1][0]-C[0][1]);
            T den = C[0][0]+C[1][1]+C[2][2]-1;
            T theta = atan2(knum.nrm(), den);
            T theta_abs = abs(theta);
            vec3<T> vec;

            T den2;
            if(theta > 150.0*M_PI/180.0)
            {
                T cang = den/2.0;
                T vers = 1.0-cang;
                vec = knum.sign();
                vec.x *= sqrt(abs(C[0][0]-cang)/vers);
                vec.y *= sqrt(abs(C[1][1]-cang)/vers);
                vec.z *= sqrt(abs(C[2][2]-cang)/vers);

                int idx;
                if(abs(vec.x)<abs(vec.y))
                    idx = (abs(vec.y) < abs(vec.z)) ? 3 : 2;
                else
                    idx = (abs(vec.x) < abs(vec.z)) ? 3 : 1;

                switch(idx)
                {
                    case 1:
                        den2 = 2.0*vers * vec.x;
                        vec.y = (C[1][0]+C[0][1])/den2;
                        vec.z = (C[0][2]+C[2][0])/den2;
                        break;
                    case 2:
                        den2 = 2.0*vers * vec.y;
                        vec.x = (C[1][0]+C[0][1])/den2;
                        vec.z = (C[2][1]+C[1][2])/den2;
                        break;
                    case 3:
                        den2 = 2.0*vers * vec.z;
                        vec.x = (C[0][2]+C[2][0])/den2;
                        vec.y = (C[2][1]+C[1][2])/den2;
                        break;
                }
            }
            else
            {
                den2 = 2.0 * sin(theta);
                vec = knum/den2;
                if (theta_abs < 30.0*M_PI/180.0)
                    vec = vec/vec.nrm();
            }

            return {vec, theta};
        }

        /**
         * @brief 球面線形補間
         * @param [in] obj 補間先姿勢
         * @param [in] t 補間係数 [0,1]
         * @param [in] detour 遠回り
         */
        vec4<T> slerp(vec4<T> obj, T t, bool detour=false)
        {
            assert(0.0<=t && t<=1.0);

            T dot = x*obj.x + y*obj.y + z*obj.z + w*obj.w;
            T theta = acos(dot);
            T s = sin(theta);
            if(s==0) return *this;  // 回転なし

            if(dot>0 &&  detour) obj = -obj;    // 遠回り
            if(dot<0 && !detour) obj = -obj;    // 近回り

            T a = sin( (1.0-t)*theta )/s;
            T b = sin( (t)*theta )/s;
            vec4<T> ret;
            ret.x = a*x + b*obj.x;
            ret.y = a*y + b*obj.y;
            ret.z = a*z + b*obj.z;
            ret.w = a*w + b*obj.w;
            return( ret );
        }
};

template <typename T>
std::ostream& operator<<(std::ostream& stream, const vec4<T>& obj)
{
    char cData[512];
    sprintf(cData,"[(%+5.4e, %+5.4e, %+5.4e), %+5.4e]", obj.x, obj.y, obj.z, obj.w);
    return( stream << cData);
}


template <typename T>
std::vector<vec3<T>> rpy2C(T roll,T pitch, T yaw)
{
	T c1 = cos(roll);
	T s1 = sin(roll);
	T c2 = cos(pitch);
	T s2 = sin(pitch);
	T c3 = cos(yaw);
	T s3 = sin(yaw);
	std::vector<vec3<T>> ret;
	ret.push_back( vec3<T>(c2*c3         , c2*s3         , -s2) );
	ret.push_back( vec3<T>(s1*s2*c3-c1*s3, s1*s2*s3+c1*c3, s1*c2) );
	ret.push_back( vec3<T>(c1*s2*c3+s1*s3, c1*s2*s3-s1*c3, c1*c2) );
	return( ret );
}
/*
template <typename T>
vec3<T> C2rpy(std::vector<vec3<T>> C)
{
    assert(C.size()==3);

    T roll, pitch, yaw;
    bool flg = (abs(C[0][0]) < 1e-9) && (abs(C[0][1]) < 1e-9);
    if( flg )
    {
        // c2=0

    }
    else
    {
        // c2!=0 
        roll  = atan2(C[1][2], C[2][2]);
        pitch = asin(-C[0][2]);
        yaw   = atan2(C[0][1], C[0][0]);
    }
    return vec3<T>(roll, pitch, yaw);
}
*/
}
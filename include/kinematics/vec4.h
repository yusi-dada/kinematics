#pragma once
#include "vec3.h"
#include <bits/stdc++.h> // M_PI

namespace kinematics
{

template <typename T>
class vec4
{
    public:
        T err = 1e-9;
        T x; 
        T y; 
        T z; 
        T w;
    
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
         * @brief 回転軸と回転角度から生成
         */
        vec4(vec3<T> alfa, T theta)
        {
            T nrm = alfa.nrm();
            T s = sin(theta/2.0);
            this->x = alfa.x * s / nrm;
            this->y = alfa.y * s / nrm;
            this->z = alfa.z * s / nrm;
            this->w = cos(theta/2.0);
            if(this->w<0) (*this) = -(*this);
        }

        /**
         * @brief 代入
         */
        template<typename U>
        vec4<T> operator=(const vec4<U>& obj)
        {
            this->x = obj.x;
            this->y = obj.y;
            this->z = obj.z;
            this->w = obj.w;
            return(*this);
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
                    assert(false);
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
            if(abs(nrm) > 1.e-6)
            {
                vec4<T> ret(this->x/nrm, this->y/nrm, this->z/nrm, this->w/nrm);
                if(this->w < 0) ret = -ret;
                return( ret );
            }
            else
                assert(false);
        }

        /**
         * @brief 3-2-1-オイラー角の生成
         */
        vec3<T> rpy()
        {
            T roll, pitch, yaw;
            T flg = -2*(this->x*this->z-this->y*this->w);
            if (abs(flg-1) < 1.e-6)
            {
                roll  = atan2(2.0*(this->x*this->y-this->w*this->z), 2.0*(this->x*this->z+this->w*this->y));
                pitch = M_PI/2.0;
                yaw   = 0;
            }
            else if(abs(flg+1) < 1.e-6)
            {
                roll  = -atan2(2.0*(this->x*this->y-this->w*this->z), 2.0*(this->x*this->z+this->w*this->y));
                pitch = -M_PI/2.0;
                yaw   = 0;
            }
            else
            {
                roll  = atan2(2.0*(this->y*this->z+this->w*this->x), 1.0-2.0*(this->x*this->x+this->y*this->y));
                pitch = asin(-2.0*(this->x*this->z-this->y*this->w));
                yaw   = atan2(2.0*(this->x*this->y+this->w*this->z), 1.0-2.0*(this->y*this->y+this->z*this->z));
            }
            return( vec3<T>(roll, pitch, yaw) );
        }

        /**
         * @brief 回転行列の生成
         */
        std::vector<vec3<T>> C()
        {
            std::vector<vec3<T>> ret;
            ret.push_back( vec3<T>(1-2*(y*y+z*z), 2*(x*y+w*z)  , 2*(x*z-w*y))  );
            ret.push_back( vec3<T>(2*(x*y-w*z)  , 1-2*(x*x+z*z), 2*(y*z+w*x))  );
            ret.push_back( vec3<T>(2*(x*z+w*y)  , 2*(y*z-w*x)  , 1-2*(x*x+y*y)));
            return (ret);
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


        std::pair<vec3<T>, T> RotationTo2(vec4<T>& obj)
        {
            std::vector<vec3<T>> C = (obj.conj()*(*this)).C();
            T vx = C[2][1]-C[1][2];
            T vy = C[0][2]-C[2][0];
            T vz = C[1][0]-C[0][1];
            T l = vx*vx + vy*vy + vz*vz;
            l = sqrt(l);
            vec3<T> axis(vx,vy,vz);
            T theta = asin(l/2);
            return {axis/l, theta};
        }

        /**
         * @brief this姿勢からobj姿勢への等価回転軸と回転角を算出
         */
        std::pair<vec3<T>, T> RotationTo(vec4<T>& obj)
        {
            std::vector<vec3<T>> C = (obj.conj()*(*this)).C();
            vec3<T> knum(C[2][1]-C[1][2], C[0][2]-C[2][0], C[1][0]-C[0][1]);
            std::cerr << knum << std::endl;
            T den = C[0][0]+C[1][1]+C[2][2]-1;
            T theta = atan2(knum.nrm(), den);
            T theta_abs = abs(theta);
            vec3<T> vec;

            if(theta_abs < 1e-9)   return{vec, 0};

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

        /**
         * @brief ベクトルの基準座標への変換
         */
        vec3<T> Trans(const vec3<T>& v)
        {
            vec4<T> p_in(v.x, v.y, v.z, 0);
            vec4<T> p_out = (*this) * p_in * (*this).conj();
            return vec3<T>(p_out.x, p_out.y, p_out.z);
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

};

template <typename T>
std::ostream& operator<<(std::ostream& stream, const vec4<T>& obj)
{
    char cData[512];
    sprintf(cData,"[(%+5.4e, %+5.4e, %+5.4e), %+5.4e]", obj.x, obj.y, obj.z, obj.w);
    return( stream << cData);
}

/*
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
*/
}
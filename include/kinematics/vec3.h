/**
 * @file vec3.h
 * @brief 3次元ベクトルクラス
 */
#pragma once
#include <iostream>
#include <math.h>
#include <vector>
#include <assert.h>

/**
 * @brief キネマティクス処理名前空間
 */
namespace kinematics
{

/**
 * @brief 3次元ベクトルクラス
 */
template <typename T>
class vec3
{
    public:
        T err = 1e-9;
        T x; 
        T y; 
        T z;
        
        vec3()
        {
            this->x = this->y = this->z = 0;
        }

        vec3(T x_, T y_, T z_)
        {
            this->x = x_;
            this->y = y_;
            this->z = z_;
        }

        template<typename U>
        vec3<T> operator=(const vec3<U>& obj)
        {
            this->x = obj.x;
            this->y = obj.y;
            this->z = obj.z;
            return(*this);
        }

        T& operator[](int n)
        {
            switch (n)
            {
                case 0: return(this->x);
                case 1: return(this->y);
                case 2: return(this->z);
                default:
                {
                    std::cerr << "[vec3] wrong index" << std::endl;
                    assert(false);
                }
            }
        }

        vec3<T> operator+() const
        {
        	return(vec3<T>(this->x,this->y,this->z));
        }

        vec3<T> operator-() const
        {
        	return(vec3<T>(-this->x,-this->y,-this->z));
        }

        template<typename U>
        bool operator==(const vec3<U>& obj) const
        {
            if(abs(this->x-obj.x)>this->err) return false;
            if(abs(this->y-obj.y)>this->err) return false;
            if(abs(this->z-obj.z)>this->err) return false;
            return true;
        }

        vec3<T> operator+(const vec3<T>& obj) const
        {
            return(vec3<T>(this->x+obj.x, this->y+obj.y, this->z+obj.z));
        }

        vec3<T> operator-(const vec3<T>& obj) const
        {
            return(vec3<T>(this->x-obj.x, this->y-obj.y, this->z-obj.z));
        }

        T operator*(const vec3<T>& obj) const
        {
        	return( this->x*obj.x + this->y*obj.y + this->z*obj.z);
        }

        vec3<T> operator%(const vec3<T>& obj) const
        {
            return( vec3<T>(this->y*obj.z - this->z*obj.y,
                            this->z*obj.x - this->x*obj.z,
                            this->x*obj.y - this->y*obj.x) );	
        }

        T nrm() const
        {
            T sum = this->x*this->x + this->y*this->y + this->z*this->z;
            return(sqrt(sum));
        }

        bool isnan() const
        {
            return(std::isnan(this->x) || std::isnan(this->y) || std::isnan(this->z));
        }

        bool isinf() const
        {
            return(std::isinf(this->x) || std::isinf(this->y) || std::isinf(this->z));
        }

        bool isnum() const
        {
            return(!isnan() && !isinf());
        }

        std::vector<vec3<T>> tilde() const
        {
            std::vector<vec3<T>> ret(3);
            ret[0] = vec3<T>(0,-this->z,this->y);
            ret[1] = vec3<T>(this->z,0 ,-this->x);
            ret[2] = vec3<T>(-this->y,this->x,0);
            return ret;
        }

        vec3<T> iszero() const
        {
            vec3<T> ret(1,1,1);
            if(abs(this->x)>this->err) ret.x = 0;
            if(abs(this->y)>this->err) ret.y = 0;
            if(abs(this->z)>this->err) ret.z = 0;
            return ret;
        }

        vec3<T> sign() const
        {
            vec3<T> ret=(*this);
            for(int i=0; i<3; i++)
            {
                if(abs(ret[i])>ret.err)
                    ret[i] = (ret[i]>0) ? 1 : -1;
                else
                    ret[i] = 0;
            }
            return ret;
        }
};

template <typename T>
std::ostream& operator<<(std::ostream& stream, const vec3<T>& obj)
{
    char cData[512];
    sprintf(cData,"[%+5.4e, %+5.4e, %+5.4e]", obj.x, obj.y, obj.z);
    return( stream << cData);
}

template <typename T, typename U>
vec3<T> operator+(const vec3<T>& obj, U k)
{
    return(vec3<T>(k+obj.x, k+obj.y, k+obj.z));
}

template <typename T, typename U>
vec3<T> operator+(U k, const vec3<T>& obj)
{
    return(vec3<T>(k+obj.x, k+obj.y, k+obj.z));
}

template <typename T, typename U>
vec3<T> operator-(const vec3<T>& obj, U k)
{
    return(vec3<T>(obj.x-k, obj.y-k, obj.z-k));
}

template <typename T, typename U>
vec3<T> operator-(U k, const vec3<T>& obj)
{
    return(vec3<T>(k-obj.x, k-obj.y, k-obj.z));
}

template <typename T, typename U>
vec3<T> operator*(const vec3<T>& obj, U k)
{
    return(vec3<T>(k*obj.x, k*obj.y, k*obj.z));
}

template <typename T, typename U>
vec3<T> operator*(U k, const vec3<T>& obj)
{
    return(vec3<T>(k*obj.x, k*obj.y, k*obj.z));
}

template <typename T, typename U>
vec3<T> operator/(U k, const vec3<T>& obj)
{
    return(vec3<T>(k/obj.x, k/obj.y, k/obj.z));
}

template <typename T, typename U>
vec3<T> operator/(const vec3<T>& obj, U k)
{
    return(vec3<T>(obj.x/k, obj.y/k, obj.z/k));
}


/**
 * @brief 行列のストリーム表示
 */
template <typename T>
std::ostream& operator<<(std::ostream& stream, const std::vector<vec3<T>>& obj)
{
    for(auto p : obj)  stream << p << std::endl;
    return( stream );
}

/**
 * @brief 3x3行列の転置
 */
template <typename T>
void Transpose(std::vector<vec3<T>> &C)
{
    assert(C.size()==3);
    T tmp;
    tmp = C[0].y;   C[0].y = C[1].x; C[1].x = tmp; 
    tmp = C[0].z;   C[0].z = C[2].x; C[2].x = tmp; 
    tmp = C[1].z;   C[1].z = C[2].y; C[2].y = tmp; 
}

/**
 * @brief 行列の一致確認
 */
template <typename T>
bool operator==(std::vector<vec3<T>> C1, std::vector<vec3<T>> C2)
{
    if(C1.size()!=C2.size()) return false;
    for(int i=0; i<C1.size(); i++)
    {
        if(!(C1[i]==C2[i])) return false;
    }
    return true;
}

}



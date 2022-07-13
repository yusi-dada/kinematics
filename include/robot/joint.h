/**
 * @file joint.h
 * @brief ジョイント関節クラス
 */
#pragma once
#include <kinematics/kinematics.h>
#include <initializer_list>
#define Naxis (6)

namespace kinematics
{
/**
 * @brief ジョイント関節クラス
 */
template <typename T>
class joint
{
    public:
        T err = 1e-9;
        std::array<T, Naxis> val;

        joint()
        {
            val.fill(0);
        }

        joint(std::initializer_list<T> _val)
        {
            auto ite = _val.begin();
            for(auto &x : val)
            {
                x = *(ite++);
                if(ite==_val.end()) break;                
            }
        }

        /**
         * @brief 代入
         */
        template<typename U>
        joint<T> operator=(const joint<U>& obj)
        {
            this->val = obj.val;
            return(*this);
        }

        /**
         * @brief 要素アクセス
         */
        T& operator[](int n)
        {
            n = (n>=0) ? (n) : (this->val.size()+n);           
            assert(0<=n && n<this->val.size());
            return val[n];
        }

        joint<T> operator+()
        {
        	return(*this);
        }

        joint<T> operator-()
        {
            joint<T> ret = (*this);
            for (T &v : ret.val) v = -v;
        	return(ret);
        }

        /**
         * @brief 各要素の一致判定（数値誤差をerrだけ許容）
         */
        template<typename U>
        bool operator==(const joint<U>& obj)
        {
            for(int i=0; i<this->val.size(); i++)
            {
                if( std::abs(this->val[i]-obj.val[i]) > this->err)
                    return false;
            }
            return true;
        }

        joint<T> operator+(const joint<T>& obj)
        {
            joint<T> ret = (*this);
            for(int i=0; i<this->val.size(); i++)
                ret.val[i] = ret.val[i] + obj.val[i];
            return(ret);
        }

        joint<T> operator-(const joint<T>& obj)
        {
            joint<T> ret = (*this);
            for(int i=0; i<this->val.size(); i++)
                ret.val[i] = ret.val[i] - obj.val[i];
            return(ret);
        }

        joint<T> operator*(const joint<T>& obj)
        {
            joint<T> ret = (*this);
            for(int i=0; i<this->val.size(); i++)
                ret.val[i] = ret.val[i] * obj.val[i];
            return(ret);
        }

        joint<T> operator/(const joint<T>& obj)
        {
            joint<T> ret = (*this);
            for(int i=0; i<this->val.size(); i++)
                ret.val[i] = ret.val[i] / obj.val[i];
            return(ret);
        }

        /**
         * @brief 不定値判定
         */
        bool isnan()
        {
            for(T x : this->val){ if(std::isnan(x)) return true; }
            return false;
        }

        /**
         * @brief 無限大値判定
         */
        bool isinf()
        {
            for(T x : this->val){ if(std::isinf(x)) return true; }
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
         * @brief 全要素０判定
         */
        bool iszero()
        {
            for(T x : val){ if(std::abs(x)>err) return false; }
            return true;
        }

        /**
         * @brief 要素毎の符号関数
         */
        joint<T> sign()
        {
            joint<T> ret = (*this);
            for(T &x : ret.val)
            {
                if(std::abs(x)>err)
                    x = (x>0) ? 1 : -1;
                else
                    x = 0;
            }
            return ret;
        }

        /**
         * @brief 要素毎の絶対値関数
         */
        joint<T> abs()
        {
            joint<T> ret = (*this);
            for(T &x : ret.val)
            {
                if(x<0) x = -x;
            }
            return ret;
        }

        /**
         * @brief 飽和関数
         * @param [out] out_of_range 制約外判定
         */
        joint<T> sat(const joint<T> _min, const joint<T> _max, joint<T> *out_of_range=nullptr)
        {
            if(out_of_range) out_of_range->val.fill(1);   // 制約範囲外で初期化

            joint<T> ret = (*this);
            for(int i=0; i<this->val.size(); i++)
            {
                assert(_min.val[i]<=_max.val[i]);

                if     (ret.val[i] < _min.val[i]){ ret.val[i] = _min.val[i]; }
                else if(ret.val[i] > _max.val[i]){ ret.val[i] = _max.val[i]; }
                else if(out_of_range) { out_of_range->val[i] = 0; }
            }
            return ret;
        }

        /**
         * @brief 制約内判定
         */
        bool in_range(const joint<T> _min, const joint<T> _max)
        {
            for(int i=0; i<this->val.size(); i++)
            {
                assert(_min.val[i]<=_max.val[i]);
                if(this->val[i] < _min.val[i]){ return false; }
                if(this->val[i] > _max.val[i]){ return false; }
            }
            return true;
        }

};

template <typename T>
std::ostream& operator<<(std::ostream& stream, const joint<T>& obj)
{
    char cData[512];
    joint<T> tmp = obj;
    sprintf(cData,"[%+5.2f, %+5.2f, %+5.2f, %+5.2f, %+5.2f, %+5.2f]",
                    tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
    return( stream << cData);
}

template <typename T, typename U>
joint<T> operator+(const joint<T>& obj, U k)
{
    joint<T> ret = obj;
    for(auto &x : ret.val) x = x + k;
    return ret;
}

template <typename T, typename U>
joint<T> operator+(U k, const joint<T>& obj)
{
    joint<T> ret = obj;
    for(auto &x : ret.val) x = k + x;
    return ret;
}

template <typename T, typename U>
joint<T> operator-(const joint<T>& obj, U k)
{
    joint<T> ret = obj;
    for(auto &x : ret.val) x = x - k;
    return ret;
}

template <typename T, typename U>
joint<T> operator-(U k, const joint<T>& obj)
{
    joint<T> ret = obj;
    for(auto &x : ret.val) x = k - x;
    return ret;
}

template <typename T, typename U>
joint<T> operator*(const joint<T>& obj, U k)
{
    joint<T> ret = obj;
    for(auto &x : ret.val) x = x * k;
    return ret;
}

template <typename T, typename U>
joint<T> operator*(U k, const joint<T>& obj)
{
    joint<T> ret = obj;
    for(auto &x : ret.val) x = k * x;
    return ret;
}

template <typename T, typename U>
joint<T> operator/(const joint<T>& obj, U k)
{
    joint<T> ret = obj;
    for(auto &x : ret.val) x = x / k;
    return ret;
}

template <typename T, typename U>
joint<T> operator/(U k, const joint<T>& obj)
{
    joint<T> ret = obj;
    for(auto &x : ret.val) x = k / x;
    return ret;
}



}
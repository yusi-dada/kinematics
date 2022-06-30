/**
 * @file camera.h
 * @brief カメラモデル
 */
#pragma once
#include "pose.h"

namespace kinematics
{
/**
 * @brief カメラモデル
 * @details 視線方向＝Z軸正（XY軸は画像座標系と同じ向き）
 */
template <typename T>
class camera
{
    public:
        T z_len = 0.05; ///< カメラモデル[m]
        T tanH;         ///< 水平方向画角/2[rad]
        T tanV;         ///< 水平方向画角/2[rad]
        pose<T> p0;     ///< 原点位置・姿勢

        camera(){}

        /**
         * @brief カメラモデル生成
         * @param [in] p0_ カメラ座標系
         * @param [in] yaw カメラ座標系で視線方向の回転
         */
        camera(T tanH_, T tanV_, pose<T> p0_, T yaw=0)
        {
            assert(tanH_>0 && tanV_>0);
            this->tanH = tanH_;
            this->tanV = tanV_;
            this->p0   = p0_.rotate(2, yaw); 
        }

        /**
         * @brief 代入
         */
        template <typename U>
        camera operator=(const camera<U>& obj)
        {
            this->z_len = obj.z_len;
            this->tanH = obj.tanH;
            this->tanV = obj.tanV;
            this->p0 = obj.p0;
            return (*this);
        }

        /**
         * @brief 画像座標系からカメラ座標系への変換
         * @param [in] p 画像座標系[0, 1]
         * @param [in] len カメラ視線方向距離
         * @param [out] p カメラ座標系(視線方向Z=z_len平面への投影)
         * @retval true 変換成功
         * @retval false 変換失敗（視野範囲外）
         */
        bool image2camera(vec3<T> &p, T len=NAN)
        {
            len = std::isnan(len) ? z_len : len;
            
            // 視野範囲外
            if(0.0>p.x || p.x>1.0 || 0.0>p.y || p.y>1.0 || len<=0)
                return false;

            // 画像座標系→カメラ座標系
            p = 2.0*p - 1.0;    // [0,1] -> [-1,1]
            p.x = p.x * len * this->tanH;
            p.y = p.y * len * this->tanV;
            p.z = len;
            return true;
        }

        /**
         * @brief 画像座標系での位置を基準座標系に変換
         * @param [in] p 画像座標系で画角に対する比率[0,1]
         * @param [out] p 基準座標系での位置
         * @param [in] surf 投影面座標系（基準座標系、Z方向が法線）
         * @retval true 変換成功
         * @retval false 変換失敗（射影面が並行 or 逆方向）
         */
        bool image2pos(vec3<T> &p, pose<T> surf)
        {
            // 画像座標系→カメラ座標系
            if(!image2camera(p)) return false;

            // 射影面への伸展倍率演算
            T d = this->p0.projection(p, surf, vec3<T>(0,0,1));
            if(isinf(d) || d<0)
            {
                // 無効解を設定
                p = p*NAN;  
                return false;
            } 
            else
            {
                // 基準座標系へ変換
                p = this->p0.Trans_pnt(d*p);
                return true;
            }
        }

        /**
         * @brief 画像座標系での位置を基準座標系に変換
         * @param [in] p 画像座標系で画角に対する比率[0,1]
         * @param [out] p 基準座標系での位置
         * @param [in] surf 投影面座標系（基準座標系、Z方向が法線）
         * @retval true 変換成功
         * @retval false 変換失敗（射影面が並行 or 逆方向）
         */
        bool image2pos(std::vector<vec3<T>> &p, pose<T> surf)
        {
            for(auto &pp : p)
            {
                if(!image2pos(pp, surf))
                    return false;
            }
            return true;
        }

        /**
         * @brief 基準座標系での位置を画像座標系に変換
         * @param [in] p 基準座標系での位置
         * @param [out] p 画像座標系で画角に対する比率[0,1]
         * @retval true 変換成功
         * @retval false 変換失敗
         */
        bool pos2image(vec3<T> &p)
        {
            // カメラ投影面生成
            // カメラ座標系に対してZ軸=1だけオフセットをとる座標系生成
            pose<T> surf = this->p0 * pose<T>(vec3<T>(0,0,1), vec4<T>());

            // 基準座標位置をカメラ座標系に変換
            // カメラ原点から入力位置までのベクトルをカメラ座標系に変換
            p = pose<T>().Trans_vec(p - this->p0.p, this->p0);

            // 射影面への伸展倍率演算
            // 射影面法線はカメラ視線方向なので(0,0,1)
            T d = this->p0.projection(p, surf, vec3<T>(0,0,1));
            if(isinf(d) || d<0) return false;

            // カメラ投影面での位置（カメラ座標系）
            // カメラ画角で正規化[-1, 1]
            p = d*p;
            p.x = p.x / this->tanH;
            p.y = p.y / this->tanV;
            p.z = 0.0;
            if((p.x > 1.0) || (p.x < -1.0) || (p.y > 1.0) || (p.y < -1.0))
                return false;   // 視野範囲外
            
            // 画像座標系に変換
            p = 0.5*(p + 1.0);
            return true;
        }

        /**
         * @brief 基準座標系での位置を画像座標系に変換
         * @param [in] p 基準座標系での位置
         * @param [out] p 画像座標系で画角に対する比率[0,1]
         * @retval true 変換成功
         * @retval false 変換失敗
         */
        bool pos2image(std::vector<vec3<T>> &p)
        {
            for(auto &pp : p)
            {
                if(!pos2image(pp))
                    return false;
            }
            return true;
        }

        

        /**
         * @brief 画像座標系表現での3点から座標系指定
         * @param [in] origin 原点（画像座標系）
         * @param [in] x X軸方向（画像座標系）
         * @param [in] y Y軸方向（画像座標系）
         * @param [in] surf 射影面（基準座標系、Z方向が法線）
         */
        /*
        bool coordinate(vec3<T> origin, vec3<T> x, vec3<T> y, pose<T> surf)
        {
            // 画像座標系を基準座標系へ変換
            if(!image2pos(origin, surf)) return false;
            if(!image2pos(x, surf)) return false;
            if(!image2pos(y, surf)) return false;

        }*/

};

}
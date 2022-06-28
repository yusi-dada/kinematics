/**
 * @file ros_marker.h
 * @brief マーカオブジェクトの生成
 */
#pragma once
#include "ros_func.h"
#include "camera.h"
#include "pose_array.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

namespace ros_func
{
using namespace kinematics;

/**
 * @brief マーカの初期化指令
 */
void deleteAll(visualization_msgs::MarkerArray& ma);

/**
 * @brief 初期化マーカデータの生成
 */
visualization_msgs::MarkerArray InitMarker();

/**
 * @brief 点マーカ生成
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] pos 位置データ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void PointMarker(visualization_msgs::MarkerArray& ma, std::string frame, std::vector<vec3<T>> pos, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    visualization_msgs::Marker m;
    m.points.clear();
    for (auto p:pos)
    {
        if(p.isnum())
            m.points.push_back(Point(p));
    }
    if(m.points.size()==0) return;

    m.header.frame_id = frame;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0);
    m.type = visualization_msgs::Marker::POINTS;
    m.scale.x = m.scale.y = m.scale.z = 0.001;
    m.color = color;
    m.id = ma.markers.size();
    m.ns = ns;
    ma.markers.push_back(m);
}

/**
 * @brief 線分マーカデータの生成
 * @brief [in] frame 基準フレーム
 * @brief [in] start 先頭位置データ
 * @brief [in] end 終端位置データ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
static visualization_msgs::Marker SegmentMarker(std::string frame, vec3<T> start, vec3<T> end, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0);
    geometry_msgs::Vector3 arrow;  // config arrow shape
    arrow.x = 0.0008;    // shaft diameter
    arrow.y = 0.004;    // head diameter
    arrow.z = 0.01;     // arrow head length
    m.scale = arrow;
    m.color = color;
    m.points.resize(2);
    m.points[0] = Point(start);
    m.points[1] = Point(end);
    m.ns = ns;
    return m;    
}

/**
 * @brief 線分マーカデータの生成
 * @brief [in] frame 基準フレーム
 * @brief [in] pos 三角形の構成点（点数は３の倍数）
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void TriangleMarker(visualization_msgs::MarkerArray& ma, std::string frame, std::vector<vec3<T>> pos, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    if(pos.size()%3!=0) return;

    visualization_msgs::Marker m;
    m.points.clear();
    for (auto p:pos)
    {
        if(!p.isnum()) return;
        m.points.push_back(Point(p));
        m.colors.push_back(color);
    }
    m.header.frame_id = frame;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0);
    m.type = visualization_msgs::Marker::TRIANGLE_LIST;
    m.scale.x = m.scale.y = m.scale.z = 1;
    m.color = color;
    m.id = ma.markers.size();
    m.ns = ns;
    ma.markers.push_back(m);
}

/**
 * @brief 直方体マーカデータの生成
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] p0 位置・姿勢データ
 * @brief [in] size 直方体サイズデータ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void CubeMarker(visualization_msgs::MarkerArray& ma, std::string frame, pose<T> p0, vec3<T> size, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0);
    m.type = visualization_msgs::Marker::CUBE;
    m.scale.x = size.x;
    m.scale.y = size.y;
    m.scale.z = size.z;
    m.pose = Pose(p0);
    m.color = color;
    m.id = ma.markers.size();
    m.ns = ns;
    ma.markers.push_back(m);
}

/**
 * @brief 点マーカ生成
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] pos 位置データ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void PointMarker(visualization_msgs::MarkerArray& ma, std::string frame, vec3<T> pos, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    std::vector<vec3<T>> pos_array;
    pos_array.push_back(pos);
    PointMarker(ma, frame, pos_array, color, ns);
}

/**
 * @brief 線マーカ生成
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] start 先頭位置データ
 * @brief [in] end 終端位置データ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void LineMarker(visualization_msgs::MarkerArray& ma, std::string frame, vec3<T> start, vec3<T> end, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    if(start.isnum() && end.isnum())
    {
        auto m = SegmentMarker(frame, start, end, color, ns);
        m.type = visualization_msgs::Marker::LINE_LIST;
        m.id = ma.markers.size();
        ma.markers.push_back(m);
    }
}

/**
 * @brief 矢印マーカ生成
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] start 先頭位置データ
 * @brief [in] end 終端位置データ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void ArrowMarker(visualization_msgs::MarkerArray& ma, std::string frame, vec3<T> start, vec3<T> end, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    if(start.isnum() && end.isnum())
    {
        auto m = SegmentMarker(frame, start, end, color, ns);
        m.type = visualization_msgs::Marker::ARROW;
        m.id = ma.markers.size();
        ma.markers.push_back(m);
    }
}

/**
 * @brief 直線マーカ生成（隣接データを接続）
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] pos 位置データ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void LineChainMarker(visualization_msgs::MarkerArray& ma, std::string frame, std::vector<vec3<T>> pos, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    if(pos.size()>1)
    {
        for(int i=0; i<pos.size()-1; i++)
            LineMarker(ma, frame, pos[i], pos[i+1], color, ns);
    }
}

/**
 * @brief 矢印マーカ生成（隣接データを接続）
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] pos 位置データ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void ArrowChainMarker(visualization_msgs::MarkerArray& ma, std::string frame, std::vector<vec3<T>> pos, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    if(pos.size()>1)
    {
        for(int i=0; i<pos.size()-1; i++)
            ArrowMarker(ma, frame, pos[i], pos[i+1], color, ns);
    }
}

/**
 * @brief 直線マーカ生成（データを順番に接続しループ）
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] pos 位置データ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void LineLoopMarker(visualization_msgs::MarkerArray& ma, std::string frame, std::vector<vec3<T>> pos, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    if(pos.size()>2)
    {
        pos.push_back(pos[0]);
        for(int i=0; i<pos.size()-1; i++)
            LineMarker(ma, frame, pos[i], pos[i+1], color, ns);
    }
}

/**
 * @brief ポリゴン生成（データを順番に接続しループ）
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] pos 位置データ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void PolygonMarker(visualization_msgs::MarkerArray& ma, std::string frame, std::vector<vec3<T>> pos, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    if(pos.size()>2)
    {
        int Ntriangle = pos.size()-2;
        std::vector<vec3<T>> pnts(3*Ntriangle);
        for(int i=0; i<Ntriangle; i++)
        {
            pnts[3*i+0] = pos[0];
            pnts[3*i+1] = pos[i+1];
            pnts[3*i+2] = pos[i+2];
        }
        TriangleMarker(ma, frame, pnts, color, ns);
    }
}

/**
 * @brief 直方体のワイヤーフレームマーカ生成
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] p0 姿勢データ
 * @brief [in] size サイズデータ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void CubeWireMarker(visualization_msgs::MarkerArray& ma, std::string frame, pose<T> p0, vec3<T> size, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    std::vector<vec3<T>> pos1, pos2;
    pos1.resize(4);
    pos2.resize(4);
    pos1[0] = p0.Trans_pnt(vec3<T>( size.x/2,  size.y/2, size.z/2));
    pos1[1] = p0.Trans_pnt(vec3<T>(-size.x/2,  size.y/2, size.z/2));
    pos1[2] = p0.Trans_pnt(vec3<T>(-size.x/2, -size.y/2, size.z/2));
    pos1[3] = p0.Trans_pnt(vec3<T>( size.x/2, -size.y/2, size.z/2));

    pos2[0] = p0.Trans_pnt(vec3<T>( size.x/2,  size.y/2, -size.z/2));
    pos2[1] = p0.Trans_pnt(vec3<T>(-size.x/2,  size.y/2, -size.z/2));
    pos2[2] = p0.Trans_pnt(vec3<T>(-size.x/2, -size.y/2, -size.z/2));
    pos2[3] = p0.Trans_pnt(vec3<T>( size.x/2, -size.y/2, -size.z/2));

    LineLoopMarker(ma, frame, pos1, color, ns);
    LineLoopMarker(ma, frame, pos2, color, ns);
    for(int i=0;i<4;i++)
        LineMarker(ma, frame, pos1[i], pos2[i], color, ns);
}

/**
 * @brief N角錐マーカ生成（頂点から底面への線分）
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] virtex 頂点データ
 * @brief [in] bottom 底辺データ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void PyramidMarker(visualization_msgs::MarkerArray& ma, std::string frame, vec3<T> virtex, std::vector<vec3<T>> bottom, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    for(auto b:bottom)
        LineMarker(ma, frame, virtex, b, ColorRGBA(1,1,1,0.5), ns);

    LineLoopMarker(ma, frame, bottom, color, ns);
    if(color.a > 0.1) color.a = 0.2;
    PolygonMarker(ma, frame, bottom, color, ns);
}

/**
 * @brief カメラマーカ生成
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] cam カメラ
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void CameraMarker(visualization_msgs::MarkerArray& ma, std::string frame, camera<T> cam, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    T len = cam.z_len;  // display size [m]
    T x = len*cam.tanH;
    T y = len*cam.tanV;
    std::vector<vec3<T>> bottom;
    bottom.resize(4);
    bottom[0] = cam.p0.Trans_pnt(vec3<T>( x,  y, len));
    bottom[1] = cam.p0.Trans_pnt(vec3<T>(-x,  y, len));
    bottom[2] = cam.p0.Trans_pnt(vec3<T>(-x, -y, len));
    bottom[3] = cam.p0.Trans_pnt(vec3<T>( x, -y, len));
    PyramidMarker(ma, frame, cam.p0.p, bottom, color, ns);
    PointMarker(ma, frame, bottom[2], ColorRGBA(1,0,0,1), ns);  // left-top
}

/**
 * @brief カメラ射影面マーカ生成
 * @brief [in/out] ma マーカデータ
 * @brief [in] frame 基準フレーム
 * @brief [in] cam カメラ
 * @brief [in] surf 射影面
 * @brief [in] color 色データ
 * @brief [in] ns 名前空間
 */
template <typename T>
void ImageSurfaceMarker(visualization_msgs::MarkerArray& ma, std::string frame, camera<T> cam, pose<T>surf, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    std::vector<vec3<T>> cornor(4);
    cornor[0] = vec3<T>(0, 0, 0);
    cornor[1] = vec3<T>(1, 0, 0);
    cornor[2] = vec3<T>(1, 1, 0);
    cornor[3] = vec3<T>(0, 1, 0);
    // 画像座標系を投影面に射影し基準座標系表現
    // 射影面に射影できない点は関数内で無効解Nanに設定、線の描画をスルーする
    for (auto &c:cornor)
        cam.image2pos(c, surf);
    PyramidMarker(ma, frame, cam.p0.p, cornor, color, ns);
}

template <typename T>
void LinkMarker(visualization_msgs::MarkerArray& ma, std::string frame, pose_array<T> pos, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1), std::string ns="")
{
    // リンクの各接点を取得し描画
    auto edge_points = pos.p();
    LineChainMarker(ma, "world", edge_points, ColorRGBA(1,1,0,1), ns+"link");
    PointMarker(ma, "world", edge_points, color, ns+"joint");
}


class PathMarker
{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        int markerSize;
        visualization_msgs::MarkerArray path;

    public:
        PathMarker(std::string name="default", int mSize=10)
        {
            assert(mSize>0);
            markerSize = mSize;
            pub = nh.advertise<visualization_msgs::MarkerArray>(name, 1);
        }

        void clearPath()
        {
            path.markers.clear();
            pub.publish(InitMarker());
        }

        template <typename T>
        void append(vec3<T> point, std_msgs::ColorRGBA color=ColorRGBA(1,1,1,1))
        {
            PointMarker(path, "world", point, color, "path");
            if (path.markers.size()>markerSize)
            {
                int id = path.markers[0].id;
                path.markers.erase(path.markers.begin());
                path.markers.back().id = id;
            }
            pub.publish(path);
        }
};

/**
 * @brief MarkerArray publisher
 */
class rviz_publisher
{
    private:
        ros::NodeHandle nh;
        tf::TransformBroadcaster br;
        ros::Publisher pub;

    public:
        /**
         * @brief Publisher生成
         * @param [in] name Publisher名
         */
        inline rviz_publisher(std::string name="default")
        {
            pub = nh.advertise<visualization_msgs::MarkerArray>(name, 1);
        }

        /**
         * @brief MakerArrayの配信
         */
        inline void publish(visualization_msgs::MarkerArray ma)
        {
            pub.publish(ma);
        }

        /**
         * @brief Axesの配信
         */
        template <typename T>
        void setAxis(std::string parent, std::string name, pose<T> pos)
        {
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(pos.p.x, pos.p.y, pos.p.z) );
            transform.setRotation(tf::Quaternion(pos.q.x, pos.q.y, pos.q.z, pos.q.w));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, name));
        }

        /**
         * @brief Axesの配信
         */
        template <typename T>
        void setAxis(std::string parent, std::string name, std::vector<pose<T>> pos)
        {
            for(int i=0; i<pos.size(); i++)
                setAxis(parent, name+std::to_string(i), pos[i]);
        }


};

}
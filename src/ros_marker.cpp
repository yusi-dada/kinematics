#include "ros_marker.h"

namespace ros_func
{

void deleteAll(visualization_msgs::MarkerArray& ma)
{
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::DELETEALL;
    ma.markers.push_back(m);
}

visualization_msgs::MarkerArray InitMarker()
{
    visualization_msgs::MarkerArray ma;
    deleteAll(ma);
    return ma;
}

}
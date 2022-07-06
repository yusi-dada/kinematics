#include <urdf/urdf.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urdf_test");
  ros::NodeHandle nh("~");

  std::string xml_string;
  nh.param<std::string>("/robot_description", xml_string, "");

  urdf_func::getModel(xml_string);

}
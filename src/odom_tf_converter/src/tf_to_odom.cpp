#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_to_odom_converter");
  std::string source_frame_id, target_frame_id, odom_frame_id;

  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  //  priv_node.param<std::string>("odom_frame", odom_frame_id, "odom");
  // priv_node.param<std::string>("source_frame", source_frame_id, "/base_footprint");
  // priv_node.param<std::string>("target_frame", target_frame_id, "/odom_combined");

  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom111", 1);

  tf::TransformListener listener;

  ros::Rate rate(25.0);
  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(20.0));
      // ros::Time::now()
      listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    nav_msgs::Odometry odom;
    // copy pose to odom msg
    odom.header.stamp = transform.stamp_;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_footprint";
    geometry_msgs::TransformStamped ts_msg;
    tf::transformStampedTFToMsg(transform, ts_msg);
    odom.pose.pose.position.x = ts_msg.transform.translation.x;
    odom.pose.pose.position.y = ts_msg.transform.translation.y;
    odom.pose.pose.position.z = ts_msg.transform.translation.z;
    odom.pose.pose.orientation = ts_msg.transform.rotation;

    odom_pub.publish(odom);
    rate.sleep();
  }

  return 0;
}

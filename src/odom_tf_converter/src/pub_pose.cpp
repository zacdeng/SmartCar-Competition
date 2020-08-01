#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>

std::string base_frame;
std::string odom_frame;
std::string map_frame;
std::string world_frame;

void transformAsMatrix(const tf::Transform& bt, Eigen::Matrix4f& out_mat)
{
  double mv[12];
  bt.getBasis().getOpenGLSubMatrix(mv);

  tf::Vector3 origin = bt.getOrigin();

  out_mat(0, 0) = mv[0];
  out_mat(0, 1) = mv[4];
  out_mat(0, 2) = mv[8];
  out_mat(1, 0) = mv[1];
  out_mat(1, 1) = mv[5];
  out_mat(1, 2) = mv[9];
  out_mat(2, 0) = mv[2];
  out_mat(2, 1) = mv[6];
  out_mat(2, 2) = mv[10];

  out_mat(3, 0) = out_mat(3, 1) = out_mat(3, 2) = 0;
  out_mat(3, 3) = 1;
  out_mat(0, 3) = origin.x();
  out_mat(1, 3) = origin.y();
  out_mat(2, 3) = origin.z();
}

void matrixAsTransfrom(const Eigen::Matrix4f& out_mat, tf::Transform& bt)
{
  double mv[12];

  mv[0] = out_mat(0, 0);
  mv[4] = out_mat(0, 1);
  mv[8] = out_mat(0, 2);
  mv[1] = out_mat(1, 0);
  mv[5] = out_mat(1, 1);
  mv[9] = out_mat(1, 2);
  mv[2] = out_mat(2, 0);
  mv[6] = out_mat(2, 1);
  mv[10] = out_mat(2, 2);

  tf::Matrix3x3 basis;
  basis.setFromOpenGLSubMatrix(mv);
  tf::Vector3 origin(out_mat(0, 3), out_mat(1, 3), out_mat(2, 3));

  ROS_DEBUG("origin %f %f %f", origin.x(), origin.y(), origin.z());

  bt = tf::Transform(basis, origin);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_to_odom_converter");

  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("base_frame", base_frame, "base");
  priv_node.param<std::string>("odom_frame", odom_frame, "odom");
  priv_node.param<std::string>("map_frame", map_frame, "map");
  priv_node.param<std::string>("world_frame", world_frame, "world");

  tf::TransformListener listener;
  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("world_pose", 10);

  ros::Rate rate(25.0);
  while (node.ok())
  {
    tf::StampedTransform Map2Odom;
    try
    {
      listener.lookupTransform(world_frame, odom_frame, ros::Time(0), Map2Odom);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    tf::StampedTransform Odom2Base;
    try
    {
      listener.lookupTransform(odom_frame, base_frame, ros::Time(0), Odom2Base);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    Eigen::Matrix4f MapToOdom, OdomToBase, TotalTrans;
    transformAsMatrix(Map2Odom, MapToOdom);
    transformAsMatrix(Odom2Base, OdomToBase);

    TotalTrans = MapToOdom * OdomToBase;

    tf::StampedTransform transform;
    matrixAsTransfrom(TotalTrans, transform);

    nav_msgs::Odometry odom;
    // copy pose to odom msg
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = world_frame;
    odom.child_frame_id = base_frame;
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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

bool should_inverse;
std::string source_frame;
std::string target_frame;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
	static tf::TransformBroadcaster br;
	static tf::Transform odomTF;

  odomTF.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x,
                               odom_msg->pose.pose.position.y,
                               odom_msg->pose.pose.position.z) );

  odomTF.setRotation(tf::Quaternion(odom_msg->pose.pose.orientation.x,
                                    odom_msg->pose.pose.orientation.y,
                                    odom_msg->pose.pose.orientation.z,
                                    odom_msg->pose.pose.orientation.w));

	if (should_inverse) {
    br.sendTransform(tf::StampedTransform(odomTF.inverse(),
                                          odom_msg->header.stamp,
                                          source_frame,
                                          target_frame));
  } else {
    br.sendTransform(tf::StampedTransform(odomTF, odom_msg->header.stamp,
                                          target_frame,
                                          source_frame));
  }

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odom_listener");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	ros::Subscriber sub = n.subscribe("odom", 100, OdomCallback);

	np.param<bool>("inverse", should_inverse, false);
	np.param<std::string>("source_frame", source_frame, "source_frame");
	np.param<std::string>("target_frame", target_frame, "target_frame");

	ros::spin();

	return 0;
}

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>




void callback(const nav_msgs::Odometry::ConstPtr& odom){
tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time  = ros::Time::now();
 
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";//base_link";

    odom_trans.transform.translation.x =  odom->pose.pose.position.x ;
    odom_trans.transform.translation.y =  odom->pose.pose.position.y ;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom->pose.pose.orientation;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
	ROS_INFO_STREAM(odom_trans);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/odom", 1000, callback);
  ros::spin();
  return 0;
}

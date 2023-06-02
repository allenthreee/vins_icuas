#include <ros/ros.h>
#include <fstream>
#include <string>
#include <nav_msgs/Odometry.h>

static std::string camera_log_path;

void result_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
    std::ofstream save(camera_log_path,std::ios::app);
    save<<std::setprecision(20)<<odom->header.stamp.now().toSec()<<" "<<odom->pose.pose.position.x<<" "<<odom->pose.pose.position.y<<" "<<odom->pose.pose.position.z<<" ";
    save<<std::setprecision(20)<<odom->pose.pose.orientation.x<<" "<<odom->pose.pose.orientation.y<<" "<<odom->pose.pose.orientation.z<<" "<<odom->pose.pose.orientation.w<<std::endl;
    save.close();
    std::cout<<camera_log_path<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "result_to_txt");
    ros::NodeHandle nh;
    ros::Subscriber result_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/camera_pose", 20, result_cb);
    
    nh.getParam("/result_to_txt/camera_log_path", camera_log_path);
    remove(camera_log_path.c_str());

    ros::spin();

    return 0;
}
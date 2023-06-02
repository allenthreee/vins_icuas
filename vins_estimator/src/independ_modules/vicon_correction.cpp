#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>


class VICON
{
    private:

    bool initialized = false;
    geometry_msgs::PoseStamped vicon_corrected;
    Eigen::Quaterniond q_init;
    Eigen::Quaterniond q_current;
    Eigen::Matrix4d translation_matrix = Eigen::Matrix4d::Constant(0);
    Eigen::Matrix3d rotation_matrix, rotation_matrix_current;
    ros::Subscriber vicon_sub;
    ros::Publisher vicon_pub;

    public:

    VICON(ros::NodeHandle& nh)
    {
        vicon_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vicon_client/METRICS/pose", 20, &VICON::vicon_cb, this);
        vicon_pub = nh.advertise<geometry_msgs::PoseStamped>("/vicon_corrected",20);
    }

    

    void vicon_cb(const geometry_msgs::PoseStamped::ConstPtr& vicon_pose)
    {   
        if (!initialized)
        {
            generate_rotation_matrix(vicon_pose);
        }

        Eigen::Vector4d position_vector_vicon(vicon_pose->pose.position.x,vicon_pose->pose.position.y,vicon_pose->pose.position.z,1);
        Eigen::Vector4d position_vector_body = translation_matrix * position_vector_vicon;
        
        q_current.w() = vicon_pose->pose.orientation.w;
        q_current.x() = vicon_pose->pose.orientation.x;
        q_current.y() = vicon_pose->pose.orientation.y;
        q_current.z() = vicon_pose->pose.orientation.z;
        rotation_matrix_current = q_current.toRotationMatrix();
        Eigen::Quaterniond q_corrected(rotation_matrix*rotation_matrix_current);

        vicon_corrected.header = vicon_pose->header;
        vicon_corrected.header.frame_id = "world";

        vicon_corrected.pose.position.x = position_vector_body(0);
        vicon_corrected.pose.position.y = position_vector_body(1);
        vicon_corrected.pose.position.z = position_vector_body(2);
        vicon_corrected.pose.orientation.w = q_corrected.w();
        vicon_corrected.pose.orientation.x = q_corrected.x();
        vicon_corrected.pose.orientation.y = q_corrected.y();
        vicon_corrected.pose.orientation.z = q_corrected.z();

        vicon_pub.publish(vicon_corrected);
    }

    void generate_rotation_matrix(const geometry_msgs::PoseStamped::ConstPtr& pose)
    {
        q_init.w() = pose->pose.orientation.w;
        q_init.x() = pose->pose.orientation.x;
        q_init.y() = pose->pose.orientation.y;
        q_init.z() = pose->pose.orientation.z;

        Eigen::Matrix3d vicon_to_body_rotation = q_init.toRotationMatrix();
        Eigen::Vector3d vicon_to_body_translation(pose->pose.position.x,pose->pose.position.y,pose->pose.position.z);

        translation_matrix.block<3,3>(0,0) = vicon_to_body_rotation;
        translation_matrix.block<3,1>(0,3) = vicon_to_body_translation;
        translation_matrix(3,3) = 1;
        translation_matrix = translation_matrix.inverse();

        rotation_matrix = vicon_to_body_rotation.inverse();

        initialized = true;
        std::cout<<"Vicon correction initialized!"<<std::endl;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_correction");
    ros::NodeHandle nh;

    VICON vicon(nh);

    ros::spin();

    return 0;
}
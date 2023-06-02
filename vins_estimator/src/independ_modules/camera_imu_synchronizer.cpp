#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>

class CAMERA_SYNC
{
    public:

    ros::Publisher camera_imu_pub;
    sensor_msgs::Imu camera_imu;
    message_filters::Subscriber<sensor_msgs::Imu> accel_sub;
    message_filters::Subscriber<sensor_msgs::Imu> gyro_sub;

    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
    boost::shared_ptr<sync> sync_; 

    public:

    CAMERA_SYNC(ros::NodeHandle& nh)
    {
        camera_imu_pub = nh.advertise<sensor_msgs::Imu>("/camera/imu",20);

        accel_sub.subscribe(nh,"/camera/accel/sample",1);
        gyro_sub.subscribe(nh,"/camera/gyro/sample",1);

        sync_.reset(new sync(MySyncPolicy(10), accel_sub, gyro_sub));            
        sync_->registerCallback(boost::bind(&CAMERA_SYNC::camera_imu_callback, this, _1, _2));          

        // sync.registerCallback(boost::bind(&camera_imu_callback, _1, _2));
    }

    void camera_imu_callback(const sensor_msgs::ImuConstPtr& accel, const sensor_msgs::ImuConstPtr& gyro)
    {
        camera_imu.header = gyro->header;
        camera_imu.linear_acceleration = accel->linear_acceleration;
        
        camera_imu.angular_velocity = gyro->angular_velocity;
        camera_imu_pub.publish(camera_imu);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"camera_imu_synchronizer");

    ros::NodeHandle nh;
    CAMERA_SYNC camera_sync(nh);

    ros::spin();

    return 0;
}
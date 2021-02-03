#include<iostream>
#include<cmath>
#include<unordered_map>
#include<vector>
#include"ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Bool.h"
#include "lab2/get_distances_to_ego_edges.h"

class Safety
{
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher brake_pub_;
    ros::Publisher brake_bool_pub_;
    ros::NodeHandle n_;
    GetDistancesToEgoEdges dist_;
    float current_vel_x_ = 0;
    std_msgs::Bool brake_bool_;
    ackermann_msgs::AckermannDriveStamped ackermann_msg_;
    float ttc_thresh_ = 0.32;
    float brake_rate_ = 10;
    float brake_duration_ = 1;
    public:
    Safety()
    {
        scan_sub_ = n_.subscribe("scan",1,&Safety::processLidarScan,this);
        odom_sub_ = n_.subscribe("odom",1,&Safety::velocityUpdate,this);
        brake_pub_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("brake",1);
        brake_bool_pub_ = n_.advertise<std_msgs::Bool>("brake_bool",1);

        float width = 0.2032;
        float height = 0.3302;
        float lidar_to_base_link = 0.275;
        float ray_count = 1080;

        n_.getParam("width",width);
        n_.getParam("wheelbase",height);
        n_.getParam("scan_distance_to_base_link",lidar_to_base_link);
        n_.getParam("scan_beams",ray_count);

        GetDistancesToEgoEdges dist(width,height,ray_count);
        dist.lidar_to_ego_center_ = lidar_to_base_link - height/2;
        dist.getEgoEdgeLidarDist();
        
        brake_bool_.data = true;
        ackermann_msg_.drive.speed = 0;
        dist_ = dist;
    }
    void processLidarScan(const sensor_msgs::LaserScanConstPtr &scan_msg)
    {
        float min_time = scan_msg->range_max+1;
        for(int i=0; i<scan_msg->ranges.size();i++)
        {
            float num = scan_msg->ranges[i] - dist_.dist_to_ego_edge_[i];
            float den = current_vel_x_ * std::cos(dist_.ray_angles_[i] - M_PI_2);
            if(den <= 0.0)
            {
                continue;
            }
            min_time = std::min(min_time,num/den);

            if(min_time<ttc_thresh_){
                ROS_INFO_STREAM("Applying Brake \n");
                brakePublish();
                ROS_INFO_STREAM("Applying Brake Stopped \n");
                return;
            }
        }
    }
    void velocityUpdate(const nav_msgs::OdometryConstPtr &odom_msg)
    {
        current_vel_x_ = odom_msg->twist.twist.linear.x;
    }
    void brakePublish(){
        ros::Rate pub_rate(brake_rate_);
        int count = 0;
        while(count < brake_duration_*brake_rate_)
        {
            count++;
            brake_pub_.publish(ackermann_msg_);
            brake_bool_pub_.publish(brake_bool_);
            pub_rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{   
    ros::init(argc,argv,"safety");
    Safety safe;
    ros::spin();
}
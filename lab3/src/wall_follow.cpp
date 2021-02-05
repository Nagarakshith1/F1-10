#include <iostream>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <queue>
#include"ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

class WallFollow
{
    ros::Publisher nav_pub_;
    ros::Subscriber scan_sub_;
    ros::NodeHandle n_;

    public:
    int look_ahead_ = 1.2;
    int angle_increment_ = 0.00582315586;
    int ray_count_ = 1080;
    std::vector<int> a_b_angles_ = {30,40,50,60};
    // std::vector<int> a_b_angles_ = {30,40,50};
    int prev_error_ = 0;
    float k_p_ = 0.5;
    float k_d_ = 0.01;
    float k_i_ = 0.0001;
    float des_dist_ = 0.55; 
    float cum_error = 0;
    

    WallFollow()
    {
        scan_sub_ = n_.subscribe("scan",1,&WallFollow::processLidarScan,this);
        nav_pub_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);
    }
    void processLidarScan(const sensor_msgs::LaserScanConstPtr &scan_msg)
    {
        int b_ray_ind = ray_count_ - ray_count_ / 4 - 1;
        float b_range = scan_msg->ranges[b_ray_ind];
        float cum_alpha = 0;
        int count = 0;
        for(auto angle:a_b_angles_)
        {
            float a_range = scan_msg->ranges[b_ray_ind - 3*angle];
            float num = a_range * std::cos(angle * M_PI/180) - b_range;
            float den = a_range * std::sin(angle * M_PI/180);
            float temp_alpha = std::atan2(num,den);
            if(temp_alpha > - 1.22173 && temp_alpha < 1.22173) //Constrainting between 70 and -70 degree
            {   
                count++;
                cum_alpha += temp_alpha;
            }
        }
        float avg_alpha = count !=0 ? cum_alpha / count : 0;
        float dist = b_range * std::cos(avg_alpha) + look_ahead_ * std::sin(avg_alpha);
        controller(dist);
    }
    void controller(int dist)
    {
        float error = dist - des_dist_;
        cum_error += error;
        ROS_INFO_STREAM("cum error "<< cum_error <<std::endl);
        float steer_angle = k_p_ * (error) + k_d_ * (error - prev_error_) + k_i_ * cum_error;
        prev_error_ = error;

        float velocity = 0;
        if(std::abs(steer_angle)< 10)
        {
            velocity = 1.5;
        }
        else if(std::abs(steer_angle) > 10 && std::abs(steer_angle) < 20)
        {
            velocity = 1;
        }
        else
        {
            velocity = 0.5;
        }
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.frame_id = "laser";
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = steer_angle;
        nav_pub_.publish(drive_msg);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wall_follow");
    WallFollow wf;
    ros::spin();
}
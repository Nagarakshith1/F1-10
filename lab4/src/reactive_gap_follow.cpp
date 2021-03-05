#include <iostream>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <queue>
#include"ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

class ReactiveGapFollow
{
    ros::Publisher nav_pub_;
    ros::Subscriber scan_sub_;
    ros::NodeHandle n_;
    float step_size_ = 0.00582315586;
    int start_ = 1080/4;
    int end_ = start_ + 270*2;
    float thresh_dist_ = 0.5;

    public:
    ReactiveGapFollow()
    {
        scan_sub_ = n_.subscribe("scan",1,&ReactiveGapFollow::processLidarScan,this);
        nav_pub_ = n_.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);
    }

    void processLidarScan(const sensor_msgs::LaserScanConstPtr &scan_msg)
    {
        
        
        int min_dist_id = start_;
        std::vector<float> scan_msg_cpy (1080,0);
        scan_msg_cpy[start_-1] = scan_msg->ranges[start_-1];
        
        for(int i = start_; i < end_; i++)
        {
            scan_msg_cpy[i] = scan_msg->ranges[i];
            if(scan_msg->ranges[i] < scan_msg->ranges[min_dist_id])
            {
                min_dist_id = i;
            }

        }
        removeClosest(scan_msg_cpy,min_dist_id);
        float steer_angle = getSteer(scan_msg_cpy);
        ROS_INFO_STREAM("Steer angle is: "<<steer_angle);
        controller(steer_angle);
    }

    void removeClosest(std::vector<float> &ranges,int min_dist_id)
    {
        float x = ranges[min_dist_id] * std::cos(min_dist_id * step_size_ - M_PI_2);
        float y = ranges[min_dist_id] * std::sin(min_dist_id * step_size_ - M_PI_2);
        for(int j=start_; j < end_; j++)
        {
            if(!ranges[j]) continue;
            float other_x  = ranges[j] * std::cos(j * step_size_ - M_PI_2);
            float other_y  = ranges[j] * std::sin(j * step_size_ - M_PI_2);
            float dist = std::pow(std::pow(x-other_x,2) + std::pow(y-other_y,2),0.5);
            if(dist<radius_thresh_)
            {
                ranges[j] = 0;
            }
        }
    }

    float getSteer(std::vector<float> &ranges)
    {
        std::vector<std::vector<int>> ids;
        int last_id = 0;
        for(int k = start_; k<end_; k++)
        {
            if(ranges[k] > thresh_dist_)
            {
                if(k - last_id > 1)
                {
                    ids.push_back({});
                }
                ids[ids.size()-1].push_back(k);
                last_id = k;
            }
        }
        int max_size = 0;
        int max_id;
        for(int j = 0; j < ids.size(); j++)
        {
            if(ids[j].size() > max_size)
            {
                max_size = ids[j].size();
                max_id = j;
            }
        }
        float angle = ids[max_id][static_cast<int>(max_size/2)] * step_size_ - M_PI;
        return angle;
    }

    void controller(float steer_angle)
    {
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
    float radius_thresh_ = 0.3;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"reactive_gap_follow");
    ReactiveGapFollow rg;
    ros::spin();
}
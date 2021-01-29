#include<iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "lab1/scan_range.h"

class LidarRange{
    private:
    ros::NodeHandle n;
    ros::Subscriber scan_sub;
    ros::Publisher close_point_pub;
    ros::Publisher far_point_pub;
    ros::Publisher scan_range_pub;
    lab1::scan_range sc_msg;
    
    public:
    LidarRange(){
        scan_sub = n.subscribe("scan",1,&LidarRange::scan_cb,this);
        close_point_pub = n.advertise<std_msgs::Float64>("closest_point",1);
        far_point_pub = n.advertise<std_msgs::Float64>("farthest_point",1);
        scan_range_pub = n.advertise<lab1::scan_range>("scan_range",1);

    }

    void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
        std_msgs::Float64 range_max;
        std_msgs::Float64 range_min;
        range_max.data = msg->range_min - 1;
        range_min.data = msg->range_max + 1;
        for(auto dist : msg->ranges){
            if(!std::isinf(dist)&& !std::isnan(dist)){
                if(dist>range_max.data){
                    range_max.data = dist;
                }
                else if(dist < range_min.data){
                    range_min.data = dist;
                }
            }
        }
        
        close_point_pub.publish(range_min);
        far_point_pub.publish(range_max);
        sc_msg.header.stamp = ros::Time::now();
        sc_msg.header.seq ++;
        sc_msg.min = range_min.data;
        sc_msg.max = range_max.data;
        scan_range_pub.publish(sc_msg);
    }
};

int main(int argc, char **argv){
    ros::init(argc,argv,"lidar_processing");
    LidarRange lidar;
    ros::spin();
}
#include<iostream>
#include<cmath>
#include<unordered_map>
#include<vector>
#include"ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

class GetDistancesToEgoEdges{
    float width_;
    float height_;
    float aspect_ratio_angle_;
    float ray_count_;
    std::unordered_map<int,std::vector<float>> line_cords_;

    float inter_x_;
    float inter_y_;
    float x2_;
    float y2_;
    float x3_;
    float y3_;
    float x4_;
    float y4_;

    void getInter(float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4)
    {
        inter_x_ = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4-y3*x4))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
        inter_y_ = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4-y3*x4))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
    }
    int getLineId(float th)
    {
        if(th< (- M_PI_2 + aspect_ratio_angle_)|| th > (3*M_PI_2 - aspect_ratio_angle_))
            {
                return 1;
            }
            else if(th > (M_PI_2+ aspect_ratio_angle_) && th < (3*M_PI_2 - aspect_ratio_angle_))
            {
                return 2;
            }
            else if(th > (M_PI_2 - aspect_ratio_angle_) && th < (M_PI_2 + aspect_ratio_angle_))
            {
                return 3;
            }
            else if(th > (-M_PI_2+aspect_ratio_angle_) && th<(M_PI_2 - aspect_ratio_angle_))
            {
                return 4;
            }
            return 0;
    }
    
    public:
    float x1_ = 0.0;
    float y1_ = 0.0;
    float lidar_to_ego_center_ = 0.0;
    
    float start_th_ = 3 * M_PI_2;
    float step_size_ = 0.00582315586;
    
    std::vector<float> x_cords_ego_;
    std::vector<float> y_cords_ego_;
    std::vector<float> dist_to_ego_edge_;

    GetDistancesToEgoEdges(float w, float h,float ray_count):width_{w},height_{h},ray_count_{ray_count}
    {
        aspect_ratio_angle_ = std::atan(width_/height_);
        line_cords_[1] = {0,-height_/2,1,-height_/2};
        line_cords_[2] = {-width_/2,0,-width_/2,1};
        line_cords_[3] = {0,height_/2,1,height_/2};
        line_cords_[4] = {width_/2,0,width_/2,1};
    }
    void getEgoEdgeCords()
    {
        float th = start_th_ + step_size_;
        for(int i=0; i<ray_count_; i++)
        {
            th -= step_size_;
            x2_ = 1 * std::cos(th);
            y2_ = 1 * std::sin(th);
            int id = getLineId(th);
            x3_ = line_cords_[id][0];
            y3_ = line_cords_[id][1];
            x4_ = line_cords_[id][2];
            y4_ = line_cords_[id][3];

            getInter(x1_,y1_,x2_,y2_,x3_,y3_,x4_,y4_);
            x_cords_ego_.push_back(inter_x_);
            y_cords_ego_.push_back(inter_y_);
        }
    }
    void getEgoEdgeLidarDist()
    {
        getEgoEdgeCords();
        for(int i=0;i<x_cords_ego_.size();i++)
        {   
            float x = x_cords_ego_[i];
            float y = y_cords_ego_[i] - lidar_to_ego_center_;
            dist_to_ego_edge_.push_back(std::sqrt(x*x + y*y));
        }

    }
    

};


int main(int argc, char** argv)
{   
    ros::init(argc,argv,"safety");
    ros::NodeHandle n;

    float width = 0.2032;
    float height = 0.3302;
    float lidar_to_base_link = 0.275;
    float ray_count = 1080;

    n.getParam("width",width);
    n.getParam("wheelbase",height);
    n.getParam("scan_distance_to_base_link",lidar_to_base_link);
    n.getParam("scan_beams",ray_count);

    GetDistancesToEgoEdges dist(width,height,ray_count);
    dist.lidar_to_ego_center_ = lidar_to_base_link - height/2;
    dist.getEgoEdgeLidarDist();

    scan_sub = n.subscribe("scan",1,&LidarRange::scan_cb,this);
    vel_sub = n.subscribe("")

    
}
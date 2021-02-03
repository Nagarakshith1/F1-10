#include "lab2/get_distances_to_ego_edges.h"

void GetDistancesToEgoEdges::getInter(float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4)
{
    inter_x_ = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4-y3*x4))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
    inter_y_ = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4-y3*x4))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
}
int GetDistancesToEgoEdges::getLineId(float th)
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
GetDistancesToEgoEdges::GetDistancesToEgoEdges(float w, float h,float ray_count):width_{w},height_{h},ray_count_{ray_count}
{
    aspect_ratio_angle_ = std::atan(width_/height_);
    line_cords_[1] = {0,-height_/2,1,-height_/2};
    line_cords_[2] = {-width_/2,0,-width_/2,1};
    line_cords_[3] = {0,height_/2,1,height_/2};
    line_cords_[4] = {width_/2,0,width_/2,1};
}
void GetDistancesToEgoEdges::getEgoEdgeCords()
{
    float th = start_th_ + step_size_;
    for(int i=0; i<ray_count_; i++)
    {
        th -= step_size_;
        ray_angles_.push_back(th);
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
    std::reverse(ray_angles_.begin(),ray_angles_.end());
}
void GetDistancesToEgoEdges::getEgoEdgeLidarDist()
{
    getEgoEdgeCords();
    for(int i=0;i<x_cords_ego_.size();i++)
    {   
        float x = x_cords_ego_[i];
        float y = y_cords_ego_[i] - lidar_to_ego_center_;
        dist_to_ego_edge_.push_back(std::sqrt(x*x + y*y));
    }
    std::reverse(dist_to_ego_edge_.begin(),dist_to_ego_edge_.end());
}
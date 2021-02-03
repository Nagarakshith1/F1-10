#ifndef LAB2_GET_DISTANCES_TO_EGO_EDGES_H
#define LAB2_GET_DISTANCES_TO_EGO_EDGES_H
#include<vector>
#include<unordered_map>
#include<cmath>
#include <algorithm>
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

    void getInter(float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4);
    int getLineId(float th);
    
    public:
    
    float x1_ = 0.0;
    float y1_ = 0.0;
    float lidar_to_ego_center_ = 0.0;  
    float start_th_ = 3 * M_PI_2;
    float step_size_ = 0.00582315586;

    std::vector<float> ray_angles_; 
    std::vector<float> x_cords_ego_;
    std::vector<float> y_cords_ego_;
    std::vector<float> dist_to_ego_edge_;

    GetDistancesToEgoEdges(){}
    GetDistancesToEgoEdges(float w, float h,float ray_count);
    void getEgoEdgeCords();
    void getEgoEdgeLidarDist();
};

#endif
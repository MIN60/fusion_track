#ifndef FUSION_TRACK_H
#define FUSION_TRACK_H

#include <ros/ros.h>
#include <cam_lidar_calib/sf_Info.h>

#include "fusion_track/visual_marker.h"
#include "fusion_track/TrackCenter.h" 
#include "fusion_track/IsCurve.h"
#include "fusion_track/ReadyToCurve.h"


class FusionTrack
{
public:
    FusionTrack(ros::NodeHandle &nh); // 생성자 업데이트

    //double getLaneCenterY(); // 라바콘 중심 y 값을 반환
    //std::pair<double, double> getLaneCenterY();
    fusion_track::TrackCenter getLaneCenterY();


private:
    void sfInfoCallback(const cam_lidar_calib::sf_Info::ConstPtr &msg);
    // VisualMarkerPtr visual_marker;

    std::unique_ptr<VisualMarker> visual_marker;

    ros::NodeHandle nh_;
    ros::Subscriber sf_info_sub_;
    ros::Publisher y_center_diff_pub_; // Publisher for track center value
    ros::Publisher curve_pub_;
    ros::Publisher ready_curve_pub_;


    double track_center_x;               // 중앙 차선의 y 좌표값
    double track_center_y;               // 중앙 차선의 y 좌표값
    double cone_interval;   // 콘과 중앙점 사이 최소 간격
    double x_Thres; //x값 유사도 판별, 임계값
    double pair_yellow_x; //x값 유사 노란 라바콘
    double pair_blue_x; //x값 유사 파란 라바콘
    double pair_yellow_y; //x값 유사 노란 라바콘
    double pair_blue_y; //x값 유사 파란 라바콘
    double smallest_diff; //최소 깊이
    double x_diff; // 두 라바콘의 x값 차이 계산
    int iscurve;
    double DISTANCE_LIMIT;


    double prev_yellow_x;
    double prev_yellow_y;
    double prev_blue_x;
    double prev_blue_y;
    bool start_yellow;
    bool start_blue;


    fusion_track::TrackCenter track_center_msg;


    bool Ready = false;
    double ReadyToCurve_x_y;
    double ReadyToCurve_y_y;
    double ReadyToCurve_x_b;
    double ReadyToCurve_y_b;


};

#endif
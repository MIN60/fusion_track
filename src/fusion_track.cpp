#include "fusion_track/fusion_track.h"
#include "fusion_track/visual_marker.h"
#include <iostream>
#include <std_msgs/Float64.h>
#include "fusion_track/TrackCenter.h" 
#include "fusion_track/ReadyToCurve.h"


const int YELLOW_CONE_ID = 2; //yolo에서 노란색 class 번호 2
const int BLUE_CONE_ID = 0; //yolo에서 파란색 class 번호 0


FusionTrack::FusionTrack(ros::NodeHandle& nh)
    : nh_(nh), track_center_x(0.0), track_center_y(0.0)
{
    sf_info_sub_ = nh_.subscribe("/sf_info", 5, &FusionTrack::sfInfoCallback, this);
   // y_center_diff_pub_ = nh_.advertise<std_msgs::Float64>("/y_center_diff", 1);
    y_center_diff_pub_ = nh_.advertise<fusion_track::TrackCenter>("/y_center_diff", 5);
    curve_pub_ = nh_.advertise<fusion_track::IsCurve>("/is_curve", 5);
    ready_curve_pub_ = nh_.advertise<fusion_track::ReadyToCurve>("/ready_curve", 5);
    


    visual_marker = std::make_unique<VisualMarker>(nh_);
}


// 둘 다 보일 때 노란색 따라가고 하나만 보이면 보이는 색깔 따라가기
void FusionTrack::sfInfoCallback(const cam_lidar_calib::sf_Info::ConstPtr& msg)
{
    double DISTANCE_LIMIT = 15.0;
    int iscurve = 0; //0이 직진, 1이 우회전 2가 좌회전
    double nearest_yellow_x = std::numeric_limits<double>::max();
    //double nearest_yellow_y;
    double nearest_yellow_y = std::numeric_limits<double>::max();  
    
    
    double nearest_blue_x = std::numeric_limits<double>::max();
    //double nearest_blue_y;
    double nearest_blue_y = std::numeric_limits<double>::max();  
    

    bool detected_yellow = false;
    bool detected_blue = false;

    double cone_interval = 0.2;

    bool Ready = false;

    for (int i = 0; i < msg->num_boxes; i++) {
        if (msg->pub_id[i] == YELLOW_CONE_ID && msg->sf_x[i] < DISTANCE_LIMIT && msg->sf_x[i] != 0.0 && msg->sf_x[i] < nearest_yellow_x) {
            detected_yellow = true;
            nearest_yellow_x = msg->sf_x[i];
            nearest_yellow_y = msg->sf_y[i];

            if (msg->sf_y[i] >= -1.0 && msg->sf_y[i] <= 1.0 && msg->sf_x[i]<=4.0) {
                ReadyToCurve_x_y = msg->sf_x[i];
                ReadyToCurve_y_y = msg->sf_y[i];
                Ready = true;
                ROS_WARN("readyYellow");
            }
            else{
                Ready = false;
            }

        } 
        else if (msg->pub_id[i] == BLUE_CONE_ID && msg->sf_x[i] < DISTANCE_LIMIT && msg->sf_x[i] != 0.0 && msg->sf_x[i] < nearest_blue_x) {
            detected_blue = true;
            nearest_blue_x = msg->sf_x[i];
            nearest_blue_y = msg->sf_y[i];

            if (msg->sf_y[i] >= -1.0 && msg->sf_y[i] <= 1.0 && msg->sf_x[i]<=4.0) {
                ReadyToCurve_x_b = msg->sf_x[i];
                ReadyToCurve_y_b = msg->sf_y[i];
                Ready = true;
                ROS_WARN("readyBlue");
            }
            else{
                Ready = false;
            }
        }
    }

    if (Ready) {
        fusion_track::ReadyToCurve ReadyToCurve;
        ReadyToCurve.Ready = Ready;
        ReadyToCurve.ReadyToCurve_x_y = ReadyToCurve_x_y;
        ReadyToCurve.ReadyToCurve_y_y = ReadyToCurve_y_y;
        ReadyToCurve.ReadyToCurve_x_b = ReadyToCurve_x_b;
        ReadyToCurve.ReadyToCurve_y_b = ReadyToCurve_y_b;
        ready_curve_pub_.publish(ReadyToCurve);
    }
    else{
        fusion_track::ReadyToCurve ReadyToCurve;
        ReadyToCurve.Ready = Ready;
        ready_curve_pub_.publish(ReadyToCurve);
    }

    

    // 라바콘 둘 다 보일 경우 노란색만 따라감
    if(detected_yellow && detected_blue){
        //if(nearest_yellow_x != 0 && nearest_blue_x != 0){    
        ROS_WARN("++++++2CONE DETECT+++++++");
        ROS_INFO("Original Yellow cone: %f", nearest_yellow_y);
        ROS_INFO("Original Blue cone: %f", nearest_blue_y);
        // if(nearest_yellow_y >= 0){
        //     nearest_blue_y = -(nearest_yellow_y+cone_interval); // 반대편 대칭에 파란색
        //     nearest_blue_x = nearest_yellow_x;
        // }
        // else{
        //     nearest_blue_y = -(nearest_yellow_y-cone_interval); // 반대편 대칭에 파란색
        //     nearest_blue_x = nearest_yellow_x;
        // }
        track_center_y = (nearest_yellow_y + nearest_blue_y) / 2.0;
        track_center_x = (nearest_yellow_x + nearest_blue_x) / 2.0;
        iscurve = 0;
        //}
    }

    // 곡선 주행(한 쪽 라바콘만 보일 경우)
    // 노란색만 보일 경우
    // std::numeric_limits<double>::max()
    else if (detected_yellow && !detected_blue) {
        //if(nearest_yellow_x != 0 && nearest_blue_x == 0){ 
        ROS_WARN("+++++++++1CONE(Yellow) DETECT++++++++++");
        iscurve = 1;
        
        if(nearest_yellow_y >=-0.5 && nearest_yellow_y<=0.5){
            ROS_INFO("노란색 라바콘 임");
        }
        // double 타입의 최대값
        // nearest_blue_y = -nearest_yellow_y; // 반대편 대칭에 파란색
        // nearest_blue_x = nearest_yellow_x;
        ROS_INFO("Original Yellow cone: %f", nearest_yellow_y);
        ROS_INFO("Original Blue cone: %f", nearest_blue_y);
        ROS_WARN("CURVE");

        if(nearest_yellow_y >= 0){
            nearest_blue_y = -(nearest_yellow_y+cone_interval); // 반대편 대칭에 파란색
            nearest_blue_x = nearest_yellow_x;
        }
        else{
            nearest_blue_y = -(nearest_yellow_y-cone_interval); // 반대편 대칭에 파란색
            nearest_blue_x = nearest_yellow_x;
        }
        iscurve = 1;

        //}
    }

    // 파란색만 보일 경우
    else if (detected_blue && !detected_yellow) {
        //if(nearest_yellow_x == 0 && nearest_blue_x != 0){ 
        ROS_WARN("+++++++++1CONE(Blue) DETECT++++++++++");
        //nearest_yellow_y = -nearest_blue_y; // 반대편 대칭에 노란색
        //nearest_yellow_x = nearest_blue_x;
        iscurve = 2;
        ROS_INFO("Original Blue cone: %f", nearest_blue_y);
        ROS_WARN("CURVE");
        if(nearest_blue_y >= 0){
            nearest_yellow_y = -(nearest_blue_y+cone_interval); // 반대편 대칭에 노란색
            nearest_yellow_x = nearest_blue_x;
        }
        else{
            nearest_yellow_y = -(nearest_blue_y-cone_interval); // 반대편 대칭에 노란색
            nearest_yellow_x = nearest_blue_x;
        }
        iscurve = 2;
        //}
    }

    track_center_y = (nearest_yellow_y + nearest_blue_y) / 2.0;
    track_center_x = (nearest_yellow_x + nearest_blue_x) / 2.0;

    fusion_track::TrackCenter track_center_msg;


    track_center_msg.x = track_center_x;
    track_center_msg.y = track_center_y;
    
    // 메시지 발행
    y_center_diff_pub_.publish(track_center_msg);

    ROS_INFO("Track center: %f , %f", track_center_x, track_center_y);

    visual_marker->publishMarkers(nearest_yellow_x, nearest_yellow_y, nearest_blue_x, nearest_blue_y,track_center_x, track_center_y);

    fusion_track::IsCurve curve_msg;
    curve_msg.data = iscurve;
    curve_pub_.publish(curve_msg);

}




fusion_track::TrackCenter FusionTrack::getLaneCenterY()
{
    return track_center_msg;
}
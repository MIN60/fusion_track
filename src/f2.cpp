#include "fusion_track/fusion_track.h"
#include "fusion_track/visual_marker.h"
#include <iostream>
#include <std_msgs/Float64.h>
#include "fusion_track/TrackCenter.h"  // TrackCenter 메시지를 포함시키기

const int YELLOW_CONE_ID = 2; //yolo에서 노란색 class 번호 2
const int BLUE_CONE_ID = 0; //yolo에서 파란색 class 번호 0


FusionTrack::FusionTrack(ros::NodeHandle& nh)
    : nh_(nh), track_center_x(0.0), track_center_y(0.0)
{
    sf_info_sub_ = nh_.subscribe("/sf_info", 5, &FusionTrack::sfInfoCallback, this);
   // y_center_diff_pub_ = nh_.advertise<std_msgs::Float64>("/y_center_diff", 1);
    y_center_diff_pub_ = nh_.advertise<fusion_track::TrackCenter>("/y_center_diff", 5);
    curve_pub_ = nh_.advertise<fusion_track::IsCurve>("is_curve", 5);
    


    visual_marker = std::make_unique<VisualMarker>(nh_);
}

/*
// 둘 다 보이면 중점, 하나만 보이면 그 색 따라가기
void FusionTrack::sfInfoCallback(const cam_lidar_calib::sf_Info::ConstPtr& msg)
{
    bool iscurve = false;
    double nearest_yellow_x = std::numeric_limits<double>::max();
    double nearest_yellow_y;
    
    double nearest_blue_x = std::numeric_limits<double>::max();
    double nearest_blue_y;

    double cone_interval = 0.5;

    for (int i = 0; i < msg->num_boxes; i++) {
        if (msg->pub_id[i] == YELLOW_CONE_ID && msg->sf_x[i] < nearest_yellow_x) {
            nearest_yellow_x = msg->sf_x[i];
            nearest_yellow_y = msg->sf_y[i];
        } else if (msg->pub_id[i] == BLUE_CONE_ID && msg->sf_x[i] < nearest_blue_x) {
            nearest_blue_x = msg->sf_x[i];
            nearest_blue_y = msg->sf_y[i];
        }
    }



    // 곡선 주행(한 쪽 라바콘만 보일 경우)
    // 노란색만 보일 경우
    if (nearest_yellow_x != std::numeric_limits<double>::max() && nearest_blue_x == std::numeric_limits<double>::max()) {
        // double 타입의 최대값
        // nearest_blue_y = -nearest_yellow_y; // 반대편 대칭에 파란색
        // nearest_blue_x = nearest_yellow_x;
        ROS_INFO("Original Yellow cone: %f", nearest_yellow_y);
        if(nearest_yellow_y >= 0){
            nearest_blue_y = -(nearest_yellow_y+cone_interval); // 반대편 대칭에 파란색
            nearest_blue_x = nearest_yellow_x;
        }
        else{
            nearest_blue_y = -(nearest_yellow_y-cone_interval); // 반대편 대칭에 파란색
            nearest_blue_x = nearest_yellow_x;
        }
        iscurve = true;

    }

    // 파란색만 보일 경우
    else if (nearest_blue_x != std::numeric_limits<double>::max() && nearest_yellow_x == std::numeric_limits<double>::max()) {
        //nearest_yellow_y = -nearest_blue_y; // 반대편 대칭에 노란색
        //nearest_yellow_x = nearest_blue_x;
        ROS_INFO("Original Blue cone: %f", nearest_blue_y);
        if(nearest_blue_y >= 0){
            nearest_yellow_y = -(nearest_blue_y+cone_interval); // 반대편 대칭에 노란색
            nearest_yellow_x = nearest_blue_x;
        }
        else{
            nearest_yellow_y = -(nearest_blue_y-cone_interval); // 반대편 대칭에 노란색
            nearest_yellow_x = nearest_blue_x;
        }
        iscurve = true;
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
*/

// 둘 다 보일 때 노란색 따라가고 하나만 보이면 보이는 색깔 따라가기
void FusionTrack::sfInfoCallback(const cam_lidar_calib::sf_Info::ConstPtr& msg)
{
    double DISTANCE_LIMIT = 5.0;
    int iscurve = 0; //0이 직진, 1이 우회전 2가 좌회전
    double nearest_yellow_x = std::numeric_limits<double>::max();
    //double nearest_yellow_y;
    double nearest_yellow_y = std::numeric_limits<double>::max();  
    
    
    double nearest_blue_x = std::numeric_limits<double>::max();
    //double nearest_blue_y;
    double nearest_blue_y = std::numeric_limits<double>::max();  
    

    bool detected_yellow = false;
    bool detected_blue = false;

    double cone_interval = 0.3;

    for (int i = 0; i < msg->num_boxes; i++) {
        if (msg->pub_id[i] == YELLOW_CONE_ID && msg->sf_x[i] < DISTANCE_LIMIT && msg->sf_x[i] != 0.0 && msg->sf_x[i] < nearest_yellow_x) {
            detected_yellow = true;
            nearest_yellow_x = msg->sf_x[i];
            nearest_yellow_y = msg->sf_y[i];
        } 
        else if (msg->pub_id[i] == BLUE_CONE_ID && msg->sf_x[i] < DISTANCE_LIMIT && msg->sf_x[i] != 0.0 && msg->sf_x[i] < nearest_blue_x) {
            detected_blue = true;
            nearest_blue_x = msg->sf_x[i];
            nearest_blue_y = msg->sf_y[i];
        }
    }


    // 라바콘 둘 다 보일 경우 노란색만 따라감
    if(detected_yellow && detected_blue){
        //if(nearest_yellow_x != 0 && nearest_blue_x != 0){    
        ROS_WARN("++++++2CONE DETECT+++++++");
        ROS_INFO("Original Yellow cone: %f", nearest_yellow_y);
        ROS_INFO("Original Blue cone: %f", nearest_blue_y);
        if(nearest_yellow_y >= 0){
            nearest_blue_y = -(nearest_yellow_y+cone_interval); // 반대편 대칭에 파란색
            nearest_blue_x = nearest_yellow_x;
        }
        else{
            nearest_blue_y = -(nearest_yellow_y-cone_interval); // 반대편 대칭에 파란색
            nearest_blue_x = nearest_yellow_x;
        }
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



/*
// 비슷한 라인(x값 유사)에 있는 라바콘 사이 중점 찍기
// 둘 다 보이는 경우 유사한 x값을 가진 라바콘 쌍의 중심을 찾습니다.
void FusionTrack::sfInfoCallback(const cam_lidar_calib::sf_Info::ConstPtr& msg)
{
    bool iscurve = false;
    
    double cone_interval = 0.5; // 콘과 중점 사이 최소 유지간격

    // x값 유사 쌍 찾기
    double x_Thres = 0.5; //노란색, 파란색 라바콘 쌍 사이 차이 임계값

    double pair_yellow_x = std::numeric_limits<double>::max();
    double pair_yellow_y;
    double pair_blue_x = std::numeric_limits<double>::max();
    double pair_blue_y;

    double smallest_diff = std::numeric_limits<double>::max();

    for (int i = 0; i < msg->num_boxes; i++) {
        for (int j = 0; j < msg->num_boxes; j++) {
            if (msg->pub_id[i] == YELLOW_CONE_ID && msg->pub_id[j] == BLUE_CONE_ID) {
                double x_diff = std::abs(msg->sf_x[i] - msg->sf_x[j]);
                
                if (x_diff < x_Thres && x_diff < smallest_diff) {
                    pair_yellow_x = msg->sf_x[i];
                    pair_yellow_y = msg->sf_y[i];
                    pair_blue_x = msg->sf_x[j];
                    pair_blue_y = msg->sf_y[j];
                    smallest_diff = x_diff;
                }
            }
        }
    }


    //가까운 라바콘 사이 찍기
    double nearest_yellow_x = std::numeric_limits<double>::max();
    double nearest_yellow_y;
    
    double nearest_blue_x = std::numeric_limits<double>::max();
    double nearest_blue_y;

    for (int i = 0; i < msg->num_boxes; i++) {
        if (msg->pub_id[i] == YELLOW_CONE_ID && msg->sf_x[i] < nearest_yellow_x) {
            nearest_yellow_x = msg->sf_x[i];
            nearest_yellow_y = msg->sf_y[i];
        } else if (msg->pub_id[i] == BLUE_CONE_ID && msg->sf_x[i] < nearest_blue_x) {
            nearest_blue_x = msg->sf_x[i];
            nearest_blue_y = msg->sf_y[i];
        }
    }



    // 라바콘 둘 다 보일 경우
    if(nearest_yellow_x != std::numeric_limits<double>::max() && nearest_blue_x != std::numeric_limits<double>::max()){
        ROS_INFO("Original Yellow cone: %f", nearest_yellow_y);
        ROS_INFO("Original Blue cone: %f", nearest_blue_x);

        if (smallest_diff != std::numeric_limits<double>::max()) {
            track_center_y = (pair_yellow_y + pair_blue_y) / 2.0;
            track_center_x = (pair_yellow_x + pair_blue_x) / 2.0;
        }
        else{
            ROS_INFO("++++++++++++++임계값 조절 필요함!!+++++++++++++");
        }
        
    }

    // 곡선 주행(한 쪽 라바콘만 보일 경우)
    // 노란색만 보일 경우
    else if (nearest_yellow_x != std::numeric_limits<double>::max() && nearest_blue_x == std::numeric_limits<double>::max()) {
        // double 타입의 최대값
        // nearest_blue_y = -nearest_yellow_y; // 반대편 대칭에 파란색
        // nearest_blue_x = nearest_yellow_x;
        ROS_INFO("Original Yellow cone: %f", nearest_yellow_y);
        if(nearest_yellow_y >= 0){
            nearest_blue_y = -(nearest_yellow_y+cone_interval); // 반대편 대칭에 파란색
            nearest_blue_x = nearest_yellow_x;
        }
        else{
            nearest_blue_y = -(nearest_yellow_y-cone_interval); // 반대편 대칭에 파란색
            nearest_blue_x = nearest_yellow_x;
        }
        track_center_y = (nearest_yellow_y + nearest_blue_y) / 2.0;
        track_center_x = (nearest_yellow_x + nearest_blue_x) / 2.0;
        iscurve = true;


    }

    // 파란색만 보일 경우
    else if (nearest_blue_x != std::numeric_limits<double>::max() && nearest_yellow_x == std::numeric_limits<double>::max()) {
        //nearest_yellow_y = -nearest_blue_y; // 반대편 대칭에 노란색
        //nearest_yellow_x = nearest_blue_x;
        ROS_INFO("Original Blue cone: %f", nearest_blue_y);
        if(nearest_blue_y >= 0){
            nearest_yellow_y = -(nearest_blue_y+cone_interval); // 반대편 대칭에 노란색
            nearest_yellow_x = nearest_blue_x;
        }
        else{
            nearest_yellow_y = -(nearest_blue_y-cone_interval); // 반대편 대칭에 노란색
            nearest_yellow_x = nearest_blue_x;
        }
        track_center_y = (nearest_yellow_y + nearest_blue_y) / 2.0;
        track_center_x = (nearest_yellow_x + nearest_blue_x) / 2.0;
        iscurve = true;

    }

    
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

*/


/*
// 쌍 찾는 건데 못찾으면 보이는 녀석들 따라가도록
// 비슷한 라인(x값 유사)에 있는 라바콘 사이 중점 찍기
// 둘 다 보이는 경우 유사한 x값을 가진 라바콘 쌍의 중심을 찾습니다.
void FusionTrack::sfInfoCallback(const cam_lidar_calib::sf_Info::ConstPtr& msg)
{
    bool iscurve = false;
    
    double cone_interval = 0.5; // 콘과 중점 사이 최소 유지간격

    // x값 유사 쌍 찾기
    double x_Thres = 0.3; //노란색, 파란색 라바콘 쌍 사이 차이 임계값

    double pair_yellow_x = std::numeric_limits<double>::max();
    double pair_yellow_y;
    double pair_blue_x = std::numeric_limits<double>::max();
    double pair_blue_y;

    double smallest_diff = std::numeric_limits<double>::max();

    for (int i = 0; i < msg->num_boxes; i++) {
        for (int j = 0; j < msg->num_boxes; j++) {
            if (msg->pub_id[i] == YELLOW_CONE_ID && msg->pub_id[j] == BLUE_CONE_ID) {
                double x_diff = std::abs(msg->sf_x[i] - msg->sf_x[j]);
                
                if (x_diff < x_Thres && x_diff < smallest_diff) {
                    pair_yellow_x = msg->sf_x[i];
                    pair_yellow_y = msg->sf_y[i];
                    pair_blue_x = msg->sf_x[j];
                    pair_blue_y = msg->sf_y[j];
                    smallest_diff = x_diff;
                }
            }
        }
    }


    //가까운 라바콘 사이 찍기
    double nearest_yellow_x = std::numeric_limits<double>::max();
    double nearest_yellow_y;
    
    double nearest_blue_x = std::numeric_limits<double>::max();
    double nearest_blue_y;

    for (int i = 0; i < msg->num_boxes; i++) {
        if (msg->pub_id[i] == YELLOW_CONE_ID && msg->sf_x[i] < nearest_yellow_x) {
            nearest_yellow_x = msg->sf_x[i];
            nearest_yellow_y = msg->sf_y[i];
        } else if (msg->pub_id[i] == BLUE_CONE_ID && msg->sf_x[i] < nearest_blue_x) {
            nearest_blue_x = msg->sf_x[i];
            nearest_blue_y = msg->sf_y[i];
        }
    }



    // 라바콘 둘 다 보일 경우
    if(nearest_yellow_x != std::numeric_limits<double>::max() && nearest_blue_x != std::numeric_limits<double>::max()){
        ROS_INFO("Original Yellow cone: %f", nearest_yellow_y);
        ROS_INFO("Original Blue cone: %f", nearest_blue_x);

        if (smallest_diff != std::numeric_limits<double>::max()) {
            track_center_y = (pair_yellow_y + pair_blue_y) / 2.0;
            track_center_x = (pair_yellow_x + pair_blue_x) / 2.0;
            ROS_WARN("pair yellow x: %f, pair yellow y: %f", pair_yellow_x, pair_yellow_y);
            ROS_WARN("pair blue x: %f, pair blue y: %f", pair_blue_x, pair_blue_y);
        }
        else{
            ROS_INFO("++++++++++++++임계값 조절 필요함!!+++++++++++++");
            // 곡선 주행(한 쪽 라바콘만 보일 경우)
            // 노란색만 보일 경우
            if (nearest_yellow_x != std::numeric_limits<double>::max() && nearest_blue_x == std::numeric_limits<double>::max()) {
                // double 타입의 최대값
                // nearest_blue_y = -nearest_yellow_y; // 반대편 대칭에 파란색
                // nearest_blue_x = nearest_yellow_x;
                ROS_INFO("Original Yellow cone: %f", nearest_yellow_y);
                if(nearest_yellow_y >= 0){
                    nearest_blue_y = -(nearest_yellow_y+cone_interval); // 반대편 대칭에 파란색
                    nearest_blue_x = nearest_yellow_x;
                }
                else{
                    nearest_blue_y = -(nearest_yellow_y-cone_interval); // 반대편 대칭에 파란색
                    nearest_blue_x = nearest_yellow_x;
                }
                track_center_y = (nearest_yellow_y + nearest_blue_y) / 2.0;
                track_center_x = (nearest_yellow_x + nearest_blue_x) / 2.0;
                iscurve = true;


            }

            // 파란색만 보일 경우
            else if (nearest_blue_x != std::numeric_limits<double>::max() && nearest_yellow_x == std::numeric_limits<double>::max()) {
                //nearest_yellow_y = -nearest_blue_y; // 반대편 대칭에 노란색
                //nearest_yellow_x = nearest_blue_x;
                ROS_INFO("Original Blue cone: %f", nearest_blue_y);
                if(nearest_blue_y >= 0){
                    nearest_yellow_y = -(nearest_blue_y+cone_interval); // 반대편 대칭에 노란색
                    nearest_yellow_x = nearest_blue_x;
                }
                else{
                    nearest_yellow_y = -(nearest_blue_y-cone_interval); // 반대편 대칭에 노란색
                    nearest_yellow_x = nearest_blue_x;
                }
                track_center_y = (nearest_yellow_y + nearest_blue_y) / 2.0;
                track_center_x = (nearest_yellow_x + nearest_blue_x) / 2.0;
                iscurve = true;

            }
        }
        
    }

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
*/



fusion_track::TrackCenter FusionTrack::getLaneCenterY()
{
    return track_center_msg;
}



// for i in range(len(vals)):
//         ser.write(vals[i]) #send!
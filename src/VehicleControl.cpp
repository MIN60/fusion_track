#include "fusion_track/VehicleControl.h"
#include "morai_msgs/CtrlCmd.h"
#include <cmath>
#include <ros/ros.h>
#include "fusion_track/pid.h"
#include "fusion_track/pure_pursuit.h"
#include <morai_msgs/CtrlCmd.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <control_msgs/Velocity.h>
#include "fusion_track/TrackCenter.h"
#include "fusion_track/IsCurve.h"
#include "fusion_track/ReadyToCurve.h"
#include <control_msgs/Gear.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>


VehicleControl::VehicleControl() : nh_(),pid_controller_(1.0, 0.4, 0.35, 0.1), pp_controller_()  {
    vehicle_length = 1.04;
    ld = 3.8;
    steering_angle = 0.0;
    current_vel = 0.0;
    double target_velocity = 100;

    ctrl_msg.gear = 0;
    ctrl_msg.velocity = target_velocity;
    ctrl_msg.brake = 0;

    steering_pub_ = nh_.advertise<std_msgs::Float64>("/steering_angle", 1);
    ctrl_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
    y_diff_sub_ = nh_.subscribe("/y_center_diff", 5, &VehicleControl::yDiffCallback, this);
    velocity_sub_ = nh_.subscribe("/ERP42_velocity", 1, &VehicleControl::velocityCallback, this);
    steer_sub = nh_.subscribe("/ERP42_steer", 1, &VehicleControl::steerCallback, this);

    target_velocity_pub = nh_.advertise<std_msgs::Float64>("/target_velocity", 1);

    //is_curve_sub_ = nh_.subscribe("/is_curve", 1, &VehicleControl::isCurveCallback, this);
    is_curve_sub_ = nh_.subscribe<fusion_track::IsCurve>("/is_curve", 1, &VehicleControl::isCurveCallback, this);
    ready_curve_sub_ = nh_.subscribe<fusion_track::ReadyToCurve>("/ready_curve", 1, &VehicleControl::readyCurveCallback, this);

    //gear_sub = nh_.subscribe("/ERP42_gear", 1, &VehicleControl::gearCallback, this);
    //gear_sub = nh_.subscribe<control_msgs::Gear>("/ERP42_gear", 1, &VehicleControl::gearCallback, this);


}

void VehicleControl::ldControl() {
    // if(current_vel <= 0) {
    //     ld = 1.9;
    // } else {
    //     ld = std::min(3.6 + 0.1 * current_vel, 7.0);
    // }

    if(current_vel <=0){
        ld = 0.9; //1.9
    }
    else if(0 < current_vel <= 30){
        ld = 2.8; //3.8
    }
    else if(30 < current_vel <= 90){
        ld = 2.5;  //3.2
    }
    else if(90 < current_vel <= 120){
        ld = 4.5;
    }
    else if(120 < current_vel <=150){
        ld = 5.8;
    }
    else{
        ld = 7;
    }
}

// 현재 속도 업데이트
void VehicleControl::velocityCallback(const control_msgs::Velocity::ConstPtr& msg) {
    current_vel = msg->velocity;
}

// 포인트 받아서 스티어링 앵글 계산
void VehicleControl::yDiffCallback(const fusion_track::TrackCenter::ConstPtr& track_center_msg) {
    ldControl(); //ld 업뎃
    ROS_INFO("yDiffCallback is called.");
    double target_velocity = 100; //50

    double track_center_x = track_center_msg->x;
    double track_center_y = track_center_msg->y;

    vehicle_msgs::WaypointsArray waypoints;
    vehicle_msgs::Waypoint waypoint;
 
    waypoint.x = track_center_x;
    waypoint.y = track_center_y;
    waypoints.waypoints.push_back(waypoint);

    double steering;
    vehicle_msgs::Waypoint forward_point;
    //std::tie(steering, forward_point, std::ignore) = pp_controller_.steering_angle(waypoints);
    std::tie(steering, forward_point, std::ignore) = pp_controller_.steering_angle(waypoints, ld);

    ctrl_msg.steering = steering;

    std_msgs::Float64 steering_msg;
    steering_msg.data = steering;

    steering_pub_.publish(steering_msg);

    ctrl_msg.gear = 0;
    ctrl_msg.accel = 70;
    ctrl_pub_.publish(ctrl_msg);
    ROS_INFO("======PUBLISH!======");

    ROS_INFO("Calculated Steering Angle: %f degrees", steering);
    updateControlCmd(target_velocity);
}

//현재 스티어링 앵글
void VehicleControl::steerCallback(const std_msgs::Float32::ConstPtr& steer_data) {
    steering_angle = steer_data->data;
    ROS_INFO("===Current ANGLE: %f===", steering_angle);
}

// 현재 기어
// void VehicleControl::gearCallback(const std_msgs::Float64::ConstPtr& gear_data) { //이거 타입 나중에 확인해봐야징
//     gear_msg = gear_data->data;
//     ROS_INFO("===Current GEAR: %d===", gear_msg);
// }

void VehicleControl::isCurveCallback(const fusion_track::IsCurve::ConstPtr& msg) {
    is_curve_ = msg->data;  
}

void VehicleControl::readyCurveCallback(const fusion_track::ReadyToCurve::ConstPtr& ReadyToCurve) {
    Ready = ReadyToCurve->Ready; 
    ReadyToCurve_x_y = ReadyToCurve->ReadyToCurve_x_y;  
    ReadyToCurve_y_y = ReadyToCurve->ReadyToCurve_y_y;
    ReadyToCurve_x_b = ReadyToCurve->ReadyToCurve_x_b;  
    ReadyToCurve_y_b = ReadyToCurve->ReadyToCurve_y_b;
    ROS_WARN("Received ReadyToCurve: Ready = %d", Ready);
    //ROS_WARN("Received ReadyToCurve: Ready = %d, x = %f, y = %f", Ready, ReadyToCurve_x, ReadyToCurve_y);

}



//PID써서 엑셀 브레이크 계산
void VehicleControl::updateControlCmd(double target_velocity) {
    ROS_INFO("updateControlCmd is called.");
    
    //double control_input = pid_controller_.pid(curvel_msg.velocity, target_velocity);

    if (is_curve_ == 1) {
        // 커브가 감지됐을 때(노란색 우회전)
        ROS_WARN("CURVE: %d", is_curve_);
        ROS_WARN("============RIGHT CURVE==============");
        // //target_velocity = 30;  // 커브에서는 30.0의 속도로 제한
        // ctrl_msg.brake = 100;
        // target_velocity = 10;//50
        // // 추가
        // ctrl_msg.steering = 18; //20
        // ctrl_pub_.publish(ctrl_msg);

        //기본정렬
        if(ReadyToCurve_y_y>0.5 && ReadyToCurve_y_y<=1){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = 15;
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE0");
        }
        //조금 더 꺾음 1
        else if(ReadyToCurve_y_y>0 && ReadyToCurve_y_y<=0.5){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = 12;
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE1");
        }
        else if(ReadyToCurve_y_y>0 && ReadyToCurve_y_y<=0.2){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = 15;
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE2");
        }
        // 조금 더 꺾음 2
        else if(ReadyToCurve_y_y>=-0.2 && ReadyToCurve_y_y<=0){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = 18; //try
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE3");
        }
        else if(ReadyToCurve_y_y>-0.5 && ReadyToCurve_y_y<-0.2){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = 17; //try
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE4");
        }
        // 조금 더 꺾음 3
        else if(ReadyToCurve_y_y>=-1 && ReadyToCurve_y_y<=-0.5){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = 20; //이거 체크
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE5");
        }
        ROS_WARN("ReadyToCurve: y = %f, steer: %f",ReadyToCurve_y_y, ctrl_msg.steering);

    }
    else if (is_curve_ == 2){
        // 커브가 감지됐을 때(파란색 좌회전)
        ROS_WARN("CURVE: %d", is_curve_);
        ROS_WARN("============LEFT CURVE==============");
        // //target_velocity = 30;  // 커브에서는 30.0의 속도로 제한
        // ctrl_msg.brake = 100;
        // target_velocity = 10; //50
        // // 추가
        // ctrl_msg.steering = -18; //-20
        // ctrl_pub_.publish(ctrl_msg);

        //기본정렬
        if(ReadyToCurve_y_b>0.5 && ReadyToCurve_y_b<=1){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = -15;
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE0");
        }
        //조금 더 꺾음 1
        else if(ReadyToCurve_y_b>0 && ReadyToCurve_y_b<=0.5){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = -12;
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE1");
        }
        else if(ReadyToCurve_y_b>0 && ReadyToCurve_y_b<=0.2){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = -15;
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE2");
        }
        // 조금 더 꺾음 2
        else if(ReadyToCurve_y_b>=-0.2 && ReadyToCurve_y_b<=0){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = -18; //try
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE3");
        }
        else if(ReadyToCurve_y_b>-0.5 && ReadyToCurve_y_b<-0.2){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = -17; //try
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE4");
        }
        // 조금 더 꺾음 3
        else if(ReadyToCurve_y_b>=-1 && ReadyToCurve_y_b<=-0.5){
            ctrl_msg.brake = 80;
            target_velocity = 30;//50
            ctrl_msg.steering = -20; //이거 체크
            ctrl_pub_.publish(ctrl_msg);
            ROS_WARN("CASE5");
        }
        ROS_WARN("ReadyToCurve: y = %f, steer: %f",ReadyToCurve_y_b, ctrl_msg.steering);
    }


    else if (is_curve_ == 0){
        ctrl_msg.brake = 0;
            //ctrl_msg.accel = 60;
        target_velocity = 70;
        // if(Ready){
        //     ROS_WARN("++++++++++slow mode+++++++++");
        //     target_velocity = 40;
        // }
        // else{
        //     ctrl_msg.brake = 0;
        //     //ctrl_msg.accel = 60;
        //     target_velocity = 70;
        //     //ctrl_msg.steering = pp_controller_.steering_angle(waypoints, ld); 
        //     //ROS_INFO("STEERING: %f", steering);
        // }
        // ctrl_pub_.publish(ctrl_msg);
    }

    //double control_input = pid_controller_.pid(target_velocity, curvel_msg.velocity);
    double control_input = pid_controller_.pid(target_velocity, current_vel);

    ROS_INFO("control_input: %f", control_input);

    

    if (control_input > 0) {
        ctrl_msg.accel = target_velocity;
        ctrl_msg.brake = 0;
    } 
    else {
        //ctrl_msg.accel = 0;
        ctrl_msg.brake = -control_input;
        ctrl_msg.accel = target_velocity;
        
    }
    ctrl_pub_.publish(ctrl_msg);
}
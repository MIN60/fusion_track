#include <ros/ros.h>
#include "morai_msgs/CtrlCmd.h"
#include "fusion_track/pid.h"
#include "fusion_track/pure_pursuit.h"
#include "fusion_track/TrackCenter.h" 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>  // gear 데이터 타입 맞춰서 수정하자
#include <control_msgs/Velocity.h>
#include "fusion_track/IsCurve.h"
#include "fusion_track/ReadyToCurve.h"



class VehicleControl {
private:
    double vehicle_length;
    double ld;
    double steering_angle;
    double current_vel; //현재 속도

    ros::NodeHandle nh_;
    ros::Publisher steering_pub_;
    ros::Publisher ctrl_pub_;
    ros::Publisher target_velocity_pub;
    ros::Subscriber y_diff_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber steer_sub;
    ros::Subscriber gear_sub;

    morai_msgs::CtrlCmd ctrl_msg;
    
    PidController pid_controller_;
    PurePursuit pp_controller_;
    
    std_msgs::Float32 steer_msg; 
    control_msgs::Velocity curvel_msg;
    ros::Subscriber is_curve_sub_;
    ros::Subscriber ready_curve_sub_;

public:
    VehicleControl();
    void updateControlCmd(double target_velocity);
    void yDiffCallback(const fusion_track::TrackCenter::ConstPtr& track_center_msg);
    void ldControl();
    void steerCallback(const std_msgs::Float32::ConstPtr& steer_data);
    void velocityCallback(const control_msgs::Velocity::ConstPtr& speed_data);
    int is_curve_;
    bool ready_curve;

    void isCurveCallback(const fusion_track::IsCurve::ConstPtr& msg);
    void readyCurveCallback(const fusion_track::ReadyToCurve::ConstPtr& ReadyToCurve);
    bool Ready;
    double ReadyToCurve_x_y;
    double ReadyToCurve_y_y;
    
    double ReadyToCurve_x_b;
    double ReadyToCurve_y_b;

};
#include "fusion_track/fusion_track.h"
#include "fusion_track/visual_marker.h"
#include "fusion_track/VehicleControl.h"

#include <iostream>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <morai_msgs/CtrlCmd.h>
#include "fusion_track/TrackCenter.h" 


int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_track_node"); 
    ros::NodeHandle nh;

    VehicleControl vehicle_control;
    morai_msgs::CtrlCmd ctrl_msg;

    FusionTrack fusion_track(nh);
    VisualMarker visual_marker(nh);

    ros::Rate rate(10); 
    
    
    while (ros::ok()) {
        //ros::spinOnce();
        ros::spin(); 
    
        vehicle_control.updateControlCmd(150); 

        rate.sleep();
    }

    return 0; 
}
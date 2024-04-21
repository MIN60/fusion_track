#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <vehicle_msgs/Waypoint.h>
#include <vehicle_msgs/WaypointsArray.h>
#include <tuple>

class PurePursuit {
private:
    vehicle_msgs::Waypoint forward_point = {};
    const double vehicle_length;
    double steering;
    std_msgs::Float32 current_vel;

public:
    PurePursuit();

    void getVelStatus(const std_msgs::Float32& msg);
    std::tuple<double, vehicle_msgs::Waypoint, double> steering_angle(const vehicle_msgs::WaypointsArray& newpoints, double ld);

    double ld;
    void updateLD(double new_ld) { ld = new_ld; }
};
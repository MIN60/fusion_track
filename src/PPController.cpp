#include "fusion_track/pure_pursuit.h"
#include "fusion_track/VehicleControl.h"
#include <cmath>
#include <morai_msgs/CtrlCmd.h>


PurePursuit::PurePursuit() : vehicle_length(1.04), steering(0) {}

void PurePursuit::getVelStatus(const std_msgs::Float32& msg) {
    current_vel = msg;
}

std::tuple<double, vehicle_msgs::Waypoint, double> PurePursuit::steering_angle(const vehicle_msgs::WaypointsArray& newpoints, double ld) {
    if (newpoints.waypoints.size() == 0) {
        return std::make_tuple(0.0, forward_point,0);
    }

    // ld를 이용하여 가장 가까운 포인트 선택
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& point : newpoints.waypoints) {
        double dx = point.x;
        double dy = point.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (std::abs(distance - ld) < min_distance) {
            min_distance = std::abs(distance - ld);
            forward_point = point;
        }
    }

    double dx = forward_point.x;
    double dy = forward_point.y;
    double dis = std::sqrt(dx * dx + dy * dy);
    double alpha = std::atan2(dy, dx);
    double delta = 2 * vehicle_length * std::sin(alpha) / dis;
    steering = -std::atan(delta) * 180.0 / M_PI;

    ROS_INFO("target point : %f %f", forward_point.x, forward_point.y);
    ROS_INFO("target steer : %f", steering);

    return std::make_tuple(steering, forward_point, 0);
}
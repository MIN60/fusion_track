#include "fusion_track/visual_marker.h"

VisualMarker::VisualMarker(ros::NodeHandle& nh)
{
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void VisualMarker::publishMarkers(double yellow_x, double yellow_y, double blue_x, double blue_y, double center_x, double center_y)
{
    visualization_msgs::Marker yellow_cone;
    yellow_cone.header.frame_id = "velodyne"; // RViz에서 표시할 좌표 프레임
    yellow_cone.header.stamp = ros::Time::now();
    yellow_cone.ns = "fusion_track";
    yellow_cone.id = 0;
    yellow_cone.type = visualization_msgs::Marker::SPHERE; // 노란색 라바콘
    yellow_cone.action = visualization_msgs::Marker::ADD;
    yellow_cone.pose.position.x = yellow_x;
    yellow_cone.pose.position.y = yellow_y;
    yellow_cone.pose.position.z = 0;
    yellow_cone.scale.x = 0.2; // 점 크기
    yellow_cone.scale.y = 0.2;
    yellow_cone.scale.z = 0.2;
    yellow_cone.color.r = 1.0; // 노란색으로 표시
    yellow_cone.color.g = 1.0;
    yellow_cone.color.b = 0.0;
    yellow_cone.color.a = 1.0; // 투명도 설정

    visualization_msgs::Marker blue_cone;
    blue_cone.header.frame_id = "velodyne"; // RViz에서 표시할 좌표 프레임
    blue_cone.header.stamp = ros::Time::now();
    blue_cone.ns = "fusion_track";
    blue_cone.id = 1;
    blue_cone.type = visualization_msgs::Marker::SPHERE; // 파란색 라바콘
    blue_cone.action = visualization_msgs::Marker::ADD;
    blue_cone.pose.position.x = blue_x;
    blue_cone.pose.position.y = blue_y;
    blue_cone.pose.position.z = 0;
    blue_cone.scale.x = 0.2; // 점 크기 조절
    blue_cone.scale.y = 0.2;
    blue_cone.scale.z = 0.2;
    blue_cone.color.r = 0.0; // 파란색
    blue_cone.color.g = 0.0;
    blue_cone.color.b = 1.0;
    blue_cone.color.a = 1.0; // 투명도

    visualization_msgs::Marker center_marker;
    center_marker.header.frame_id = "velodyne"; // RViz에서 표시할 좌표 프레임
    center_marker.header.stamp = ros::Time::now();
    center_marker.ns = "fusion_track";
    center_marker.id = 2;
    center_marker.type = visualization_msgs::Marker::SPHERE; // 중점
    center_marker.action = visualization_msgs::Marker::ADD;
    center_marker.pose.position.x = center_x;
    center_marker.pose.position.y = center_y;
    center_marker.pose.position.z = 0;
    center_marker.scale.x = 0.2; // 점의 크기 조절
    center_marker.scale.y = 0.2;
    center_marker.scale.z = 0.2;
    center_marker.color.r = 1.0; // 중점을 흰색
    center_marker.color.g = 1.0;
    center_marker.color.b = 1.0;
    center_marker.color.a = 1.0; // 투명도

    marker_pub.publish(yellow_cone);
    marker_pub.publish(blue_cone);
    marker_pub.publish(center_marker);
}
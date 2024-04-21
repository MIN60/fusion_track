// visual_marker.h

#ifndef VISUAL_MARKER_H
#define VISUAL_MARKER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class VisualMarker {
public:
    VisualMarker(ros::NodeHandle& nh);
    void publishMarkers(double yellow_x, double yellow_y, double blue_x, double blue_y, double center_x, double center_y);

private:
    ros::Publisher marker_pub;
};

#endif

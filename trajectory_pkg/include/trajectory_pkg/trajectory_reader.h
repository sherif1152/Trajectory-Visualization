#ifndef TRAJECTORY_READER_H
#define TRAJECTORY_READER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsoncpp/json/json.h>
#include <fstream>

class TrajectoryReader {
private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_; 
    std::string file_path_;
    std::string marker_topic_;

    void loadAndPublishMarkers();

public:
    TrajectoryReader();
};

#endif

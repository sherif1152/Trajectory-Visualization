#ifndef TRAJECTORY_SAVER_H
#define TRAJECTORY_SAVER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_pkg/SaveTrajectory.h>
#include <jsoncpp/json/json.h>
#include <fstream>

struct PoseData {
    double x, y, z;
    ros::Time timestamp;
};

class TrajectorySaver {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::ServiceServer save_service_;
    std::vector<PoseData> trajectory_data_;
    std::string odom_topic_;
    std::string file_path_;
    bool odom_received_ = false;
    bool has_valid_odom_ = false;


    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    bool saveTrajectory(trajectory_pkg::SaveTrajectory::Request &req,
                        trajectory_pkg::SaveTrajectory::Response &res);

public:
    TrajectorySaver();
};

#endif

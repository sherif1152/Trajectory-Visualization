#include <trajectory_pkg/trajectory_saver.h>

TrajectorySaver::TrajectorySaver()
{
    ros::NodeHandle p_nh("~");

    // Get parameters from launch file
    p_nh.param<std::string>("file_path", file_path_, "trajectory_data.json");
    p_nh.param<std::string>("odom_topic", odom_topic_, "/odom");

    // Initialize subscriber, and service
    odom_sub_ = nh_.subscribe(odom_topic_, 1000, &TrajectorySaver::odomCallback, this);
    save_service_ = nh_.advertiseService("save_trajectory", &TrajectorySaver::saveTrajectory, this);

    ROS_INFO("Trajectory Saver Node Initialized.");
}

// Callback to store odometry data
void TrajectorySaver::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{


    if (msg->header.stamp.toSec() == 0 ||
        !std::isfinite(msg->pose.pose.position.x) ||
        !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(msg->pose.pose.position.z) ||
        fabs(msg->pose.pose.position.x) > 1e6 ||
        fabs(msg->pose.pose.position.y) > 1e6 ||
        fabs(msg->pose.pose.position.z) > 1e6)
    {
        ROS_WARN_STREAM("Invalid odometry data detected.");
        has_valid_odom_ = false; // Mark as invalid
        return;
    }
    if (!odom_received_)
    {
        ROS_INFO("Odom callback triggered for the first time.");
        odom_received_ = true;
    }

    has_valid_odom_ = true; // Mark as valid

    PoseData data;
    data.x = msg->pose.pose.position.x;
    data.y = msg->pose.pose.position.y;
    data.z = msg->pose.pose.position.z;
    data.timestamp = msg->header.stamp;
    trajectory_data_.push_back(data);
}

// Service to save trajectory data to JSON file
bool TrajectorySaver::saveTrajectory(trajectory_pkg::SaveTrajectory::Request &req,
                                     trajectory_pkg::SaveTrajectory::Response &res)
{
    if (!has_valid_odom_)
    {
        ROS_WARN("No valid odometry data received. Cannot save trajectory.");
        res.success = false;
        return false;
    }

    Json::Value root;
    ros::Time now = ros::Time::now();

    // Collect data within the specified duration
    for (const auto &data : trajectory_data_)
    {
        if (now.toSec() - data.timestamp.toSec() <= req.duration)
        {
            Json::Value point;
            point["timestamp"] = (Json::UInt64)data.timestamp.toNSec();
            point["x"] = data.x;
            point["y"] = data.y;
            // point["z"] = data.z;
            root.append(point);
        }
    }

    // Write data to JSON file
    std::ofstream file(file_path_.c_str(), std::ios::out | std::ios::trunc);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", file_path_.c_str());
        res.success = false;
        return false;
    }

    file << root.toStyledString();
    file.close();
    res.success = true;
    ROS_INFO("Trajectory saved successfully to %s", file_path_.c_str());
    return true;
}

// Main function to run the ROS node
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_saver_node");
    TrajectorySaver ts;
    ros::spin();
    return 0;
}

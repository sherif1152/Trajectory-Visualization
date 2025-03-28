#include <trajectory_pkg/trajectory_reader.h>

TrajectoryReader::TrajectoryReader()
{
    ros::NodeHandle p_nh("~");

    // Get parameters from launch file or use defaults
    p_nh.param<std::string>("file_path", file_path_, "/tmp/trajectory_data.json");
    p_nh.param<std::string>("marker_topic", marker_topic_, "/trajectory_marker");

    // Initialize publisher
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);
    ROS_INFO("Trajectory Reader Node Started.");
    // Load and publish markers on initialization
    loadAndPublishMarkers();
}

// Function to load and publish markers from JSON file
void TrajectoryReader::loadAndPublishMarkers()
{
    std::ifstream file(file_path_);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", file_path_.c_str());
        return;
    }

    if (file.peek() == std::ifstream::traits_type::eof())
    {
        ROS_ERROR("File is empty: %s", file_path_.c_str());
        return;
    }

    Json::Value root;
    file >> root;
    file.close();

    visualization_msgs::MarkerArray markers;

    // Create markers from JSON data
    for (unsigned int i = 0; i < root.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = root[i]["x"].asDouble();
        marker.pose.position.y = root[i]["y"].asDouble();
        marker.pose.position.z = root[i]["z"].asDouble();
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.b = 1.0;
        markers.markers.push_back(marker);
    }

    if (markers.markers.empty())
    {
        ROS_WARN("No markers to publish. File might have no valid data.");
        return;
    }

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        marker_pub_.publish(markers);
        ROS_INFO_ONCE("Markers are shown. Done.");
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Main function to run the ROS node
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_reader_node");
    TrajectoryReader tr;
    ros::spin();
    return 0;
}

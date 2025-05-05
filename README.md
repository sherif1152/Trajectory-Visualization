# Trajectory Visualization and Storage for AMR Navigation

## Package Description
This package is designed to save and read trajectory data for robotic applications using ROS (Robot Operating System). It records odometry data, stores it in JSON format, and visualizes it using markers in RViz.

# 🎥 Demo
![alt text](trajectory_pkg/trajectory.gif)


# Main Files

- ###  trajectory_saver_node.cpp
  - **Purpose:**
    - Subscribes to odometry data .
    - Saves trajectory data to a JSON file via a ROS service.
  - **Functionality:**
    - Subscribes to the `/odom` topic to collect pose data (x, y, z).
    - Provides a `save_trajectory` service to save data on request.

- ###  trajectory_reader_node.cpp
  - **Purpose:**
    - Reads trajectory data from a JSON file.
    - Publishes markers to RViz for visualization.
  - **Functionality:**
    - Reads saved trajectory points.
    - Publishes markers to the `/trajectory_marker` topic.



# Launch Files

- ### trajectory_saver.launch
  Initializes the `trajectory_saver_node` with specified parameters.

  ```xml
  <launch>
      <node pkg="trajectory_pkg" type="trajectory_saver_node" name="trajectory_saver_node" output="screen">
          <param name="file_path" value="/home/sherif/anscer_ws/src/trajectory_pkg/data/trajectory_data.json"/>
          <param name="odom_topic" value="/odom"/>
      </node>
  </launch>
  ```
  Parameters:

  - `file_path`: Path to save the trajectory data (JSON format).

  - `odom_topic`: Odometry topic to subscribe to.

- ### trajectory_reader.launch
  Initializes the `trajectory_reader_node` with specified parameters.

  ```xml
  <launch>
      <node pkg="trajectory_pkg" type="trajectory_reader_node" name="trajectory_reader_node" output="screen">
          <param name="file_path" value="/home/sherif/anscer_ws/src/trajectory_pkg/data/trajectory_data.json"/>
          <param name="marker_topic" value="/trajectory_marker"/>
      </node>
  </launch>
  ```
  Parameters:

  - `file_path`: Path to read the trajectory data from.

  - `marker_topic`: Topic to publish markers.



# Header Files

- ### trajectory_saver.h
  Defines the `TrajectorySaver` class, its attributes, and its main functions.

  - **Attributes:**
    - `odom_sub_`: Subscribes to odometry data.
    - `save_service_`: Service to save trajectory data.
    - `trajectory_data_`: Stores odometry points.

  - **Main Functions:**
    - `odomCallback`: Collects odometry data.
    - `saveTrajectory`: Writes data to a JSON file.

- ### trajectory_reader.h
  Defines the `TrajectoryReader` class, its attributes, and its main functions.

  - **Attributes:**
    - `marker_pub_`: Publishes markers to RViz.
    - `file_path_`: Path to the JSON file.

  - **Main Functions:**
    - `loadAndPublishMarkers`: Reads JSON data and publishes markers.



# Service File

### SaveTrajectory.srv
Defines the request and response format for saving trajectory data.

```plaintext
string filename      # The file to save trajectory data
float64 duration     # Duration in seconds (backwards from now)
---
bool success         # True if saving was successful
```

# How to Build

To use these pkg , follow these installation steps:

### 1. Creating a workspace:
```sh
mkdir -p ~/catkin_ws/src
cd ~/robot_ws/src 
```
### 2. Clone this repository to your local machine:

```sh
git clone https://github.com/sherif1152/Trajectory-Visualization.git
```

### 3.Build and Setup

```sh
cd ..
catkin_make
source devel/setup.bash 
```

## Usage



### 1. Launch save trajectory nodes:

   To save trajectory data:
```bash
roslaunch trajectory_pkg trajectory_saver.launch
```
   
### 2. Save Trajectory:

  ```bash
  rosservice call /save_trajectory "duration: 10.0"
  ```
   - Saves the last 10 seconds of trajectory data.

### 3. Launch read trajectory nodes:
To read and visualize trajectory data:
  ```bash
  roslaunch trajectory_pkg trajectory_reader.launch
  ```


### 4. Visualize in RViz:
  - Launch RViz:
    ```bash
      rviz
    ```
  - Add a `MarkerArray` display.

  - Set the topic to `/trajectory_marker`.

---

### `Notes`

- Ensure that the odometry topic is publishing valid data.

- The saver will not save data if no valid odometry messages are received.

- The reader will warn if the file is empty or invalid.

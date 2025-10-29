
# EspeleoRobo Planning 2 - ROS 2 Path Planner

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![ROS 2](https://img.shields.io/badge/ROS%202-Foxy-blue)
![License](https://img.shields.io/badge/License-MIT-green)

Path planning package for EspeleoRobo and other robotics platforms developed at ITV/UFMG. This package provides multiple methods for generating and publishing reference paths in ROS 2 environments.

## Installation

### Prerequisites
- ROS 2 Jazzy (or current distribution)
- `colcon` build system
- Basic ROS 2 workspace setup

### Installation Steps

1. Create and navigate to your ROS 2 workspace:
   ```bash
   mkdir -p ~/espeleo_ws/src
   cd ~/espeleo_ws/src
   ```

2. Clone this repository:
   ```bash
   git clone https://github.com/ITVRoC/polaris_planning.git
   ```
3. Install dependencies
    ```bash
    cd ~/espeleo_ws
    rosdep install -i --from-paths src/polaris_planning --rosdistro $ROS_DISTRO -y
    ```
    
4. Build the package:
	```bash
	cd ~/espeleo_ws
	colcon build --symlink-install
	```
5. Source the workspace:

	```bash
	source install/setup.bash
	```


## How to use

Launch the planner with:
```bash
ros2 launch polaris_planning launch_planner.xml
```
	
Select your desired planner by editing the launch file `your_workspace/src/polaris_planning/launch/launch_planner.xml`.


## Planner types

### Path from points

Generates smooth polynomial paths from waypoints selected in [RViz2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html) using the 2D Nav Goal Tool.

**Topics:**

- `path_topic_name`  (type: `nav_msgs/Path`): publisher for the generated path.

- `visualization_topic_name`  (type: `visualization_msgs/MarkerArray`): publisher for visualizing the last published path or the waypoints. This message is published only once, thus, rviz2 should be ready to receive it when the start service is called.

- `clicked_point_topic_name`  (type: `geometry_msgs/PoseStamped`): subscriber of the waypoints used by the planner.

- `pose_topic_name`  (type: `nav_msgs/Odometry`): subscriber for robot's pose.


**Services:**

- `start_service_name` (type: `std_srvs/Trigger`): service for starting the planner, computing the polynomials and publishing the path.

- `clear_service_name`  (type: `std_srvs/Trigger`): service for clearing all the points already saved.

- `remove_last_point_service_name` (type: `std_srvs/Trigger`): service for clearing the last point saved.

- `close_path_service_name` (type: `std_srvs/Trigger`): service for adding the fist point again and closing the path.


**Input parameters:**

Set the parameters for the launch in the file `config/path_from_points.yaml`.

- `N_points`: number of points to be sampled in between ech pair of points;
- `max_point_distance`: maximum allowed distance between points
- `flag_interpolation_method`: if 2 quadratic interpolation if 3 cubid and smooth;
- `path_topic_name`: name of the topic in which the path will be passed to the controller ;
- `visualization_topic_name`: name of the topic in which the path will be passed to Rviz2;
- `clicked_point_topic_name`: name of the topic in which the pose will be obtained;
- `pose_topic_name`: name of the topic in which the pose will be obtained;
- `start_service_name`: name of the service for starting the planner, computing the polynomials and publishing the path;
- `clear_service_name`: name of the service for clearing all the points already saved;
- `remove_last_point_service_name`: name of the service for clearing the last point saved;


### Path from file

Loads paths from text files in `path_txt/` directory.

**File Format**

```txt
N  # Number of points
x1 y1 z1
x2 y2 z2
...
xN yN zN
```

**Topics:**

- `path_topic_name`: name of the topic in which the path will be passed to the controller ;
- `visualization_topic_name`: name of the topic in which the path will be passed to Rviz2;

**Input parameters:**

Set the parameters for the launch in the file `config/path_from_file.yaml`.

- `pkg_path`: specify the path for this package in your computer. (~/your_workspace/src/espeleo_planning)
- `path_number`: choose the Nth path `path_N.txt`
- `marker_scale`: marker parameter
- `frame_id`: reference frame
- `publish_path_topic_name`: name of the topic in which the path will be passed to the controller
- `visual_path_topic_name`: name of the topic in which the path will be passed to Rviz2;

### Path from equation

Publishes a path originated from a equation. Several equation templates are already implemented inside the code. You can also add/modify an existing equation method and create a new one.

**Topics:**
- `path_topic_name`: name of the topic in which the path will be passed to the controller ;
- `visualization_topic_name`: name of the topic in which the path will be passed to Rviz2;

**Input parameters:**

Parameters for the example trajectories:
- `path_number`: number of example curve to be generated, from 1 to 5;
- `number_of_samples`: number of points of the curve to be sampled, 200 for example;
- `a`: stretch factor of the curve in the x direction;
- `b`: stretch factor of the curve in the y direction;
- `phi`: rotation of the curve around the z axis (in degrees);
- `cx`: displacement in the x direction (meters);
- `cy`: displacement in the y direction (meters);
- `closed_path_flag` (`bool`): Flag to indicate if the path is closed or not;
- `insert_n_poins` (`int`): Number of points to be inserted in between each pair of points of the received trajectory;
- `filter_path_n_average` (`int`): Number of points to use in the average filter (it is forced to be an odd number) - if 0 the path is not filtered.
- `marker_scale`: marker parameter
- `marker_z_offset`: marker parameter
- `frame_id`: reference frame
- `publish_path_topic_name`: name of the topic in which the path will be passed to the controller
- `visual_path_topic_name`: name of the topic in which the path will be passed to Rviz2;

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contact:

João Felipe Ribeiro Baião
<baiaojfr@gmail.com>
Instituto Tecnológico Vale (ITV)
Universidade Federal de Minas Gerais (UFMG)


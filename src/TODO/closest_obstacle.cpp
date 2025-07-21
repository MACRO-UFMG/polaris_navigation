
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rviz_discontinuity_;

    rclcpp::SubscriptionBase::SharedPtr sub_laser_;

        pub_rviz_discontinuity_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_discontinuity", 10);

        // Subscriber for laser data
        if (laser_topic_type_ == "LaserScan") {
            sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>(
                laser_topic_name_, 10, std::bind(&VectorFieldController::callbackLaserScan, this, std::placeholders::_1));
        } else if (laser_topic_type_ == "Point") {
            sub_laser_ = create_subscription<geometry_msgs::msg::Point>(
                laser_topic_name_, 10, std::bind(&VectorFieldController::callbackPoint, this, std::placeholders::_1));
        } else if (laser_topic_type_ == "PointCloud") {
            sub_laser_ = create_subscription<sensor_msgs::msg::PointCloud>(
                laser_topic_name_, 10, std::bind(&VectorFieldController::callbackPointCloud, this, std::placeholders::_1));
        } else {
            RCLCPP_ERROR(get_logger(), "Invalid value for laser_topic_type: %s", laser_topic_type_.c_str());
        }




    void computeDiscontinuity() {
        // Get robot state
        double x_n = pos_[0];
        double y_n = pos_[1];
        double theta_n = rpy_[2];
        
        // Initialize arrays
        std::vector<double> theta_p;  // angles of discontinuities
        std::vector<double> laser_p;  // distances at discontinuities
        idx_obst_.clear();  // indices of discontinuities
        
        // Detect transitions between obstacle and free space
        int sig = (laserVec_[0] < range_max_) ? 1 : 0;
        for (size_t k = 2; k < laserVec_.size(); k++) {
            if (sig == 1 && laserVec_[k-1] == range_max_) {  // Obstacle to free space
                sig = 0;
                addDiscontinuity(k-2, theta_p, laser_p);
            } else if (sig == 0 && laserVec_[k-1] < range_max_) {  // Free space to obstacle
                sig = 1;
                addDiscontinuity(k-1, theta_p, laser_p);
            } else {  // Large changes in distance
                if (laserVec_[k-1] - laserVec_[k-2] > trig_) {  // Near to far
                    addDiscontinuity(k-2, theta_p, laser_p);
                } else if (laserVec_[k-1] - laserVec_[k-2] < -trig_) {  // Far to near
                    addDiscontinuity(k-1, theta_p, laser_p);
                }
            }
        }
        
        // Check scan boundaries
        if (laserVec_[0] < range_max_) {
            addDiscontinuity(0, theta_p, laser_p);
        }
        if (laserVec_.back() < range_max_) {
            addDiscontinuity(laserVec_.size()-1, theta_p, laser_p);
        }
        
        // Convert to Cartesian coordinates
        x_obst_local_.clear();
        y_obst_local_.clear();
        for (size_t k = 0; k < laser_p.size(); k++) {
            x_obst_local_.push_back(laser_p[k] * cos(theta_p[k]));
            y_obst_local_.push_back(laser_p[k] * sin(theta_p[k]));
        }
        
        // Convert to world frame
        x_obst_world_.clear();
        y_obst_world_.clear();
        for (size_t k = 0; k < x_obst_local_.size(); k++) {
            x_obst_world_.push_back(cos(theta_n) * x_obst_local_[k] - 
                                   sin(theta_n) * y_obst_local_[k] + x_n);
            y_obst_world_.push_back(sin(theta_n) * x_obst_local_[k] + 
                                   cos(theta_n) * y_obst_local_[k] + y_n);
        }
        
        // Visualize discontinuities
        sendDiscontinuityMarkerToRviz();
    }

    void addDiscontinuity(int idx, std::vector<double>& theta_p, std::vector<double>& laser_p) {
        theta_p.push_back(thetaVec_[idx]);
        laser_p.push_back(laserVec_[idx]);
        idx_obst_.push_back(idx);
    }

    void sendDiscontinuityMarkerToRviz() {
        auto points_marker = visualization_msgs::msg::MarkerArray();
        
        // Create markers for each discontinuity point
        for (size_t k = 0; k < x_obst_local_.size(); k++) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.id = k;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = x_obst_local_[k];
            marker.pose.position.y = y_obst_local_[k];
            marker.pose.position.z = 0.2;
            points_marker.markers.push_back(marker);
        }
        
        // Clear old markers
        for (size_t k = x_obst_local_.size(); k < 20; k++) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.id = k;
            points_marker.markers.push_back(marker);
        }
        
        pub_rviz_discontinuity_->publish(points_marker);
    }
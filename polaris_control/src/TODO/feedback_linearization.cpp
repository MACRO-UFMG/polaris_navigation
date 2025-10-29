
    std::pair<double, double> feedbackLinearization(double Ux, double Uy) {
        double psi = rpy_[2];
        double VX = cos(psi) * Ux + sin(psi) * Uy;
        double WZ = (-sin(psi) / d_) * Ux + (cos(psi) / d_) * Uy;
        return {VX, WZ};
    }

    std::pair<double, double> feedbackLinearizationRepulsive(double Ux, double Uy, double delta_m, double phi_m) {
        auto cmd = feedbackLinearization(Ux, Uy);
        double VX = cmd.first;
        double WZ = cmd.second;
        
        // Add repulsive action if recent laser data available
        if (this->now().seconds() - last_laser_data_ < 0.5 && 0.02 < delta_m && delta_m < 5.0) {
            double Ux_rep_body = -K_repulsive_ * cos(phi_m) / (delta_m * delta_m);
            double Uy_rep_body = -K_repulsive_ * sin(phi_m) / (delta_m * delta_m);
            VX += Ux_rep_body;
            WZ += Uy_rep_body / d_;
        }
            
        return {VX, WZ};
    }


    std::pair<double, double> computeObstacleFollowingCommand() {
        double G = (2/M_PI) * atan2(kf_*(delta_m_ - epsilon_), 1);
        double H = sqrt(1 - G*G) * DIR_;
        
        double v = v_r_ * (cos(phi_m_)*G - sin(phi_m_)*H);
        double w = v_r_ * (sin(phi_m_)*G/d_ + cos(phi_m_)*H/d_);
        
        return {v, w};
    }

    geometry_msgs::msg::Twist computeVectorFieldCommand(double Vx_ref, double Vy_ref) {
        std::pair<double, double> cmd;
        if (flag_repulsive_action_) {
            cmd = feedbackLinearizationRepulsive(Vx_ref, Vy_ref, delta_m_, phi_m_);
        } else {
            cmd = feedbackLinearization(Vx_ref, Vy_ref);
        }
            
        auto vel = geometry_msgs::msg::Twist();
        vel.linear.x = invert_motion_flag_ ? -cmd.first : cmd.first;
        vel.angular.z = cmd.second;
        
        // Visualize reference velocity
        sendCommandMarkerToRviz(Vx_ref, Vy_ref);
        
        return vel;
    }


    geometry_msgs::msg::Twist computeObstacleAwareCommand(double Vx_ref, double Vy_ref) {
        // Compute obstacle vector in world frame
        double b_x = delta_m_ * cos(phi_m_ + rpy_[2]);
        double b_y = delta_m_ * sin(phi_m_ + rpy_[2]);
        
        double V_forward, w_z;
        if (STATE_ == 0) {  // Following vector field
            auto cmd = feedbackLinearization(Vx_ref, Vy_ref);
            V_forward = cmd.first;
            w_z = cmd.second;
            
            // Switch to obstacle following if close and moving toward obstacle
            if (delta_m_ < switch_dist_ && (Vx_ref*b_x + Vy_ref*b_y > 0)) {
                STATE_ = 1;
                double cross_product = Vx_ref*b_y - Vy_ref*b_x;
                DIR_ = (cross_product > 0) ? -1 : 1;
                RCLCPP_WARN(get_logger(), "Following obstacle %s...", (DIR_ == -1) ? "right" : "left");
            }
        } else if (STATE_ == 1) {  // Following obstacle
            auto cmd = computeObstacleFollowingCommand();
            V_forward = cmd.first;
            w_z = cmd.second;
            
            // Switch back to vector field if moving away from obstacle
            if (Vx_ref*b_x + Vy_ref*b_y < 0) {
                STATE_ = 0;
                RCLCPP_INFO(get_logger(), "Following vector field...");
            }
        }
                
        auto vel = geometry_msgs::msg::Twist();
        vel.linear.x = invert_motion_flag_ ? -V_forward : V_forward;
        vel.angular.z = w_z;
        
        // Visualize reference velocity
        sendCommandMarkerToRviz(Vx_ref, Vy_ref);
        
        return vel;
    }


    void stopRobotIfMoving() {
        auto vel = geometry_msgs::msg::Twist();
        if (vel.linear.x != 0.0 || vel.angular.z != 0.0) {
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            pub_cmd_vel_->publish(vel);
        }
    }



    void callbackLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        laserVec_ = msg->ranges;
        range_min_ = msg->range_min;
        range_max_ = msg->range_max;
        
        // Compute angle vector
        thetaVec_.clear();
        thetaVec_.push_back(msg->angle_min);
        for (size_t k = 1; k < laserVec_.size(); k++) {
            thetaVec_.push_back(thetaVec_[k-1] + msg->angle_increment);
        }
        
        // Find minimum distance and direction
        delta_m_ = std::numeric_limits<double>::infinity();
        int k_m = -1;
        for (size_t k = 0; k < laserVec_.size(); k++) {
            if (0.10 < laserVec_[k] && laserVec_[k] < delta_m_) {
                delta_m_ = laserVec_[k];
                k_m = k;
            }
        }
                
        phi_m_ = msg->angle_min + k_m * msg->angle_increment;
        last_laser_data_ = this->now().seconds();
        
        computeDiscontinuity();
    }

    void callbackPoint(const geometry_msgs::msg::Point::SharedPtr msg) {
        phi_m_ = atan2(msg->y, msg->x);
        delta_m_ = sqrt(msg->x * msg->x + msg->y * msg->y);
        last_laser_data_ = this->now().seconds();
    }

    void callbackPointCloud(const sensor_msgs::msg::PointCloud::SharedPtr msg) {
        RCLCPP_ERROR(get_logger(), "The callback for sensor_msgs/PointCloud message needs to be implemented");
    }

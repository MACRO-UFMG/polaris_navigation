#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

/*
Universidade Federal de Minas Gerais (UFMG) - 2025
Laboratório CORO
Instituto Tecnologico Vale (ITV)
Contact:
Thales Andrade Soares, <thalesasoares02@gmail.com>
*/

using namespace std::chrono_literals;

class CorridorFollower : public rclcpp::Node
{
public:
    CorridorFollower() : Node("corridor_follower")
    {
        // Initialize parameters with default values
        this->declare_parameter<double>("vr", 1.0);
        this->declare_parameter<double>("kf", 1.0);
        this->declare_parameter<double>("d", 0.2);
        this->declare_parameter<bool>("invert_motion_flag", false);
        this->declare_parameter<bool>("log_gt_flag", false);
        this->declare_parameter<int>("center", 120);
        
        // Topic names
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        this->declare_parameter<std::string>("scan_topic", "scan");
        this->declare_parameter<std::string>("laser_frame_id", "base_laser_link");
        this->declare_parameter<std::string>("gt_topic", "");
        this->declare_parameter<std::string>("log_path_name", "");
        
        // Load parameters
        load_parameters();
        
        // Initialize ROS components
        initialize_ros_components();
        
        RCLCPP_INFO(this->get_logger(), "Corridor follower node initialized");
    }

    ~CorridorFollower()
    {
        if (file_handle_.is_open())
        {
            file_handle_.close();
        }
    }

private:
    // Parameters
    double vr_;
    double kf_;
    double d_;
    bool invert_motion_;
    bool log_gt_flag_;
    int center_;
    
    // Topic names
    std::string cmd_vel_topic_;
    std::string scan_topic_;
    std::string laser_frame_id_;
    std::string gt_topic_;
    std::string log_path_;
    
    // State variables
    double delta_1_ = std::numeric_limits<double>::infinity();
    double phi_1_ = 0.0;
    double delta_2_ = std::numeric_limits<double>::infinity();
    double phi_2_ = 0.0;
    bool new_data_ = false;
    int no_data_counter_ = 0;
    std::ofstream file_handle_;
    
    // ROS components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_1_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_2_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
    visualization_msgs::msg::Marker marker_1_;
    visualization_msgs::msg::Marker marker_2_;
    rclcpp::TimerBase::SharedPtr timer_;

    void load_parameters()
    {
        try
        {
            vr_ = this->get_parameter("vr").as_double();
            kf_ = this->get_parameter("kf").as_double();
            d_ = this->get_parameter("d").as_double();
            invert_motion_ = this->get_parameter("invert_motion_flag").as_bool();
            log_gt_flag_ = this->get_parameter("log_gt_flag").as_bool();
            center_ = this->get_parameter("center").as_int();
            
            // Topic parameters
            cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
            scan_topic_ = this->get_parameter("scan_topic").as_string();
            laser_frame_id_ = this->get_parameter("laser_frame_id").as_string();
            
            // Logging parameters
            if (log_gt_flag_)
            {
                gt_topic_ = this->get_parameter("gt_topic").as_string();
                log_path_ = this->get_parameter("log_path_name").as_string();
            }
            
            print_parameters();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter loading failed: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), "Using default parameters");
        }
    }

    void print_parameters()
    {
        RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
        RCLCPP_INFO(this->get_logger(), "vr: %f", vr_);
        RCLCPP_INFO(this->get_logger(), "kf: %f", kf_);
        RCLCPP_INFO(this->get_logger(), "d: %f", d_);
        RCLCPP_INFO(this->get_logger(), "invert_motion: %d", invert_motion_);
        RCLCPP_INFO(this->get_logger(), "log_gt_flag: %d", log_gt_flag_);
        RCLCPP_INFO(this->get_logger(), "center: %d", center_);
        RCLCPP_INFO(this->get_logger(), "cmd_vel_topic: %s", cmd_vel_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "scan_topic: %s", scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "laser_frame_id: %s", laser_frame_id_.c_str());
        
        if (log_gt_flag_)
        {
            RCLCPP_INFO(this->get_logger(), "gt_topic: %s", gt_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "log_path: %s", log_path_.c_str());
        }
    }

    void initialize_ros_components()
    {
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
        marker_1_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("close_marker_1", 1);
        marker_2_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("close_marker_2", 1);
        
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 1, std::bind(&CorridorFollower::laser_callback, this, std::placeholders::_1));
            
        if (log_gt_flag_)
        {
            gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                gt_topic_, 1, std::bind(&CorridorFollower::ground_truth_callback, this, std::placeholders::_1));
            file_handle_.open(log_path_);
        }
        
        // Initialize markers
        marker_1_ = create_marker(0.0, 1.0, 0.0); // Green for right
        marker_2_ = create_marker(0.0, 0.0, 1.0); // Blue for left
        
        // Create timer for control loop (10Hz)
        timer_ = this->create_wall_timer(100ms, std::bind(&CorridorFollower::control_loop, this));
    }

    visualization_msgs::msg::Marker create_marker(float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = laser_frame_id_;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 3 * d_;
        marker.scale.y = 3 * d_;
        marker.scale.z = 3 * d_;
        marker.color.a = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.pose.orientation.w = 1.0;
        return marker;
    }

    void ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (file_handle_.is_open())
        {
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
            
            // Extract yaw from quaternion
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            file_handle_ << x << "\t" << y << "\t" << yaw << "\n";
        }
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const auto& ranges = msg->ranges;
        
        // Find closest valid measurement on right side (first half)
        delta_1_ = std::numeric_limits<double>::infinity();
        int k1 = -1;
        for (int k = 0; k < center_; k++)
        {
            if (ranges[k] > 0.10 && ranges[k] < delta_1_)
            {
                delta_1_ = ranges[k];
                k1 = k;
            }
        }
        
        // Find closest valid measurement on left side (second half)
        delta_2_ = std::numeric_limits<double>::infinity();
        int k2 = -1;
        for (int k = center_; k < static_cast<int>(ranges.size()); k++)
        {
            if (ranges[k] > 0.10 && ranges[k] < delta_2_)
            {
                delta_2_ = ranges[k];
                k2 = k;
            }
        }
        
        // Update center point between the two closest points
        if (k1 != -1 && k2 != -1)
        {
            center_ = (k1 + k2) / 2;
            RCLCPP_DEBUG(this->get_logger(), "Center index: %d", center_);
            
            // Compute angles for both points
            phi_1_ = msg->angle_min + k1 * msg->angle_increment;
            phi_2_ = msg->angle_min + k2 * msg->angle_increment;
            new_data_ = true;
        }
    }

    std::pair<double, double> compute_control_command()
    {
        double alpha = (phi_2_ - phi_1_ - M_PI) / 2.0;
        double phi_D = phi_1_ + alpha;
        double phi_T = phi_1_ + alpha + M_PI / 2.0;
        
        // Compute corridor width
        double D = (delta_2_ - delta_1_) / (2 * cos(alpha));
        
        // Compute vector field components
        double G = -(2 / M_PI) * atan2(kf_ * D, 1);
        double H = sqrt(1 - G * G);
        
        // Compute velocity components
        double vx = G * cos(phi_D) + H * cos(phi_T);
        double vy = G * sin(phi_D) + H * sin(phi_T);
        
        // Final commands
        double v = vr_ * vx;
        double omega = vr_ * vy / d_;
        
        return {v, omega};
    }

    void control_loop()
    {
        auto vel = geometry_msgs::msg::Twist();
        
        if (new_data_)
        {
            // Compute and apply control command
            auto [v, omega] = compute_control_command();
            
            // Update markers
            marker_1_.header.stamp = this->now();
            marker_1_.pose.position.x = delta_1_ * cos(phi_1_);
            marker_1_.pose.position.y = delta_1_ * sin(phi_1_);
            marker_1_pub_->publish(marker_1_);
            
            marker_2_.header.stamp = this->now();
            marker_2_.pose.position.x = delta_2_ * cos(phi_2_);
            marker_2_.pose.position.y = delta_2_ * sin(phi_2_);
            marker_2_pub_->publish(marker_2_);
            
            // Apply motion inversion if needed
            vel.linear.x = invert_motion_ ? -v : v;
            vel.angular.z = omega;
            
            new_data_ = false;
            no_data_counter_ = 0;
        }
        else
        {
            no_data_counter_++;
            // Stop if no laser data for 1 second
            if (no_data_counter_ > 10)
            {
                vel.linear.x = 0.0;
                vel.angular.z = 0.0;
            }
        }
        
        cmd_vel_pub_->publish(vel);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CorridorFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
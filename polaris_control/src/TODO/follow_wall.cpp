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

/*
Universidade Federal de Minas Gerais (UFMG) - 2025
Laboratório CORO
Instituto Tecnologico Vale (ITV)
Contact:
João Felipe Ribeiro Baião, <baiaojfr@gmail.com>
*/

using namespace std::chrono_literals;

class WallFollower : public rclcpp::Node
{
public:
    WallFollower() : Node("wall_follower")
    {
        // Initialize parameters with default values
        this->declare_parameter<double>("vr", 1.0);
        this->declare_parameter<double>("kf", 1.0);
        this->declare_parameter<double>("epsilon", 1.0);
        this->declare_parameter<double>("d", 0.2);
        this->declare_parameter<bool>("invert_motion", false);
        this->declare_parameter<bool>("keep_wall_right", true);
        this->declare_parameter<bool>("log_gt_flag", false);
        
        // Topic names
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        this->declare_parameter<std::string>("scan_topic", "scan");
        this->declare_parameter<std::string>("laser_frame_id", "base_laser_link");
        this->declare_parameter<std::string>("gt_topic", "");
        this->declare_parameter<std::string>("log_path", "");
        
        // Load parameters
        load_parameters();
        
        // Initialize ROS components
        initialize_ros_components();
        
        RCLCPP_INFO(this->get_logger(), "Wall follower node initialized");
    }

    ~WallFollower()
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
    double epsilon_;
    double d_;
    bool invert_motion_;
    bool keep_wall_right_;
    bool log_gt_flag_;
    
    // Topic names
    std::string cmd_vel_topic_;
    std::string scan_topic_;
    std::string laser_frame_id_;
    std::string gt_topic_;
    std::string log_path_;
    
    // State variables
    double delta_m_ = std::numeric_limits<double>::infinity();
    double phi_m_ = 0.0;
    bool new_data_ = false;
    std::ofstream file_handle_;
    
    // ROS components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
    visualization_msgs::msg::Marker closest_point_marker_;
    rclcpp::TimerBase::SharedPtr timer_;

    void load_parameters()
    {
        try
        {
            vr_ = this->get_parameter("vr").as_double();
            kf_ = this->get_parameter("kf").as_double();
            epsilon_ = this->get_parameter("epsilon").as_double();
            d_ = this->get_parameter("d").as_double();
            invert_motion_ = this->get_parameter("invert_motion").as_bool();
            keep_wall_right_ = this->get_parameter("keep_wall_right").as_bool();
            log_gt_flag_ = this->get_parameter("log_gt_flag").as_bool();
            
            // Topic parameters
            cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
            scan_topic_ = this->get_parameter("scan_topic").as_string();
            laser_frame_id_ = this->get_parameter("laser_frame_id").as_string();
            
            // Logging parameters
            if (log_gt_flag_)
            {
                gt_topic_ = this->get_parameter("gt_topic").as_string();
                log_path_ = this->get_parameter("log_path").as_string();
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
        RCLCPP_INFO(this->get_logger(), "epsilon: %f", epsilon_);
        RCLCPP_INFO(this->get_logger(), "d: %f", d_);
        RCLCPP_INFO(this->get_logger(), "invert_motion: %d", invert_motion_);
        RCLCPP_INFO(this->get_logger(), "keep_wall_right: %d", keep_wall_right_);
        RCLCPP_INFO(this->get_logger(), "cmd_vel_topic: %s", cmd_vel_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "scan_topic: %s", scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "laser_frame_id: %s", laser_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "log_gt_flag: %d", log_gt_flag_);
        
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
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("closest_laser_marker", 1);
        
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 1, std::bind(&WallFollower::laser_callback, this, std::placeholders::_1));
            
        if (log_gt_flag_)
        {
            gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                gt_topic_, 1, std::bind(&WallFollower::ground_truth_callback, this, std::placeholders::_1));
            file_handle_.open(log_path_);
        }
        
        // Initialize marker for visualization
        closest_point_marker_ = create_marker();
        
        // Create timer for control loop
        timer_ = this->create_wall_timer(100ms, std::bind(&WallFollower::control_loop, this));
    }

    visualization_msgs::msg::Marker create_marker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = laser_frame_id_;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 3 * d_;
        marker.scale.y = 3 * d_;
        marker.scale.z = 3 * d_;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
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
        // Find closest valid measurement
        double min_dist = std::numeric_limits<double>::infinity();
        int min_idx = -1;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double dist = msg->ranges[i];
            if (dist > 0.10 && dist < min_dist)
            {
                min_dist = dist;
                min_idx = i;
            }
        }
        
        if (min_idx != -1)
        {
            delta_m_ = min_dist;
            phi_m_ = msg->angle_min + min_idx * msg->angle_increment;
            new_data_ = true;
        }
    }

    std::pair<double, double> compute_control_command()
    {
        double G = (2/M_PI) * atan2(kf_ * (delta_m_ - epsilon_), 1);
        double H = sqrt(1 - G*G);
        
        if (!keep_wall_right_)
        {
            H = -H;
        }
        
        double v = vr_ * (cos(phi_m_)*G - sin(phi_m_)*H);
        double omega = vr_ * (sin(phi_m_)*G/d_ + cos(phi_m_)*H/d_);
        
        return {v, omega};
    }

    void control_loop()
    {
        auto vel = geometry_msgs::msg::Twist();
        
        if (new_data_)
        {
            // Update marker position
            closest_point_marker_.header.stamp = this->now();
            closest_point_marker_.pose.position.x = delta_m_ * cos(phi_m_);
            closest_point_marker_.pose.position.y = delta_m_ * sin(phi_m_);
            closest_point_marker_.pose.position.z = d_;
            marker_pub_->publish(closest_point_marker_);
            
            // Compute and apply control command
            auto [v, omega] = compute_control_command();
            vel.linear.x = invert_motion_ ? -v : v;
            vel.angular.z = omega;
            new_data_ = false;
        }
        
        cmd_vel_pub_->publish(vel);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
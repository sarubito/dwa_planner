#include "dwa_planner/dwa_planner_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace dwa_planner
{
    DWAPlanner::DwaPlanner(const rclcpp::NodeOptions & options) : Node("dwa_planner", options)
    {
        scan_update_flg = false;
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&DWAPlanner::scan_callback, this, std::placeholders::_1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&DWAPlanner::odom_callback, this, std::placeholders::_1))
    }

    void DWAPlanner::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan = *msg;
        scan_update_flg = true;
    }


    void DWAPlanner::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_odometry = *msg;
        odom_updated = true;
    }

    float DWAPlanner::calc_heading(geometry_msgs::msg::Point goal_point)
    {
        float angle = atan2(robot_odometry.pose.pose.position.y - goal_point.y, robot_odometry.pose.pose.position.x - goal_point.x);
        if(angle > (2 * asin(odometry.pose.pose.orientation.z)))
        {
            float heading = angle - (2 * asin(odometry.pose.pose.orientation.z));
        } else {
            float heading = (2 * asin(odometry.pose.pose.orientation.z)) - angle;
        }        
        heading = 180 - heading;
        return heading;
    }

    float DWAPlanner::calc_dist(float linear, float angular)
    {
        std::vector<geometry_msgs::msg::Point> point_list = scan_to_cartesian();
        for(auto &v : point_list){
            //距離計算
        }
    }

    float DWAPlanner::calc_velocity(void)
    {
        float linear = robot_odometry.twist.twist.linear.x;
        float angular_velocity = robot_odometry.twist.twist.angular.z;
        float speed = linear_velocity + angular_velocity;
        return speed;
    }


    std::vector<geometry_msgs::msg::Point> DWAPlanner::scan_to_cartesian(void)
    {
        std::vector<geometry_msgs::msg::Point> point_list;
        eometry_msgs::msg::Point xyz;
        float angle = scan.angle_min;
        for(auto r : scan.ranges){
            xyz.x = r * cos(angle);
            xyz.y = r * sin(angle);
            xyz.z = 0.0;
            point_list.push_back(xyz);
            angle += scan.angle_increment;
        }

        return point_list;
    }

}
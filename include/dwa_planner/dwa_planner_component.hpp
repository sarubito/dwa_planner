#ifndef DWA_PLANNER_COMPONENT_HPP_
#define DWA_PLANNER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DWA_PLANNER_COMPONENT_EXPORT __attribute__((dllexport))
#define DWA_PLANNER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define DWA_PLANNER_COMPONENT_EXPORT __declspec(dllexport)
#define DWA_PLANNER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef DWA_PLANNER_COMPONENT_BUILDING_DLL
#define DWA_PLANNER_COMPONENT_PUBLIC \
  DWA_PLANNER_COMPONENT_EXPORT
#else
#define DWA_PLANNER_COMPONENT_PUBLIC \
  DWA_PLANNER_COMPONENT_IMPORT
#endif
#define DWA_PLANNER_COMPONENT_PUBLIC_TYPE \
  DWA_PLANNER_COMPONENT_PUBLIC
#define DWA_PLANNER_COMPONENT_LOCAL
#else
#define DWA_PLANNER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define DWA_PLANNER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define DWA_PLANNER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define DWA_PLANNER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define DWA_PLANNER_COMPONENT_PUBLIC
#define DWA_PLANNER_COMPONENT_LOCAL
#endif
#define DWA_PLANNER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "
#endif

#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include<math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

namespace dwa_planner
{
    class DWAPlanner : public rclcpp::Node
    {
        public:
            DWA_PLANNER_COMPONENT_PUBLIC
            explicit DWAPlanner(const rclcpp::NodeOptions & options);
            virtual ~DWAPlanner(void);

        private:
            void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
            void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
            geometry_msgs::msg::Point scan_to_cartesian();
            float calc_heading(geometry_msgs::msg::Point goal_point);
            float calc_dist(float linear, float angular);
            float calc_velocity(void);


            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
            
            sensor_msgs::msg::LaserScan scan;
            nav_msgs::msg::Odometry odometry;
            bool scan_update_flg;
            geometry_msgs::Twist current_velocity;
            bool odom_updated

            std::vector<geometry_msgs::msg::Point> point_list;

            struct Limit{
                float cost = 1e6;
                float min_obs_cost;
                float min_goal_cost;
                float min_speed_cost;
            }
            


    }
}

#endif

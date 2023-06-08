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
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#define DT 0.1

using namespace std::chrono_literals;

namespace dwa_planner
{
    typedef struct Limit_{
        float min_cost = 1e6;
        float min_obs_cost = 0.0;
        float min_goal_cost = 0.0;
        float min_speed_cost = 0.0;
    } Limit;

    typedef struct State_{
      float x_position=0.0;
      float y_position=0.0;
      float yaw=0.0;
      float velocity=0.0;
      float yawrate=0.0;
    } State;

    typedef struct Window_{
      float min_velocity;
      float max_velocity;
      float min_yawrate;
      float max_yawrate;
    } Window;

    class DWAPlanner : public rclcpp::Node
    {
        public:
            DWA_PLANNER_COMPONENT_PUBLIC
            explicit DWAPlanner(const rclcpp::NodeOptions & options);
            virtual ~DWAPlanner(void);

        private:
            void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
            void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
            void local_goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
            void process_callback(void);
            geometry_msgs::msg::Point scan_to_cartesian(void);
            float calc_heading(const std::vector<State> traj, geometry_msgs::msg::Pose goal_pose);
            float calc_dist(void);
            float calc_velocity(const std::vector<State> traj);
            Window calc_dynamic_window(void);
            void motion(State state, const float velocity, const double yawrate);

            std::vector<State> dwaplanner(Window dynamic_window, geometry_msgs::msg::Pose goal_pose);

            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_subscription_;

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
            rclcpp::TimerBase::SharedPtr timer_;

            sensor_msgs::msg::LaserScan scan;
            nav_msgs::msg::Odometry odometry;
            geometry_msgs::msg::PoseStamped local_goal;

            bool scan_update_flg;
            geometry_msgs::Twist current_velocity;
            bool odom_updated

            std::vector<geometry_msgs::msg::Point> point_list;

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            geometry_msgs::msg::TransformStamped transformStamped_;
            geometry_msgs::msg::Pose local_goal_pose_;


            float MAX_VELOCITY_;
            float MIN_VELOCITY_;
            float MAX_ACCELERATION_;
            float MAX_ANGULAR_ACCELERATION_;
            float MAX_YAWRATE_;
            float VELOCITY_RESOLUTION_;
            float YAWRATE_RESOLUTION_;
            float PREDICT_TIME_;

            float HEADING_COST_GAIN;
            float DIST_COST_GAIN;
            float VELOCITY_COST_GAIN;

            float TARGET_VELOCITY;
            std::string ROBOT_FRAME;
            std::string SOURCE_FRAME;

    }
}

#endif

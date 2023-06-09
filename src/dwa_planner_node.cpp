#include "rclcpp/rclcpp.hpp"
#include "dwa_planner/dwa_planner_component.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto dwa_planner_component = std::make_shared<dwa_planner::DWAPlanner>(rclcpp::NodeOptions());
    exec.add_node(dwa_planner_component);
    exec.spin();
    rclcpp::shutdown();
}
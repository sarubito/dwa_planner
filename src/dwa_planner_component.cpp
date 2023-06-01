#include "dwa_planner/dwa_planner_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace dwa_planner
{
    DWAPlanner::DwaPlanner(const rclcpp::NodeOptions & options) : Node("dwa_planner", options)
    {
        scan_update_flg = false;
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&DWAPlanner::scan_callback, this, std::placeholders::_1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&DWAPlanner::odom_callback, this, std::placeholders::_1));
        local_goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/local_goal", 1, std::bind(&DWAPlanner::local_goal_callback, this, std::placeholders::_1));

        this->declare_parameter("MAX_VELOCITY", 0.0);
        this->get_parameter("MAX_VELOCITY", MAX_VELOCITY_);
        this->declare_parameter("MIN_VELOCITY", 0.0);
        this->get_parameter("MIN_VELOCITY", MIN_VELOCITY_);
        this->declare_parameter("MAX_ACCELERATION", 0.0);
        this->get_parameter("MAX_ACCELERATION", MAX_ACCELERATION_);
        this->declare_parameter("MAX_ANGULAR_ACCELERATION", 0.0);
        this->get_parameter("MAX_ANGULAR_ACCELERATION", MAX_ANGULAR_ACCELERATION_);
        this->declare_parameter("MAX_YAWRATE", 0.0);
        this->get_parameter("MAX_YAWRATE", MAX_YAWRATE_);
        this->declare_parameter("VELOCITY_RESOLUTION", 0.0);
        this->get_parameter("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION_);
        this->declare_parameter("YAWRATE_RESOLUTION", 0.0);
        this->get_parameter("YAWRATE_RESOLUTION", YAWRATE_RESOLUTION_);
        this->declare_parameter("PREDICT_TIME", 0.0);
        this->get_parameter("PREDICT_TIME", PREDICT_TIME_);

        this->declare_parameter("HEADING_COST_GAIN", 0.0);
        this->get_parameter("HEADING_COST_GAIN", HEADING_COST_GAIN);
        this->declare_parameter("DIST_COST_GAIN", 0.0);
        this->get_parameter("DIST_COST_GAIN", DIST_COST_GAIN);
        this->declare_parameter("VELOCITY_COST_GAIN", 0.0);
        this->get_parameter("VELOCITY_COST_GAIN", VELOCITY_COST_GAIN);

        this->declare_parameter("TARGET_VELOCITY", 0.0);
        this->get_parameter("TARGET_VELOCITY", TARGET_VELOCITY);

        this->declare_parameter("ROBOT_FRAME", "base_link");
        this->get_parameter("ROBOT_FRAME", ROBOT_FRAME);
        this->declare_parameter("SOURCE_FRAME", "map");
        this->get_parameter("SOURCE_FRAME", SOURCE_FRAME);


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

    void DWAPlanner::local_goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        local_goal = *msg;
        //地図座標系からロボット座標系に座標変換
        try{
            transformStamped = tfBuffer.lookupTransform(ROBOT_FRAME, SOURCE_FRAME, this->get_clock()->now());
            point.x = transformStamped.transform
        } catch(tf2::TransformException ex){
            RCLCPP_INFO(this->get_logger(),"%s", ex.what());
        }
    }

    Window DWAPlanner::calc_dynamic_window(void)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear = robot_odometry.twist.twist.linear;
        Window window;
        window.min_velocity = std::max((twist.linear.x - MAX_ACCELERATION_*DT), MIN_VELOCITY_);
        window.max_velocity = std::min((twist.linear.x + MAX_ACCELERATION_*DT),MAX_VELOCITY_);
        window.min_yawrate = std::max((twist.angular.z - MAX_ANGULAR_ACCELERATION_*DT), -MAX_YAWRATE_);
        window.max_yawrate = std::min((twist.angular.z + MAX_ANGULAR_ACCELERATION_*DT), MAX_YAWRATE_);

        return window;
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

    float DWAPlanner::calc_dist(void)
    {
        float max_distance;
        bool flg = true;
        std::vector<geometry_msgs::msg::Point> point_list = scan_to_cartesian();
        for(auto &v : point_list){
            float distance = sqrt(pow(robot_odometry.pose.pose.position.y - v.y, robot_odometry.pose.pose.position.x - v.x));
            if(flg == true){
                flg = false;
                max_distance = distance;
            }else{
                if(max_distance < distance) max_distance = distance;
            }
        }
        
        return max_distance;
    }

    float DWAPlanner::calc_velocity(const std::vector<State> traj, const float target_velocity)
    {
        float cost = fabs(target_velocity - fabs(traj[traj.size()-1].velocity)); //軌跡の終端の速度と目的の速度との差
        return cost;
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

    State DWAPlanner::motion(State state, float velocity, float yawrate)
    { 
        state.yaw += yawrate*DT;
        state.x += velocity*cos(state.yaw)*DT;
        state.y += velocity*sin(state.yaw)*DT;
        state.velocity = velocity;
        state.yawrate = yawrate;

        return state;
    }

    void DWAPlanner::dwaplanner(Window dynamic_window, geometry_msgs::msg::Point goal_point, std::vector<geometry_msgs::msg::Point> point_list)
    {
        Limit limit_;
        State state_;
        state_.velocity = robot_odometry.twist.twist.linear.x;
        state_.yawrate = robot_odometry.twist.twist.angular.z;
        for(float v=dynamic_window.min_velocity; v<=dynamic_window.max_velocity; v+=VELOCITY_RESOLUTION_){
            for(float w=dynamic_window.min_yawrate; w<=dynamic_window.max_yawrate; w+=YAWRATE_RESOLUTION_){
                state_.x_position = 0.0;
                state_.y_position = 0.0;
                state_.yaw = 0.0;
                state_.velocity = robot_odometry.twist.twist.linear.x;
                state_.yawrate = robot_odometry.twist.twist.angular.z;
                //予測軌跡
                std::vector<State> traj_;
                for(float t=0.0; t<=PREDICT_TIME_; t+=DT){
                    state_ = motion(state_, v, w);
                    traj_.push_back(state_);
                }
                
                float heading = calc_heading();
                float dist = calc_dist();
                float velocity = calc_velocity(traj_, TARGET_VELOCITY);
                float final_cost = (HEADING_COST_GAIN * heading) + (DIST_COST_GAIN * dist) + (VELOCITY_COST_GAIN * velocity); 
            }
        }

    }

}
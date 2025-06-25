#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"


enum class RobotState{
    MOVING_STRAIGHT,
    TURNING_LEFT,
    TURNING_RIGHT,
    STOP
};


class MazeSolving : public rclcpp::Node{
    public:
        MazeSolving() : Node("maze_solving"){
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10,
                std::bind(&MazeSolving::lidarCallback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }

    private:
        void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            analyze_obstacles(msg);
            determine_state();
            publish_velocity();
        }

        void analyze_obstacles(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            right_obstacle = *std::min_element(msg->ranges.begin() + 260,
                                                    msg->ranges.begin() + 280);
            front_obstacle = *std::min_element(msg->ranges.begin() + 340,
                                                    msg->ranges.begin() + 360);
            left_obstacle = *std::min_element(msg->ranges.begin() + 80,
                                                    msg->ranges.begin() + 100);
            RCLCPP_INFO(this->get_logger(), "\n Front: %f \n Right: %f \n Left: %f \n",
                        sanitize_range(front_obstacle), sanitize_range(right_obstacle), sanitize_range(left_obstacle));

            // if(front_obstacle == std::numeric_limits<float>::infinity() &&
            //     right_obstacle == std::numeric_limits<float>::infinity() &&
            //     left_obstacle == std::numeric_limits<float>::infinity()){
            //     state_ = RobotState::STOP;
            // }
        }
        void determine_state(){
            if(front_obstacle <= front_threshold){
                if(sanitize_range(right_obstacle) < sanitize_range(left_obstacle)){
                    state_ = RobotState::TURNING_LEFT;
                }else if(sanitize_range(right_obstacle) > sanitize_range(left_obstacle)){
                    state_ = RobotState::TURNING_RIGHT;
                }
            }else{
                state_ = RobotState::MOVING_STRAIGHT;
            }
        }

        float sanitize_range(float range) {
            if (std::isnan(range) || std::isinf(range)) return 10.0;
            return range;
        }

        void publish_velocity(){
            switch (state_){
            case RobotState::MOVING_STRAIGHT:
                cmd_vel.linear.x = linear_vel;
                cmd_vel.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Moving Straight");
                break;
            case RobotState::TURNING_LEFT:
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = angular_vel;
                RCLCPP_INFO(this->get_logger(), "Turning Left");
                break;
            case RobotState::TURNING_RIGHT:
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -angular_vel;
                RCLCPP_INFO(this->get_logger(), "Turning Right");
                break;
            case RobotState::STOP:
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "STOP");
                break;

            default:
                break;
            }
            
            publisher_->publish(cmd_vel);
        }
        float front_threshold = 2.0;
        float angular_vel = 0.8;
        float linear_vel = 0.6;
        float right_obstacle, front_obstacle, left_obstacle;
        geometry_msgs::msg::Twist cmd_vel;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        RobotState state_;
};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeSolving>());
    rclcpp::shutdown();
    return 0;
}
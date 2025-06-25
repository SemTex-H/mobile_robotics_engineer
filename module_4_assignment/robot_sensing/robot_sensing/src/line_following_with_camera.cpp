#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


class LineFollowing : public rclcpp::Node{
    public:
        LineFollowing() : Node("line_following"){
            this->declare_parameter<int>("lower_threshold", 200);
            this->declare_parameter<int>("upper_threshold", 250);
            this->declare_parameter<int>("row_val", 650);
            this->declare_parameter<int>("column_val", 540);
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", 10,
                std::bind(&LineFollowing::cameraCallback, this, std::placeholders::_1));
        }
    private:
        void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg){

            geometry_msgs::msg::Twist cmd_vel;
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            cv::Mat gray_image, canny_image;
            cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
            int upper_threshold = this->get_parameter("upper_threshold").as_int();
            int lower_threshold = this->get_parameter("lower_threshold").as_int();
            int row_val = this->get_parameter("row_val").as_int();
            int column_val = this->get_parameter("column_val").as_int();
            cv::Canny(gray_image, canny_image, lower_threshold, upper_threshold);

            int row = row_val;
            int column = column_val;
            cv::Mat roi = canny_image(cv::Range(row, row+240), cv::Range(column, column+540));


            std::vector<int> edge;
            for(int i=0; i<540; ++i){
                if(roi.at<uchar>(160, i) == 255){
                    edge.push_back(i);
                }
            }

            switch (edge.size())
            {
            case 1:
                count_1_edge++;
                break;
            case 2:
                count_2_edge++;
                break;
            case 3:
                count_3_edge++;
                break;
            case 4:
                count_4_edge++;
                break;
            default:
                break;
            }
            double error = 0;
            if (edge.size() >= 2) {
                int mid_area = edge[1] - edge[0];
                int mid_point = edge[0] + mid_area/2;
                int robot_mid_point = 540/2;

                error = robot_mid_point - mid_point;
                cv::circle(roi, cv::Point(mid_point, 160), 2, cv::Scalar(255, 255, 255), -1);
                cv::circle(roi, cv::Point(robot_mid_point, 160), 5, cv::Scalar(255, 255, 255), -1);
            }else{
                RCLCPP_WARN(this->get_logger(), "Not enough edges detected: %zu", edge.size());
                error = 0.0;
                cmd_vel.angular.z = 0.0;
                return;
            }
            cmd_vel.linear.x = 0.2;
            if(error>0){
                cmd_vel.angular.z = -0.4;
                RCLCPP_INFO(this->get_logger(), "Turn Left");
            }else if(error<0){
                cmd_vel.angular.z = 0.4;
                RCLCPP_INFO(this->get_logger(), "Turn Right");
            }

            publisher_->publish(cmd_vel);
            cv::imshow("Image", roi);
            cv::waitKey(1);
        }

        int count_1_edge = 0;
        int count_2_edge = 0;
        int count_3_edge = 0;
        int count_4_edge = 0;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowing>());
    rclcpp::shutdown();
    return 0;
}
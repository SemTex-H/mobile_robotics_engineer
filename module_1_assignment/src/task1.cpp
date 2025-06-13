#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/LinearMath/Quaternion.h>

class StaticFramePublisher : public rclcpp::Node{
    public:
        StaticFramePublisher() : Node("static_frame_publisher"), broadcaster_(this){
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                            std::bind(&StaticFramePublisher::timer_callback, this));
        }
    
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        tf2_ros::StaticTransformBroadcaster broadcaster_;
        double x_trans;

        void timer_callback(){
            geometry_msgs::msg::TransformStamped t;
            geometry_msgs::msg::TransformStamped t1;
            geometry_msgs::msg::TransformStamped t2;
            geometry_msgs::msg::TransformStamped t3;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "base";
            t.child_frame_id = "dock_link";
            t.transform.translation.x = 0.1;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            // 
            t1.header.stamp = this->get_clock()->now();
            t1.header.frame_id = "dock_link";
            t1.child_frame_id = "stick1_link";
            t1.transform.translation.x = 0.0;
            t1.transform.translation.y = 0.0;
            t1.transform.translation.z = 0.6;
            tf2::Quaternion q1;
            q1.setRPY(0, 0, 0);
            t1.transform.rotation.x = q1.x();
            t1.transform.rotation.y = q1.y();
            t1.transform.rotation.z = q1.z();
            t1.transform.rotation.w = q1.w();
            //
            t2.header.stamp = this->get_clock()->now();
            t2.header.frame_id = "stick1_link";
            t2.child_frame_id = "stick2_link";
            t2.transform.translation.x = 0.0;
            t2.transform.translation.y = 0.1;
            t2.transform.translation.z = 1.1;
            tf2::Quaternion q2;
            q2.setRPY(0, 0, 0);
            t2.transform.rotation.x = q2.x();
            t2.transform.rotation.y = q2.y();
            t2.transform.rotation.z = q2.z();
            t2.transform.rotation.w = q2.w();
            //
            t3.header.stamp = this->get_clock()->now();
            t3.header.frame_id = "stick2_link";
            t3.child_frame_id = "stick3_link";
            t3.transform.translation.x = 0.0;
            t3.transform.translation.y = -0.1;
            t3.transform.translation.z = 1.6;
            tf2::Quaternion q3;
            q3.setRPY(0, 0, 0);
            t3.transform.rotation.x = q3.x();
            t3.transform.rotation.y = q3.y();
            t3.transform.rotation.z = q3.z();
            t3.transform.rotation.w = q3.w();
            broadcaster_.sendTransform(t);
            broadcaster_.sendTransform(t1);
            broadcaster_.sendTransform(t2);
            broadcaster_.sendTransform(t3);
        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>());
    rclcpp::shutdown();
    return 0;
}


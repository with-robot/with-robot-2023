#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

class CarOdom:public rclcpp::Node
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_raw;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        double scale_x = 1.0;
        double scale_y = 1.0;
        double scale_z = 1.0;

        double delta_time = 0.0;
        double vel_x = 0.0;
        double vel_y = 0.0;
        double vel_z = 0.0;
        double delta_x = 0.0;
        double delta_y = 0.0;
        double delta_z = 0.0;
        double pos_x = 0.0;
        double pos_y = 0.0;
        double pos_z = 0.0;

        rclcpp::Time last_time;
        geometry_msgs::msg::TransformStamped tf;
    
    public:
        CarOdom() : Node("car_odom"){
            this->declare_parameter<double>("scale_x", 1.0);
            this->declare_parameter<double>("scale_y", 1.0);
            this->declare_parameter<double>("scale_z", 1.0);

            this->get_parameter<double>("scale_x", scale_x);
            this->get_parameter<double>("scale_y", scale_y);
            this->get_parameter<double>("scale_z", scale_z);

            tf.header.frame_id = "odom";
            tf.child_frame_id = "base_link";

            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            sub_vel_raw = this->create_subscription<geometry_msgs::msg::TwistStamped>("vel_raw", 10, std::bind(&CarOdom::vel_raw_callback, this, _1));
        }
    
    private:
        void vel_raw_callback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg){
            // delta_time
            rclcpp::Time curr_time = msg->header.stamp;
            if (last_time.seconds() == 0){
                last_time = curr_time;
                return;
            }
            delta_time = (curr_time - last_time).seconds();
            last_time = curr_time;
            // recv value
            vel_x = msg->twist.linear.x * scale_x;
            vel_y = msg->twist.linear.y * scale_y;
            vel_z = msg->twist.angular.z * scale_z;
            // delta value
            delta_x = (vel_x * cos(pos_z) - vel_y * sin(pos_z)) * delta_time;
            delta_y = (vel_x * sin(pos_z) + vel_y * cos(pos_z)) * delta_time;
            delta_z = vel_z * delta_time;
            // position value
            pos_x += delta_x;
            pos_y += delta_y;
            pos_z += delta_z;
            // Euler to Quaternion
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.00, 0.00, pos_z);

            // publish tf
            tf.header.stamp = curr_time;
            tf.transform.translation.x = pos_x;
            tf.transform.translation.y = pos_y;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.x = quaternion.x();
            tf.transform.rotation.y = quaternion.y();
            tf.transform.rotation.z = quaternion.z();
            tf.transform.rotation.w = quaternion.w();
            tf_broadcaster->sendTransform(tf);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarOdom>());
    rclcpp::shutdown();
    return 0;
}
#ifndef RELBOT_CONTROL_HPP
#define RELBOT_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class DiffDriveController: public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the DiffDriveController class.
         * 
         * Initializes the DiffDriveController object. Sets up subscriptions to topics 
         * for bounding box and image messages, creates publishers for twist commands 
         * and debug images, and initializes parameters.
         */
        DiffDriveController();

    private:
        /// Callback functions.
        /**
         * @brief Callback function for controlling the robot based on bounding box information.
         * 
         * This function is called whenever a new bounding box message is received. 
         * It computes the control commands based on the bounding box information and publishes 
         * TwistStamped messages to control the robot's motion.
         *
         * @param bounding_box A shared pointer to the bounding box message.
         */
        void compute_diff_velocities(const geometry_msgs::msg::TwistStamped::SharedPtr rebot_twists);

        /// Subscriber variables.
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_twist_;

        /// Publisher variables.
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_right_vel_;
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_left_vel_;
};

#endif /* RELBOT_CONTROL_HPP */

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
         * Initializes the DiffDriveController object. Sets up subscriptions 
         * to twist messages, creates publishers for motor velocity commands, 
         * and initializes parameters.
         */
        DiffDriveController();

    private:
        /// Callback functions.
        /**
         * @brief Callback function for computing differential velocities.
         * 
         * This function computes the angular velocities for the left and right wheels
         * based on linear and angular velocity inputs and publishes them to control
         * the robot's motion.
         *
         * @param rebot_twists A shared pointer to the received TwistStamped message.
         */
        void compute_diff_velocities(const geometry_msgs::msg::TwistStamped::SharedPtr rebot_twists);

        /// Subscriber variables.
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_twist_;

        /// Publisher variables.
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_right_vel_;
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_left_vel_;
};

#endif /* RELBOT_CONTROL_HPP */

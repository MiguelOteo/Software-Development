#ifndef RELBOT_CONTROL_HPP
#define RELBOT_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "relbot_interfaces/msg/bounding_box.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class RELbotControl: public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the RELbotControl class.
         * 
         * Initializes the RELbotControl object. Sets up subscriptions to topics 
         * for bounding box and image messages, creates publishers for twist commands 
         * and debug images, and initializes parameters.
         */
        RELbotControl();

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
        void control_callback(const relbot_interfaces::msg::BoundingBox::SharedPtr bounding_box);

        /**
         * @brief Callback function for processing debug images.
         * 
         * This function is called whenever a new debug image message is received. 
         * If debug visualization is enabled, it draws a rectangle overlay on the image representing the 
         * reference bounding box and publishes the debug image.
         *
         * @param image A shared pointer to the debug image message.
         */
        void debug_image_callback(const sensor_msgs::msg::Image::SharedPtr image);

        /**
         * @brief Callback function for updating the center of the reference bounding box.
         * 
         * This function is called whenever a new image message is received. It updates the 
         * center point of the reference bounding box based on the resolution of the received image.
         *
         * @param msg A shared pointer to the image message.
         */
        void bounding_box_center(const sensor_msgs::msg::Image::SharedPtr image);

        /// Private variables.
        bool debug_visualization_;
        relbot_interfaces::msg::BoundingBox reference_bounding_box;
        double linear_gain_;
        double angular_gain_;

        /// Subscriber variables.
        rclcpp::Subscription<relbot_interfaces::msg::BoundingBox>::SharedPtr subscription_box_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_subscription_;

        /// Publisher variables.
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_twist_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_right_vel_;
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_left_vel_;
};

#endif /* RELBOT_CONTROL_HPP */

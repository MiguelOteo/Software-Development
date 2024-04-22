#ifndef RELBOT_TEST_CONTROL_HPP
#define RELBOT_TEST_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"

class RELbotTestController: public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the RELbotTestController class.
         */
        RELbotTestController();

        /**
         * @brief Callback to publish data 
        */
        void timer_callback();

    private:
        /// Publisher variables.
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_right_vel_;
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_left_vel_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif /* RELBOT_TEST_CONTROL_HPP */

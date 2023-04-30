#include <chrono>
#include <functional>
#include <memory>
#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class KeyboardSubscriber : public rclcpp::Node
{
public:
  KeyboardSubscriber() : Node("keyboard_subscriber")
  {
    // Set up pigpio interface
    pi_ = pigpio_start(NULL, NULL);
    if (pi_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio");
      return;
    }

    // Subscribe to keyboard input topic
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "keyboard_input",
      10,
      std::bind(&KeyboardSubscriber::keyboard_callback, this, std::placeholders::_1)
    );
  }

private:
  void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "w") {
      // Set GPIO pin 15 and 18 high
      set_mode(pi_, 15, PI_OUTPUT);
      set_mode(pi_, 18, PI_OUTPUT);
      gpio_write(pi_, 15, 1);
      gpio_write(pi_, 18, 1);
      RCLCPP_INFO(this->get_logger(), "GPIO pins 15 and 18 set high");
    } else {
      // Set GPIO pin 15 and 18 low
      set_mode(pi_, 15, PI_OUTPUT);
      set_mode(pi_, 18, PI_OUTPUT);
      gpio_write(pi_, 15, 0);
      gpio_write(pi_, 18, 0);
      RCLCPP_INFO(this->get_logger(), "GPIO pins 15 and 18 set low");
    }
  }

  int pi_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardSubscriber>());
  rclcpp::shutdown();
  return 0;
}

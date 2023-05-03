#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <pigpiod_if2.h>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;

class capsulerobot01 : public rclcpp::Node
{
public:
  capsulerobot01() : Node("capsulerobot01") 
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&capsulerobot01::joy_callback, this, std::placeholders::_1));

    pi_ = pigpio_start(NULL, NULL);
    if (pi_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio.");
      return;
    }

    //set GPIO modes
    set_mode(pi_, 15, PI_OUTPUT);
    set_mode(pi_, 18, PI_OUTPUT);
    set_mode(pi_, 23, PI_OUTPUT);
    set_mode(pi_, 24, PI_OUTPUT);

    //Set initial output status/
    gpio_write(pi_,15 ,PI_LOW);
    gpio_write(pi_,18 ,PI_LOW);
    gpio_write(pi_,23 ,PI_LOW);
    gpio_write(pi_,24 ,PI_LOW);

  }

  ~capsulerobot01()
  {
    pigpio_stop(pi_);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes[7] == -1.0 && msg->axes[6] == 0.0) // Dualshock4 axes forward
    {
      RCLCPP_INFO(this->get_logger(),"press forward");

      set_gpio_low(15); //right
      set_gpio_high(18);
      set_gpio_high(23);
      set_gpio_low(24);

    }
    else if (msg->axes[7] == 1.0 && msg->axes[6] == 0.0) // Dualshock4 axes backward
    {
      RCLCPP_INFO(this->get_logger(),"press backward");

      set_gpio_high(15);
      set_gpio_low(18);
      set_gpio_low(23);
      set_gpio_high(24);

    }

    else if (msg->axes[7] == 0.0 && msg->axes[6] == -1.0) // Dualshock4 axes backward
    {
      RCLCPP_INFO(this->get_logger(),"press right");

      set_gpio_low(15);
      set_gpio_high(18);
      set_gpio_low(23);
      set_gpio_high(24);
    
    }
    else if (msg->axes[7] == 0.0 && msg->axes[6] == 1.0) // Dualshock4 axes backward
    {
      RCLCPP_INFO(this->get_logger(),"press left");

      set_gpio_high(15);
      set_gpio_low(18);
      set_gpio_high(23);
      set_gpio_low(24);

      
    }

    else
    {
      set_gpio_low(15);
      set_gpio_low(18);
      set_gpio_low(23);
      set_gpio_low(24);
    }

 
  }

  void set_gpio_high(int gpio)
  {
    gpio_write(pi_, gpio, 1);
  }

  void set_gpio_low(int gpio)

  {
    gpio_write(pi_, gpio, 0);
  }

  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("capsulerobot01");
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  int pi_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<capsulerobot01>());
  rclcpp::shutdown();
  return 0;
}

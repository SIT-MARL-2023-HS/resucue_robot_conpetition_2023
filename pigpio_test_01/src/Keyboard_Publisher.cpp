#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class KeyboardPublisher : public rclcpp::Node
{
public:
  KeyboardPublisher()
  : Node("keyboard_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("keyboard_input", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&KeyboardPublisher::keyboardInputCallback, this));
  }

private:
  void keyboardInputCallback()
  {
    std::string input_str;
    std::cout << "Enter a keyboard character: ";
    std::getline(std::cin, input_str);

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = input_str;
    publisher_->publish(std::move(msg));
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardPublisher>());
  rclcpp::shutdown();
  return 0;
}

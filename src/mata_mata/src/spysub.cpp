#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class BatterySubscriber : public rclcpp::Node
{
public:
  BatterySubscriber() : Node("battery_subscriber")
  {
    RCLCPP_INFO(this->get_logger(), "Max Battery: %.2f Wh", MAXBATTERY);

    // --- TODO 1: Inisialisasi Subscriber ---
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_level", 10, std::bind(&BatterySubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Float32 &msg)
  {
    currentlevel = msg.data;

    // --- TODO 2: Perhitungan & Log Dasar ---
    // Formula: (Percentage / 100) * Max Capacity (20 Wh)
    float remaining_wh = (currentlevel / 100.0) * MAXBATTERY;

    RCLCPP_INFO(this->get_logger(), "Battery Level: %.0f%%", currentlevel);
    RCLCPP_INFO(this->get_logger(), "Current Battery: %.2f Wh", remaining_wh);

    // --- TODO 3: Logika Status Alert (If-Else) ---
    if (currentlevel > 70.0) {
      RCLCPP_INFO(this->get_logger(), "Status: HIGH");
    } 
    else if (currentlevel > 40.0) {
      RCLCPP_INFO(this->get_logger(), "Status: MEDIUM");
    } 
    else {
      // RCLCPP_WARN makes the text YELLOW in the terminal
      RCLCPP_WARN(this->get_logger(), "Status: LOW");
    }
    
    // Visual separator for clarity
    std::cout << "----------------------------" << std::endl;
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  const float MAXBATTERY = 20.00; // Total capacity in Wh
  float currentlevel;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatterySubscriber>());
  rclcpp::shutdown();
  return 0;
}
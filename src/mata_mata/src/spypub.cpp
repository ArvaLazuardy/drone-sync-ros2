#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class BatteryPublisher : public rclcpp::Node
{
public:
  BatteryPublisher() : Node("battery_publisher")
  {
    // --- TODO 1: Implementasi Input Awal ---    
    // Loop until we get a valid percentage between 0 and 100
    while (current_battery < 0 || current_battery > 100) {
      std::cout << "Initial Current Battery Level (0 - 100): ";
      std::cin >> current_battery;
      if (current_battery < 0 || current_battery > 100) {
        std::cout << "Invalid input! Please enter a value between 0 and 100." << std::endl;
      }
    }

    // --- TODO 2: Inisialisasi Publisher ---
    // Create a publisher on the "battery_level" topic
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_level", 10);

    // --- TODO 3: Inisialisasi Timer (Set to 500ms) ---
    timer_ = this->create_wall_timer(500ms, std::bind(&BatteryPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Check if battery is empty before publishing
    if (current_battery <= 0) {
      RCLCPP_INFO(this->get_logger(), "Battery reached 0%%. Shutting down...");
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    auto message = std_msgs::msg::Float32();
    
    // --- TODO 4: Persiapan & Publish Data ---
    message.data = current_battery;
    RCLCPP_INFO(this->get_logger(), "Publishing: Battery Level %.2f%%", message.data);
    publisher_->publish(message);

    // --- TODO 5: Simulasi Pengurasan ---
    current_battery -= 4.0; 
    
    // Ensure it doesn't show negative numbers in the final log
    if (current_battery < 0) current_battery = 0;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  float current_battery = -1; // Initial state to trigger the while loop
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryPublisher>()); 
  return 0;
}
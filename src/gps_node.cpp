#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<rtf_imu::rtfImu>());
  rclcpp::shutdown();
  return 0;
}
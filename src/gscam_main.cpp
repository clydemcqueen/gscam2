#include "gscam2/gscam_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  // Create NodeOptions, turn off IPC
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);

  // Create node
  auto node = std::make_shared<gscam2::GSCamNode>(options);

  // Set logging level
  auto result = rcutils_logging_set_logger_level(
    node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  (void) result;

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

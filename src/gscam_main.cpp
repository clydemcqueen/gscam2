#include "gscam/gscam_node.hpp"

#include "rclcpp/rclcpp.hpp"

// Launch GSCamNode with use_intra_process_comms=true

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<gscam::GSCamNode>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  if (!node->configure()) {
    RCLCPP_FATAL(node->get_logger(), "Failed to configure gscam!");
    return 1;
  }

  if (!node->init_stream()) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize gscam stream!");
    return 1;
  }

  // Spin as fast as possible, the loop will block on the stream
  while (rclcpp::ok()) {
    // Do our work
    node->spin_once();

    // Respond to incoming messages
    rclcpp::spin_some(node);
  }

  RCLCPP_INFO(node->get_logger(), "GStreamer stream stopped!");
  node->cleanup_stream();

  rclcpp::shutdown();
  return 0;
}

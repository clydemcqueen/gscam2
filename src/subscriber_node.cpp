#include "gscam2/subscriber_node.hpp"

namespace gscam2
{

ImageSubscriberNode::ImageSubscriberNode(const rclcpp::NodeOptions & options)
: Node("image_subscriber", options)
{
  (void) sub_;

  RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d", options.use_intra_process_comms());

  sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 10,
    [this](sensor_msgs::msg::Image::UniquePtr msg)
    {
      RCLCPP_INFO_ONCE(get_logger(), "receiving messages");    // NOLINT

#undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
      static int count = 0;
      RCLCPP_INFO(get_logger(), "%d, %p", count++, reinterpret_cast<std::uintptr_t>(msg.get()));
#else
      (void) this;
      (void) msg;
#endif
    });

  RCLCPP_INFO(get_logger(), "ready");
}

} // namespace gscam2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gscam2::ImageSubscriberNode)  // NOLINT

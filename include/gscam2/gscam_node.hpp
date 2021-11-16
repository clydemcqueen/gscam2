#ifndef __GSCAM_GSCAM_H
#define __GSCAM_GSCAM_H

#include "rclcpp/rclcpp.hpp"

namespace gscam2
{

class GSCamNode : public rclcpp::Node
{
  // Hide implementation
  class impl;
  std::unique_ptr<impl> pImpl_;

  void validate_parameters();

public:
  explicit GSCamNode(const rclcpp::NodeOptions & options);

  ~GSCamNode() override;
};

}

#endif // ifndef __GSCAM_GSCAM_H

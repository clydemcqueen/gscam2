#ifndef __GSCAM_GSCAM_H
#define __GSCAM_GSCAM_H

#include "camera_info_manager/camera_info_manager.h"

namespace gscam
{

  class GSCamNode : public rclcpp::Node
  {
    struct impl;
    std::unique_ptr<impl> pImpl;

    camera_info_manager::CameraInfoManager camera_info_manager_;

  public:
    GSCamNode();

    ~GSCamNode();

    bool configure();

    bool init_stream();

    void cleanup_stream();

    void spin_once();
  };

}

#endif // ifndef __GSCAM_GSCAM_H

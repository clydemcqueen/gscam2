#include "gscam/gscam_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace gscam {

  GSCamNode::GSCamNode():
    rclcpp::Node("gscam_publisher"),
    gsconfig_(""),
    pipeline_(NULL),
    sink_(NULL),
    camera_info_manager_(this)
  {
  }

  GSCamNode::~GSCamNode()
  {
  }

  bool GSCamNode::configure()
  {
    // Example config:
    // export GSCAM_CONFIG="udpsrc port=5600 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert"

    // Get gstreamer configuration
    // (either from environment variable or ROS param)
    std::string gsconfig_rosparam = "";
    bool gsconfig_rosparam_defined = false;
    char *gsconfig_env = NULL;

    // Declare parameters
    declare_parameter("gscam_config");
    declare_parameter("sync_sink", true);
    declare_parameter("preroll", false);
    declare_parameter("use_gst_timestamps", false);
    declare_parameter("camera_info_url", "");
    declare_parameter("camera_name", "");
    declare_parameter("image_encoding", std::string(sensor_msgs::image_encodings::RGB8));
    declare_parameter("frame_id");

    gsconfig_rosparam_defined = get_parameter("gscam_config", gsconfig_rosparam);
    gsconfig_env = getenv("GSCAM_CONFIG");

    if (!gsconfig_env && !gsconfig_rosparam_defined) {
      RCLCPP_FATAL(get_logger(),
                   "Problem getting GSCAM_CONFIG environment variable and 'gscam_config' rosparam is not set. This is needed to set up a gstreamer pipeline.");
      return false;
    } else if (gsconfig_env && gsconfig_rosparam_defined) {
      RCLCPP_FATAL(get_logger(),
                   "Both GSCAM_CONFIG environment variable and 'gscam_config' rosparam are set. Please only define one.");
      return false;
    } else if (gsconfig_env) {
      gsconfig_ = gsconfig_env;
      RCLCPP_INFO(get_logger(), "Using gstreamer config from env: %s", gsconfig_env);
    } else if (gsconfig_rosparam_defined) {
      gsconfig_ = gsconfig_rosparam;
      RCLCPP_INFO(get_logger(), "Using gstreamer config from rosparam: %s", gsconfig_rosparam.c_str());
    }

    // Get additional gscam configuration
    get_parameter("sync_sink", sync_sink_);
    get_parameter("preroll", preroll_);
    get_parameter("use_gst_timestamps", use_gst_timestamps_);

    // Get the camera parameters file
    get_parameter("camera_info_url", camera_info_url_);
    get_parameter("camera_name", camera_name_);

    // Get the image encoding
    get_parameter("image_encoding", image_encoding_);
    if (image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
        image_encoding_ != sensor_msgs::image_encodings::MONO8 &&
        image_encoding_ != "jpeg") {
      RCLCPP_FATAL(get_logger(), "Unsupported image encoding: %s", image_encoding_.c_str());
    }

    camera_info_manager_.setCameraName(camera_name_);

    if (camera_info_manager_.validateURL(camera_info_url_)) {
      camera_info_manager_.loadCameraInfo(camera_info_url_);
      RCLCPP_INFO(get_logger(), "Loaded camera calibration from %s", camera_info_url_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Camera info at: %s not found. Using an uncalibrated config.", camera_info_url_.c_str());
    }

    // Get TF Frame
    if (!get_parameter("frame_id", frame_id_)) {
      frame_id_ = "/camera_frame";
      RCLCPP_WARN(get_logger(), "No camera frame_id set, using frame %s", frame_id_.c_str());
      rclcpp::Parameter frame_id_param{"frame_id", frame_id_};
      set_parameter(frame_id_param);
    }

    return true;
  }

  bool GSCamNode::init_stream()
  {
    if (!gst_is_initialized()) {
      // Initialize gstreamer pipeline
      RCLCPP_DEBUG(get_logger(), "Initializing gstreamer...");
      gst_init(0, 0);
    }

    RCLCPP_DEBUG(get_logger(), "Gstreamer Version: %s", gst_version_string());

    GError *error = 0; // Assignment to zero is a gst requirement

    pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
    if (pipeline_ == NULL) {
      RCLCPP_FATAL(get_logger(), error->message);
      return false;
    }

    // Create RGB sink
    sink_ = gst_element_factory_make("appsink", NULL);
    GstCaps *caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));

    // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
    if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
      caps = gst_caps_new_simple("video/x-raw",
                                 "format", G_TYPE_STRING, "RGB",
                                 NULL);
    } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
      caps = gst_caps_new_simple("video/x-raw",
                                 "format", G_TYPE_STRING, "GRAY8",
                                 NULL);
    } else if (image_encoding_ == "jpeg") {
      caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
    }

    gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
    gst_caps_unref(caps);

    // Set whether the sink should sync
    // Sometimes setting this to true can cause a large number of frames to be
    // dropped
    gst_base_sink_set_sync(
      GST_BASE_SINK(sink_),
      (sync_sink_) ? TRUE : FALSE);

    if (GST_IS_PIPELINE(pipeline_)) {
      GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
      g_assert(outpad);

      GstElement *outelement = gst_pad_get_parent_element(outpad);
      g_assert(outelement);
      gst_object_unref(outpad);

      if (!gst_bin_add(GST_BIN(pipeline_), sink_)) {
        RCLCPP_FATAL(get_logger(), "gst_bin_add() failed");
        gst_object_unref(outelement);
        gst_object_unref(pipeline_);
        return false;
      }

      if (!gst_element_link(outelement, sink_)) {
        RCLCPP_FATAL(get_logger(), "GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));
        gst_object_unref(outelement);
        gst_object_unref(pipeline_);
        return false;
      }

      gst_object_unref(outelement);
    } else {
      GstElement *launchpipe = pipeline_;
      pipeline_ = gst_pipeline_new(NULL);
      g_assert(pipeline_);

      gst_object_unparent(GST_OBJECT(launchpipe));

      gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);

      if (!gst_element_link(launchpipe, sink_)) {
        RCLCPP_FATAL(get_logger(), "GStreamer: cannot link launchpipe -> sink");
        gst_object_unref(pipeline_);
        return false;
      }
    }

    // Calibration between rclcpp::Time and gst timestamps
    GstClock *clock = gst_system_clock_obtain();
    GstClockTime ct = gst_clock_get_time(clock);
    gst_object_unref(clock);
    time_offset_ = now().seconds() - GST_TIME_AS_USECONDS(ct) / 1e6;
    RCLCPP_INFO(get_logger(), "Time offset: %.3f", time_offset_);

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);

    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_FATAL(get_logger(), "Failed to PAUSE stream, check your gstreamer configuration.");
      return false;
    } else {
      RCLCPP_DEBUG(get_logger(), "Stream is PAUSED.");
    }

    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 1);
    if (image_encoding_ == "jpeg") {
      jpeg_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("camera/image_raw/compressed", 1);
    } else {
      camera_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 1);
    }

    RCLCPP_INFO(get_logger(), "Publishing stream...");

    // Pre-roll camera if needed
    if (preroll_) {
      RCLCPP_DEBUG(get_logger(), "Performing preroll...");

      // The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
      // I am told this is needed and am erring on the side of caution.
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
      if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_ERROR(get_logger(), "Failed to PLAY during preroll.");
        return false;
      } else {
        RCLCPP_DEBUG(get_logger(), "Stream is PLAYING in preroll.");
      }

      gst_element_set_state(pipeline_, GST_STATE_PAUSED);
      if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_ERROR(get_logger(), "Failed to PAUSE.");
        return false;
      } else {
        RCLCPP_INFO(get_logger(), "Stream is PAUSED in preroll.");
      }
    }

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Could not start stream!");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Started stream.");

    return true;
  }

  void GSCamNode::spin_once()
  {
    // This should block until a new frame is awake, this way, we'll run at the
    // actual capture framerate of the device.
    // RCLCPP_DEBUG(get_logger(), "Getting data...");
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
    if (!sample) {
      RCLCPP_ERROR(get_logger(), "Could not get gstreamer sample.");
      return;
    }
    GstBuffer *buf = gst_sample_get_buffer(sample);
    GstMemory *memory = gst_buffer_get_memory(buf, 0);
    GstMapInfo info;

    gst_memory_map(memory, &info, GST_MAP_READ);
    gsize &buf_size = info.size;
    guint8 *&buf_data = info.data;
    GstClockTime bt = gst_element_get_base_time(pipeline_);
    // RCLCPP_INFO(get_logger(), "New buffer: timestamp %.6f %lu %lu %.3f",
    //         GST_TIME_AS_USECONDS(buf->timestamp+bt)/1e6+time_offset_, buf->timestamp, bt, time_offset_);

    // Stop on end of stream
    if (!buf) {
      RCLCPP_INFO(get_logger(), "Stream ended.");
      return;
    }

    // RCLCPP_DEBUG(get_logger(), "Got data.");

    // Get the image width and height
    GstPad *pad = gst_element_get_static_pad(sink_, "sink");
    const GstCaps *caps = gst_pad_get_current_caps(pad);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &width_);
    gst_structure_get_int(structure, "height", &height_);

    // Update header information
    camera_info_manager::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
    auto cinfo = std::make_unique<sensor_msgs::msg::CameraInfo>(cur_cinfo);
    if (use_gst_timestamps_) {
      cinfo->header.stamp = rclcpp::Time(GST_TIME_AS_USECONDS(buf->pts + bt) / 1e6 + time_offset_);
    } else {
      cinfo->header.stamp = now();
    }
    // RCLCPP_INFO(get_logger(), "Image time stamp: %.3f",cinfo->header.stamp.toSec());
    cinfo->header.frame_id = frame_id_;
    if (image_encoding_ == "jpeg") {
      auto img = std::make_unique<sensor_msgs::msg::CompressedImage>();
      img->header = cinfo->header;
      img->format = "jpeg";
      img->data.resize(buf_size);
      std::copy(buf_data, (buf_data) + (buf_size), img->data.begin());
      jpeg_pub_->publish(std::move(img));
      cinfo_pub_->publish(std::move(cinfo));
    } else {
      // Complain if the returned buffer is smaller than we expect
      const unsigned int expected_frame_size =
        image_encoding_ == sensor_msgs::image_encodings::RGB8
        ? width_ * height_ * 3
        : width_ * height_;

      if (buf_size < expected_frame_size) {
        RCLCPP_WARN(get_logger(), "GStreamer image buffer underflow: Expected frame to be %d bytes but got only %d"
                                  " bytes. (make sure frames are correctly encoded)", expected_frame_size, (buf_size));
      }

      // Construct Image message
      auto img = std::make_unique<sensor_msgs::msg::Image>();

      img->header = cinfo->header;

      // Image data and metadata
      img->width = width_;
      img->height = height_;
      img->encoding = image_encoding_;
      img->is_bigendian = false;
      img->data.resize(expected_frame_size);

      // Copy only the data we received
      // Since we're publishing shared pointers, we need to copy the image so
      // we can free the buffer allocated by gstreamer
      if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
        img->step = width_ * 3;
      } else {
        img->step = width_;
      }
      std::copy(
        buf_data,
        (buf_data) + (buf_size),
        img->data.begin());

      // Publish the image/info
      camera_pub_->publish(std::move(img));
      cinfo_pub_->publish(std::move(cinfo));
    }

    // Release the buffer
    gst_memory_unmap(memory, &info);
    gst_memory_unref(memory);
    gst_buffer_unref(buf);
  }

  void GSCamNode::cleanup_stream()
  {
    // Clean up
    RCLCPP_INFO(get_logger(), "Stopping gstreamer pipeline...");
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = NULL;
    }
  }

} // namespace gscam

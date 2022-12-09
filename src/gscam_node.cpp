#include "gscam2/gscam_node.hpp"

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "camera_info_manager/camera_info_manager.hpp"
#include "ros2_shared/context_macros.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace gscam2
{

//=============================================================================
// Parameters
//=============================================================================

#define GSCAM_ALL_PARAMS \
  CXT_MACRO_MEMBER(gst_plugin_path, std::string, "")        /* Additional plugin path  */ \
  CXT_MACRO_MEMBER(gscam_config, std::string, "")           /* Stream config  */ \
  CXT_MACRO_MEMBER(sync_sink, bool, true)                   /* Sync to the clock  */ \
  CXT_MACRO_MEMBER(preroll, bool, false)                    /* Pre-fill buffers  */ \
  CXT_MACRO_MEMBER(use_gst_timestamps, bool, false)         /* Use gst time instead of ROS time  */ \
  CXT_MACRO_MEMBER(image_encoding, std::string, sensor_msgs::image_encodings::RGB8) /*   */ \
  CXT_MACRO_MEMBER(camera_info_url, std::string, "")        /* Location of camera info file  */ \
  CXT_MACRO_MEMBER(camera_name, std::string, "")            /* Camera name  */ \
  CXT_MACRO_MEMBER(frame_id, std::string, "camera_frame")   /* Camera frame id  */ \
  /* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct GSCamContext
{
  CXT_MACRO_DEFINE_MEMBERS(GSCAM_ALL_PARAMS)    // NOLINT
};

//=============================================================================
// GSCamNode::impl
//=============================================================================

class GSCamNode::impl
{
  // ROS node
  rclcpp::Node * node_;

  // Manage camera info
  camera_info_manager::CameraInfoManager camera_info_manager_;

  // Gstreamer structures
  GstElement * pipeline_;
  GstElement * sink_;

  // We need to poll GStreamer to get data
  // Move this to its own thread to avoid blocking or slowing down the rclcpp::spin() thread
  std::thread pipeline_thread_;

  // Used to stop the pipeline thread
  std::atomic<bool> stop_signal_;

  // Discover width and height from the incoming data
  int width_, height_;

  // Calibration between ros::Time and gst timestamps
  GstClockTime time_offset_;

  // Publish images...
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;

  // ... or compressed images
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_pub_;

  // Publish camera info
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;

  // Create gstreamer pipeline, return true if successful
  bool create_pipeline();

  // Delete gstreamer pipeline
  void delete_pipeline();

  // Process one frame
  void process_frame();

public:
  // Parameters
  GSCamContext cxt_;

  // Constructor
  explicit impl(rclcpp::Node * node)
  : node_(node),
    camera_info_manager_(node),
    pipeline_(nullptr),
    sink_(nullptr),
    stop_signal_(false),
    width_(0),
    height_(0),
    time_offset_(0)
  {
  }

  // Destructor
  ~impl()
  {
    if (pipeline_) {
      // Stop thread
      stop_signal_ = true;
      pipeline_thread_.join();

      // Delete pipeline
      delete_pipeline();
    }
  }

  // Start or re-start pipeline
  void restart();
};

bool GSCamNode::impl::create_pipeline()
{
  if (!gst_is_initialized()) {
    // Only need to do this once
    gst_init(nullptr, nullptr);
    RCLCPP_INFO(node_->get_logger(), "Gstreamer initialized");

    if (!cxt_.gst_plugin_path_.empty()) {
      auto registry = gst_registry_get();
      if (!registry) {
        RCLCPP_ERROR(node_->get_logger(), "Could not get registry, ignoring gst_plugin_path");
      } else {
        if (gst_registry_scan_path(registry, cxt_.gst_plugin_path_.c_str())) {
          RCLCPP_INFO(
            node_->get_logger(), "Scanned '%s' and added plugins to the registry",
            cxt_.gst_plugin_path_.c_str());
        } else {
          RCLCPP_WARN(
            node_->get_logger(), "Scanned '%s' for plugins, but the registry did not change",
            cxt_.gst_plugin_path_.c_str());
        }
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Gstreamer version: %s", gst_version_string());

  GError * error = nullptr;
  pipeline_ = gst_parse_launch(cxt_.gscam_config_.c_str(), &error);
  if (!pipeline_) {
    RCLCPP_FATAL(node_->get_logger(), error->message);
    return false;
  }

  // Create RGB sink
  sink_ = gst_element_factory_make("appsink", nullptr);
  GstCaps * caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));

  // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
  if (cxt_.image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "RGB",
      nullptr);
  } else if (cxt_.image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "GRAY8",
      nullptr);
  } else if (cxt_.image_encoding_ == sensor_msgs::image_encodings::YUV422_YUY2) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "YUY2",
      nullptr);
  } else if (cxt_.image_encoding_ == "jpeg") {
    caps = gst_caps_new_simple("image/jpeg", nullptr, nullptr);
  }

  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);

  // Set whether the sink should sync
  // Sometimes setting this to true can cause a large number of frames to be dropped
  gst_base_sink_set_sync(
    GST_BASE_SINK(sink_),
    (cxt_.sync_sink_) ? TRUE : FALSE);

  if (GST_IS_PIPELINE(pipeline_)) {
    GstPad * outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
    g_assert(outpad);

    GstElement * outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);

    if (!gst_bin_add(GST_BIN(pipeline_), sink_)) {
      RCLCPP_FATAL(node_->get_logger(), "gst_bin_add() failed");
      gst_object_unref(outelement);
      return false;
    }

    if (!gst_element_link(outelement, sink_)) {
      RCLCPP_FATAL(
        node_->get_logger(), "Cannot link outelement(\"%s\") -> sink\n",
        gst_element_get_name(outelement));
      gst_object_unref(outelement);
      return false;
    }

    gst_object_unref(outelement);
  } else {
    GstElement * launchpipe = pipeline_;
    pipeline_ = gst_pipeline_new(nullptr);
    g_assert(pipeline_);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, nullptr);

    if (!gst_element_link(launchpipe, sink_)) {
      RCLCPP_FATAL(node_->get_logger(), "Cannot link launchpipe -> sink");
      return false;
    }
  }

  // Calibration between rclcpp::Time and gst timestamps
  GstClock * clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  time_offset_ = node_->now().nanoseconds() - ct;
  RCLCPP_INFO(node_->get_logger(), "Time offset: %ld", time_offset_);

  gst_element_set_state(pipeline_, GST_STATE_PAUSED);

  if (gst_element_get_state(pipeline_, nullptr, nullptr, -1) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(node_->get_logger(), "Failed to pause stream, check gscam_config");
    return false;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Stream is paused");
  }

  cinfo_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
  if (cxt_.image_encoding_ == "jpeg") {
    jpeg_pub_ =
      node_->create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 1);
  } else {
    camera_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("image_raw", 1);
  }

  // Pre-roll camera if needed
  if (cxt_.preroll_) {
    // The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    // I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (gst_element_get_state(pipeline_, nullptr, nullptr, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to play in preroll");
      return false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Stream is playing in preroll");
    }

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    if (gst_element_get_state(pipeline_, nullptr, nullptr, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(node_->get_logger(), "failed to pause in preroll");
      return false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Stream is paused in preroll");
    }
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(node_->get_logger(), "Could not start stream!");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Pipeline running");
  return true;
}

void GSCamNode::impl::delete_pipeline()
{
  // Stop a running stream, or cleanup a stream that failed to fully start
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    RCLCPP_INFO(node_->get_logger(), "Pipeline deleted");
  }
}

unsigned int bytes_per_pixel(const std::string & encoding)
{
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    return 3;
  } else if (encoding == sensor_msgs::image_encodings::MONO8) {
    return 1;
  } else {
    // sensor_msgs::image_encodings::YUV422_YUY2
    return 2;
  }
}

void GSCamNode::impl::process_frame()
{
  // This should block until a new frame is awake, this way, we'll run at the
  // actual capture framerate of the device.
  // TODO use timeout to handle the case where there's no data
  // RCLCPP_DEBUG(get_logger(), "Getting data...");
  GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
  if (!sample) {
    RCLCPP_ERROR(node_->get_logger(), "Could not get sample, pause for 1s");
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s);
    return;
  }
  GstBuffer * buf = gst_sample_get_buffer(sample);
  GstMemory * memory = gst_buffer_get_memory(buf, 0);
  GstMapInfo info;

  gst_memory_map(memory, &info, GST_MAP_READ);
  gsize & buf_size = info.size;
  guint8 * & buf_data = info.data;
  GstClockTime bt = gst_element_get_base_time(pipeline_);
  // RCLCPP_INFO(get_logger(), "New buffer: timestamp %.6f %lu %lu %.3f",
  //         GST_TIME_AS_USECONDS(buf->timestamp+bt)/1e6+time_offset_, buf->timestamp, bt, time_offset_);

  // Stop on end of stream
  if (!buf) {
    RCLCPP_INFO(node_->get_logger(), "Stream ended, pause for 1s");
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s);
    return;
  }

  // RCLCPP_DEBUG(get_logger(), "Got data.");

  // Get the image width and height
  GstPad * pad = gst_element_get_static_pad(sink_, "sink");
  const GstCaps * caps = gst_pad_get_current_caps(pad);
  GstStructure * structure = gst_caps_get_structure(caps, 0);
  gst_structure_get_int(structure, "width", &width_);
  gst_structure_get_int(structure, "height", &height_);

  // Update header information
  camera_info_manager::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
  auto cinfo = std::make_unique<sensor_msgs::msg::CameraInfo>(cur_cinfo);
  if (cxt_.use_gst_timestamps_) {
    cinfo->header.stamp = rclcpp::Time(static_cast<int64_t>(buf->pts + bt + time_offset_));
  } else {
    cinfo->header.stamp = node_->now();
  }
  // RCLCPP_INFO(get_logger(), "Image time stamp: %.3f",cinfo->header.stamp.toSec());
  cinfo->header.frame_id = cxt_.frame_id_;
  if (cxt_.image_encoding_ == "jpeg") {
    auto img = std::make_unique<sensor_msgs::msg::CompressedImage>();
    img->header = cinfo->header;
    img->format = "jpeg";
    img->data.resize(buf_size);
    std::copy(buf_data, (buf_data) + (buf_size), img->data.begin());
    jpeg_pub_->publish(std::move(img));
    cinfo_pub_->publish(std::move(cinfo));
  } else {
    // Complain if the returned buffer is smaller than we expect
    const unsigned int expected_frame_size = width_ * height_ *
      bytes_per_pixel(cxt_.image_encoding_);

    if (buf_size < expected_frame_size) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Image buffer underflow: expected frame to be %d bytes but got only %lu"
        " bytes (make sure frames are correctly encoded)", expected_frame_size, (buf_size));
    }

    // Construct Image message
    auto img = std::make_unique<sensor_msgs::msg::Image>();

    img->header = cinfo->header;

    // Image data and metadata
    img->width = width_;
    img->height = height_;
    img->encoding = cxt_.image_encoding_;
    img->is_bigendian = false;
    img->data.resize(expected_frame_size);

    // Copy the image, so we can free the buffer allocated by gstreamer
    img->step = width_ * bytes_per_pixel(cxt_.image_encoding_);
    std::copy(
      buf_data,
      (buf_data) + (buf_size),
      img->data.begin());

#undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
    static int count = 0;
    RCLCPP_INFO(
      node_->get_logger(), "%d, %p", count++,
      reinterpret_cast<std::uintptr_t>(img.get()));
#endif

    // Publish the image/info
    camera_pub_->publish(std::move(img));
    cinfo_pub_->publish(std::move(cinfo));
  }

  // Release the buffer
  gst_memory_unmap(memory, &info);
  gst_memory_unref(memory);
  gst_sample_unref(sample);
}

void GSCamNode::impl::restart()
{
  if (pipeline_) {
    // Stop thread
    stop_signal_ = true;
    pipeline_thread_.join();

    // Delete pipeline
    delete_pipeline();
  }

  // If gscam_config is empty look for GSCAM_CONFIG in the environment
  if (cxt_.gscam_config_.empty()) {
    auto gsconfig_env = getenv("GSCAM_CONFIG");
    if (gsconfig_env) {
      RCLCPP_INFO(node_->get_logger(), "Using GSCAM_CONFIG env var: %s", gsconfig_env);
      cxt_.gscam_config_ = gsconfig_env;
    } else {
      RCLCPP_FATAL(
        node_->get_logger(),
        "GSCAM_CONFIG env var and gscam_config param are both missing, can't start stream");
      return;
    }
  }

  if (cxt_.image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
    cxt_.image_encoding_ != sensor_msgs::image_encodings::MONO8 &&
    cxt_.image_encoding_ != sensor_msgs::image_encodings::YUV422_YUY2 &&
    cxt_.image_encoding_ != "jpeg")
  {
    RCLCPP_FATAL(
      node_->get_logger(), "Unsupported image encoding: %s",
      cxt_.image_encoding_.c_str());
    return;
  }

  camera_info_manager_.setCameraName(cxt_.camera_name_);

  if (camera_info_manager_.validateURL(cxt_.camera_info_url_)) {
    camera_info_manager_.loadCameraInfo(cxt_.camera_info_url_);
    RCLCPP_INFO(
      node_->get_logger(), "Loaded camera calibration from %s", cxt_.camera_info_url_.c_str());
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Camera info url '%s' is not valid, missing 'file://' prefix?",
      cxt_.camera_info_url_.c_str());
  }

  // [Re-]start the pipeline in its own thread
  if (create_pipeline()) {
    pipeline_thread_ = std::thread(
      [this]()
      {
        RCLCPP_INFO(node_->get_logger(), "Thread running");    // NOLINT

        while (!stop_signal_ && rclcpp::ok()) {
          process_frame();
        }

        stop_signal_ = false;
        RCLCPP_INFO(node_->get_logger(), "Thread stopped");    // NOLINT
      });
  } else {
    delete_pipeline();
  }
}

//=============================================================================
// GSCamNode
//=============================================================================

GSCamNode::GSCamNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("gscam_publisher", options),
  pImpl_(std::make_unique<GSCamNode::impl>(this))
{
  RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d", options.use_intra_process_comms());

  // Declare and get parameters, this will call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), pImpl_->cxt_, n, t, d)
  CXT_MACRO_INIT_PARAMETERS(GSCAM_ALL_PARAMS, validate_parameters)

  // Register parameters, if they change validate_parameters() will be called again
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
  CXT_MACRO_REGISTER_PARAMETERS_CHANGED(
    (*this), pImpl_->cxt_, GSCAM_ALL_PARAMS,
    validate_parameters)                                                                                // NOLINT
}

GSCamNode::~GSCamNode()
{
  pImpl_.reset();
}

void GSCamNode::validate_parameters()
{
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER( \
    RCLCPP_INFO, \
    get_logger(), pImpl_->cxt_, n, t, d)
  GSCAM_ALL_PARAMS

  pImpl_->restart();
}

} // namespace gscam2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gscam2::GSCamNode)  // NOLINT

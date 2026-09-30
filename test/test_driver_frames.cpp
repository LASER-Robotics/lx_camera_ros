#include "lx_camera/lx_camera.h"
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <thread>

namespace {
// A deterministic SDK fixture: these tests never load or open a physical camera.
int alignment = 0;
int coordinate = 0;
bool invalid_color = false;
bool imu_enabled = false;
uint16_t depth_pixels[] = {1000, 1000};
uint8_t rgb_pixels[] = {10, 20, 30, 40, 50, 60};
float xyz_pixels[6];
float distortion[5] = {};
float extrinsics[] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 50, 0, 0};
LxIntrinsicParameters intrinsics{};
FrameInfo frame{};

DcLib FakeSdk() {
  DcLib sdk{};
  sdk.DcGetApiVersion = []() { return "test-sdk"; };
  sdk.DcSetInfoOutput = [](int, bool, const char *) { return LX_SUCCESS; };
  sdk.DcSetGpuEnable = [](bool) { return LX_SUCCESS; };
  sdk.DcSetJpegDecodeMethod = [](int) { return LX_SUCCESS; };
  sdk.DcGetDeviceList = [](LxDeviceInfo **devices, int *count) {
    static LxDeviceInfo device{};
    *devices = &device;
    *count = 1;
    return LX_SUCCESS;
  };
  sdk.DcOpenDevice = [](LX_OPEN_MODE, const char *, DcHandle *handle, LxDeviceInfo *info) {
    *handle = 1;
    *info = LxDeviceInfo{};
    return LX_SUCCESS;
  };
  sdk.DcCloseDevice = [](DcHandle) { return LX_SUCCESS; };
  sdk.DcStartStream = [](DcHandle) { return LX_SUCCESS; };
  sdk.DcStopStream = [](DcHandle) { return LX_SUCCESS; };
  sdk.DcSetBoolValue = [](DcHandle, int, bool) { return LX_SUCCESS; };
  sdk.DcSetIntValue = [](DcHandle, int, int) { return LX_SUCCESS; };
  sdk.DcGetBoolValue = [](DcHandle, int feature, bool *value) {
    *value = feature == LX_BOOL_ENABLE_IMU ? imu_enabled : true;
    return LX_SUCCESS;
  };
  sdk.DcGetIntValue = [](DcHandle, int feature, LxIntValueInfo *value) {
    *value = LxIntValueInfo{};
    switch (feature) {
      case LX_INT_RGBD_ALIGN_MODE: value->cur_value = alignment; break;
      case LX_INT_XYZ_COORDINATE: value->cur_value = coordinate; break;
      case LX_INT_XYZ_UNIT: value->cur_value = 0; break;
      case LX_INT_3D_IMAGE_WIDTH:
      case LX_INT_2D_IMAGE_WIDTH: value->cur_value = 2; break;
      case LX_INT_3D_IMAGE_HEIGHT:
      case LX_INT_2D_IMAGE_HEIGHT: value->cur_value = 1; break;
      case LX_INT_2D_IMAGE_CHANNEL: value->cur_value = 3; break;
      default: break;
    }
    return LX_SUCCESS;
  };
  sdk.DcGetFloatValue = [](DcHandle, int, LxFloatValueInfo *value) {
    *value = LxFloatValueInfo{};
    return LX_SUCCESS;
  };
  sdk.DcGetPtrValue = [](DcHandle, int feature, void **value) {
    switch (feature) {
      case LX_PTR_3D_EXTRIC_PARAM:
        *value = invalid_color ? nullptr : extrinsics;
        return LX_SUCCESS;
      case LX_PTR_2D_INTRINSIC_PARAMETERS:
      case LX_PTR_3D_INTRINSIC_PARAMETERS:
        intrinsics.width = 2;
        intrinsics.height = 1;
        intrinsics.intrinsics[0] = feature == LX_PTR_2D_INTRINSIC_PARAMETERS ? 200 : 100;
        intrinsics.intrinsics[4] = intrinsics.intrinsics[0];
        intrinsics.intrinsics[8] = 1;
        intrinsics.distortion_model = LX_DISTORTION_RADTAN_5;
        intrinsics.distortion_coeffs = distortion;
        intrinsics.num_distortion_coeffs = 5;
        *value = &intrinsics;
        return LX_SUCCESS;
      case LX_PTR_FRAME_DATA: *value = &frame; return LX_SUCCESS;
      case LX_PTR_XYZ_DATA: *value = xyz_pixels; return LX_SUCCESS;
      default: *value = nullptr; return LX_ERROR;
    }
  };
  sdk.DcSetCmd = [](DcHandle, int) { return LX_SUCCESS; };
  sdk.DcGetErrorString = [](LX_STATE) { return "test-sdk status"; };
  sdk.DcRegisterImuDataCallback = [](DcHandle, LX_IMUDATA_CALLBACK, void *) { return LX_SUCCESS; };
  sdk.DcUnregisterImuDataCallback = [](DcHandle) { return LX_SUCCESS; };
  return sdk;
}

class DriverFrames : public ::testing::Test {
protected:
  void SetUp() override {
    alignment = coordinate = 0;
    invalid_color = imu_enabled = false;
    rclcpp::init(0, nullptr);
    sdk_ = FakeSdk();
    listener_ = std::make_shared<rclcpp::Node>("frame_test_listener");
    frame = FrameInfo{};
    frame.depth_data = {static_cast<LX_DATA_TYPE>(CV_16U), 2, 1, 1, depth_pixels, 1234567, 0};
    frame.amp_data = {static_cast<LX_DATA_TYPE>(CV_16U), 2, 1, 1, depth_pixels, 1234568, 0};
    frame.rgb_data = {static_cast<LX_DATA_TYPE>(CV_8U), 2, 1, 3, rgb_pixels, 1234569, 0};
  }

  void TearDown() override {
    rclcpp::shutdown();
    if (stream_.joinable()) stream_.join();
    camera_.reset();
    listener_.reset();
  }

  void Open(std::vector<rclcpp::Parameter> parameters = {}) {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/uav_test", "-r", "__node:=s11"});
    options.parameter_overrides(parameters);
    camera_ = std::make_shared<LxCamera>(&sdk_, options);
  }

  bool Wait(const std::function<bool()> &ready, double seconds = 3.0) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(listener_);
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  DcLib sdk_{};
  std::shared_ptr<LxCamera> camera_;
  rclcpp::Node::SharedPtr listener_;
  std::thread stream_;
};

TEST_F(DriverFrames, LateSubscriberReceivesStaticTreeWithoutMountOrDynamicTf) {
  Open();
  tf2_msgs::msg::TFMessage::SharedPtr transforms;
  auto sub = listener_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(1).transient_local().reliable(),
      [&](tf2_msgs::msg::TFMessage::SharedPtr msg) { transforms = msg; });
  ASSERT_TRUE(Wait([&] { return bool(transforms); }));
  ASSERT_EQ(transforms->transforms.size(), 4u);
  for (const auto &tf : transforms->transforms) {
    EXPECT_EQ(tf.child_frame_id.find("uav_test/s11/"), 0u);
    EXPECT_NE(tf.child_frame_id, "uav_test/s11/link");
    if (tf.child_frame_id == "uav_test/s11/color") {
      EXPECT_EQ(tf.header.frame_id, "uav_test/s11/link");
      EXPECT_NEAR(tf.transform.translation.y, 0.05, 1e-6);
    }
  }
  EXPECT_EQ(camera_->count_publishers("/tf"), 0u);
  EXPECT_EQ(camera_->count_publishers("LxCamera_TF"), 0u);
}

TEST_F(DriverFrames, PublishTfFalseDisablesInternalAndMountTransforms) {
  Open({rclcpp::Parameter("publish_tf", false), rclcpp::Parameter("publish_mount_tf", true)});
  EXPECT_EQ(camera_->count_publishers("/tf_static"), 0u);
  EXPECT_EQ(camera_->count_publishers("/tf"), 0u);
}

TEST_F(DriverFrames, InvalidColorCalibrationDoesNotCreateIdentityAndExplicitImuIsUsed) {
  invalid_color = imu_enabled = true;
  Open({rclcpp::Parameter("publish_mount_tf", true), rclcpp::Parameter("x", 0.25),
        rclcpp::Parameter("imu_pose", std::vector<double>{0.01, 0.02, 0.03, 0, 0, 0})});
  tf2_msgs::msg::TFMessage::SharedPtr transforms;
  auto sub = listener_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(1).transient_local().reliable(),
      [&](tf2_msgs::msg::TFMessage::SharedPtr msg) { transforms = msg; });
  ASSERT_TRUE(Wait([&] { return bool(transforms); }));
  ASSERT_EQ(transforms->transforms.size(), 4u);
  bool mount_found = false, imu_found = false;
  for (const auto &tf : transforms->transforms) {
    EXPECT_NE(tf.child_frame_id, "uav_test/s11/color");
    if (tf.child_frame_id == "uav_test/s11/link") {
      mount_found = true;
      EXPECT_EQ(tf.header.frame_id, "uav_test/fcu");
      EXPECT_DOUBLE_EQ(tf.transform.translation.x, 0.25);
    }
    if (tf.child_frame_id == "uav_test/s11/imu") {
      imu_found = true;
      EXPECT_DOUBLE_EQ(tf.transform.translation.z, 0.03);
    }
  }
  EXPECT_TRUE(mount_found);
  EXPECT_TRUE(imu_found);
}

class AlignedFrames : public DriverFrames, public ::testing::WithParamInterface<std::pair<int, int>> {};

TEST_P(AlignedFrames, ImagesInfoAndCloudUseEffectiveSdkFramesAndTimestamps) {
  alignment = GetParam().first;
  coordinate = GetParam().second;
  for (int i = 0; i < 2; ++i) {
    xyz_pixels[3 * i] = coordinate == 1 ? 1000 : 0;
    xyz_pixels[3 * i + 1] = 0;
    xyz_pixels[3 * i + 2] = coordinate == 0 ? 1000 : 0;
  }
  Open();
  sensor_msgs::msg::Image::SharedPtr depth, color, amp;
  sensor_msgs::msg::CameraInfo::SharedPtr depth_info, color_info, amp_info;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  auto depth_sub = listener_->create_subscription<sensor_msgs::msg::Image>(
      "/uav_test/LxCamera_Depth", 1, [&](sensor_msgs::msg::Image::SharedPtr msg) { depth = msg; });
  auto color_sub = listener_->create_subscription<sensor_msgs::msg::Image>(
      "/uav_test/LxCamera_Rgb", 1, [&](sensor_msgs::msg::Image::SharedPtr msg) { color = msg; });
  auto amp_sub = listener_->create_subscription<sensor_msgs::msg::Image>(
      "/uav_test/LxCamera_Amp", 1, [&](sensor_msgs::msg::Image::SharedPtr msg) { amp = msg; });
  auto depth_info_sub = listener_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/uav_test/LxCamera_TofInfo", 1, [&](sensor_msgs::msg::CameraInfo::SharedPtr msg) { depth_info = msg; });
  auto color_info_sub = listener_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/uav_test/LxCamera_RgbInfo", 1, [&](sensor_msgs::msg::CameraInfo::SharedPtr msg) { color_info = msg; });
  auto amp_info_sub = listener_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/uav_test/LxCamera_AmpInfo", 1, [&](sensor_msgs::msg::CameraInfo::SharedPtr msg) { amp_info = msg; });
  auto cloud_sub = listener_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/uav_test/LxCamera_Cloud", 1, [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud = msg; });
  stream_ = std::thread([&] { camera_->Run(); });
  ASSERT_TRUE(Wait([&] { return depth && color && amp && depth_info && color_info && amp_info && cloud; }));
  const bool depth_in_color = alignment == 1 || alignment == 3;
  const std::string depth_origin = depth_in_color ? "color" : "depth";
  const std::string color_origin = alignment == 2 ? "depth" : "color";
  EXPECT_EQ(depth->header.frame_id, "uav_test/s11/" + depth_origin + "_optical");
  EXPECT_EQ(color->header.frame_id, "uav_test/s11/" + color_origin + "_optical");
  EXPECT_EQ(depth_info->header, depth->header);
  EXPECT_EQ(color_info->header, color->header);
  EXPECT_EQ(amp_info->header, amp->header);
  EXPECT_EQ(depth->header.stamp.sec, 1);
  EXPECT_EQ(depth->header.stamp.nanosec, 234567000u);
  EXPECT_EQ(cloud->header.frame_id, "uav_test/s11/" + depth_origin + (coordinate == 0 ? "_optical" : ""));
  sensor_msgs::PointCloud2ConstIterator<float> distance(*cloud, coordinate == 0 ? "z" : "x");
  EXPECT_FLOAT_EQ(*distance, 1.0f);  // SDK millimeters converted to ROS meters.
  EXPECT_DOUBLE_EQ(depth_info->k[0], depth_in_color ? 200 : 100);
  EXPECT_DOUBLE_EQ(color_info->k[0], alignment == 2 ? 100 : 200);
  EXPECT_DOUBLE_EQ(depth_info->r[0], 1.0);
  EXPECT_DOUBLE_EQ(depth_info->p[3], 0.0);
  EXPECT_DOUBLE_EQ(depth_info->p[0], depth_info->k[0]);
}

INSTANTIATE_TEST_SUITE_P(AlignmentAndAxes, AlignedFrames,
    ::testing::Values(std::make_pair(0, 0), std::make_pair(0, 1),
                      std::make_pair(1, 0), std::make_pair(1, 1),
                      std::make_pair(2, 0), std::make_pair(2, 1),
                      std::make_pair(3, 0), std::make_pair(3, 1)));
}  // namespace

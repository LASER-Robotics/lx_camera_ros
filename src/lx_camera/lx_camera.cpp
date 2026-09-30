#include "lx_camera/lx_camera.h"
#include "lx_camera.h"
#include "rclcpp/callback_group.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "utils/json.hpp"
#include <cv_bridge/cv_bridge.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <omp.h>
#include <algorithm>
#include <array>

static DcLib *LX_DYNAMIC_LIB = nullptr;



#define SET_INT_PARAM(cmd){                             \
int value = -1;                                         \
this->declare_parameter<int>(#cmd, -1);                 \
this->get_parameter<int>(#cmd, value);                  \
if (value >= 0){                                        \
RCLCPP_INFO(this->get_logger(), "%s: %d", #cmd,value);  \
if(cmd>1000&&cmd<2000)                                  \
Check(#cmd, DcSetIntValue(handle_, cmd, value));        \
if(cmd>3000&&cmd<4000)                                  \
Check(#cmd, DcSetBoolValue(handle_, cmd, value));       \
}}



struct CameraCalibMatrices {
  std::array<double, 9> K{};
  std::array<double, 14> D{};
  LX_DISTORTION_MODEL distortion_model;
};

static bool GetCameraCalibration(DcHandle handle, int type,
    CameraCalibMatrices& out) {
    LxIntrinsicParameters* calib = nullptr;
    if (LX_SUCCESS != DcGetPtrValue(handle,
        0 == type ? LX_PTR_2D_INTRINSIC_PARAMETERS : LX_PTR_3D_INTRINSIC_PARAMETERS,
        (void**)&calib) || !calib || calib->num_distortion_coeffs > out.D.size() ||
        (calib->num_distortion_coeffs && !calib->distortion_coeffs))
        return false;

    for (size_t i = 0; i < 9; ++i) {
        out.K[i] = static_cast<double>(calib->intrinsics[i]);
    }
    out.distortion_model = calib->distortion_model;
    out.D.fill(0.0);
    for (size_t i = 0; i < calib->num_distortion_coeffs; ++i) {
        out.D[i] = static_cast<double>(calib->distortion_coeffs[i]);
    }
    return true;
}

void LxCamera::ImuDataCallback(LxImuData *data_ptr, void *usr_data) {
  if (!data_ptr || !usr_data) return;
  auto *camera = static_cast<LxCamera *>(usr_data);
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu);
  msg->header.frame_id = camera->frames_->imu;
  msg->orientation_covariance[0] = -1.0;
  int64_t nanoseconds = static_cast<int64_t>(data_ptr->imu_data.sensor_timestamp * 1e3);
  msg->header.stamp.sec = nanoseconds / 1e9;
  msg->header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
  msg->linear_acceleration.x = data_ptr->imu_data.acc_x;
  msg->linear_acceleration.y = data_ptr->imu_data.acc_y;
  msg->linear_acceleration.z = data_ptr->imu_data.acc_z;
  msg->angular_velocity.x = data_ptr->imu_data.gry_x;
  msg->angular_velocity.y = data_ptr->imu_data.gry_y;
  msg->angular_velocity.z = data_ptr->imu_data.gry_z;
  camera->pub_imu_->publish(*msg);
}

LxCamera::LxCamera(DcLib *dynamic_lib, const rclcpp::NodeOptions &options)
    : Node("lx_camera_node", options) {
  RCLCPP_INFO(this->get_logger(), "lx_camera_node start!");
  LX_DYNAMIC_LIB = dynamic_lib;
  qos_ = rmw_qos_profile_default;
  ConfigureFrames();

  auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  pub_rgb_ = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Rgb", 1);
  pub_rgb_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "LxCamera_RgbInfo", 1);
  pub_amp_ = this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Amp", 1);
  pub_amp_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("LxCamera_AmpInfo", 1);
  pub_depth_ =
      this->create_publisher<sensor_msgs::msg::Image>("LxCamera_Depth", 1);
  pub_tof_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "LxCamera_TofInfo", 1);
  pub_error_ =
      this->create_publisher<std_msgs::msg::String>("LxCamera_Error", 10);

  pub_pallet_ =
      this->create_publisher<lx_camera_ros::msg::Pallet>("LxCamera_Pallet", 1);
  pub_temper_ = this->create_publisher<lx_camera_ros::msg::FrameRate>(
      "LxCamera_FrameRate", 1);
  pub_obstacle_ = this->create_publisher<lx_camera_ros::msg::Obstacle>(
      "LxCamera_Obstacle", 1);
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("LxCamera_Imu", 20);
  pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "LxCamera_Cloud", 10);
  //pub_lidarCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
  //  "LxCamera_LidarCloud", 10);


  auto cmd = this->create_service<lx_camera_ros::srv::LxCmd>(
      "LxCamera_LxCmd", std::bind(&LxCamera::LxCmd, this, std::placeholders::_1,
                                  std::placeholders::_2));
  auto lxi = this->create_service<lx_camera_ros::srv::LxInt>(
      "LxCamera_LxInt", std::bind(&LxCamera::LxInt, this, std::placeholders::_1,
                                  std::placeholders::_2));
  auto lxb = this->create_service<lx_camera_ros::srv::LxBool>(
      "LxCamera_LxBool",
      std::bind(&LxCamera::LxBool, this, std::placeholders::_1,
                std::placeholders::_2));
  auto lxf = this->create_service<lx_camera_ros::srv::LxFloat>(
      "LxCamera_LxFloat",
      std::bind(&LxCamera::LxFloat, this, std::placeholders::_1,
                std::placeholders::_2));
  auto lxs = this->create_service<lx_camera_ros::srv::LxString>(
      "LxCamera_LxString",
      std::bind(&LxCamera::LxString, this, std::placeholders::_1,
                std::placeholders::_2));

  services_ = {cmd, lxi, lxb, lxf, lxs};

  // set sdk log
  RCLCPP_INFO(this->get_logger(), "Api version: %s", DcGetApiVersion());
  this->declare_parameter<int>("log_level", 1);
  this->declare_parameter<std::string>("log_path", "./");
  int log_level_ = 1;
  std::string log_path_ = "./";
  this->get_parameter<int>("log_level", log_level_);
  this->get_parameter<std::string>("log_path", log_path_);
  RCLCPP_INFO(this->get_logger(), "Log level: %d, file path: %s", log_level_, log_path_.c_str());
  DcSetInfoOutput(log_level_, true, log_path_.c_str());

  int enable_gpu = 0;
  this->declare_parameter<int>("enable_gpu", 1);
  this->get_parameter<int>("enable_gpu", enable_gpu);
  DcSetGpuEnable(enable_gpu);

  int jpeg_decode = 0;
  this->declare_parameter<int>("jpeg_decode", 0);
  this->get_parameter<int>("jpeg_decode", jpeg_decode);
  DcSetJpegDecodeMethod(jpeg_decode);

  this->declare_parameter<std::string>("ip", "0");
  this->get_parameter<std::string>("ip", ip_);
  RCLCPP_INFO(this->get_logger(), "ip: %s", ip_.c_str());
  if (!SearchAndOpenDevice()) {
      return;
  }

  this->declare_parameter<int>("is_depth", 1);
  this->declare_parameter<int>("is_xyz", 1);
  this->get_parameter<int>("is_depth", is_depth_);
  this->get_parameter<int>("is_xyz", is_xyz_);
  RCLCPP_INFO(this->get_logger(), "publish xyz: %d", is_xyz_);
  RCLCPP_INFO(this->get_logger(), "publish depth: %d", is_depth_);
  Check("LX_BOOL_ENABLE_3D_DEPTH_STREAM",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_DEPTH_STREAM,
                        is_xyz_ || is_depth_));

  SET_INT_PARAM(LX_BOOL_ENABLE_3D_AMP_STREAM);
  SET_INT_PARAM(LX_BOOL_ENABLE_2D_STREAM);
  SET_INT_PARAM(LX_BOOL_ENABLE_IMU);

  SET_INT_PARAM(LX_INT_IMU_ACCELERATION_LEVEL);
  SET_INT_PARAM(LX_INT_IMU_ANGULAR_RANGE_LEVEL);
  SET_INT_PARAM(LX_INT_XYZ_UNIT);
  SET_INT_PARAM(LX_INT_XYZ_COORDINATE);

  SET_INT_PARAM(LX_INT_RGBD_ALIGN_MODE);
  SET_INT_PARAM(LX_INT_ALGORITHM_MODE);
  SET_INT_PARAM(LX_INT_WORK_MODE);
  SET_INT_PARAM(LX_INT_3D_FPS);

  SET_INT_PARAM(LX_BOOL_ENABLE_2D_UNDISTORT);
  SET_INT_PARAM(LX_INT_2D_UNDISTORT_SCALE);
  SET_INT_PARAM(LX_INT_2D_BINNING_MODE);

  SET_INT_PARAM(LX_BOOL_ENABLE_3D_UNDISTORT);
  SET_INT_PARAM(LX_INT_3D_UNDISTORT_SCALE);
  SET_INT_PARAM(LX_INT_3D_BINNING_MODE);

  SET_INT_PARAM(LX_BOOL_ENABLE_MULTI_MACHINE);
  SET_INT_PARAM(LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR);
  SET_INT_PARAM(LX_INT_FIRST_EXPOSURE);
  SET_INT_PARAM(LX_INT_SECOND_EXPOSURE);
  SET_INT_PARAM(LX_INT_MIN_DEPTH);
  SET_INT_PARAM(LX_INT_MAX_DEPTH);

  this->declare_parameter<float>("x", 0.0);
  this->declare_parameter<float>("y", 0.0);
  this->declare_parameter<float>("z", 0.0);
  this->declare_parameter<float>("yaw", 0.0);
  this->declare_parameter<float>("roll", 0.0);
  this->declare_parameter<float>("pitch", 0.0);
  this->get_parameter<float>("x", install_x_);
  this->get_parameter<float>("y", install_y_);
  this->get_parameter<float>("z", install_z_);
  this->get_parameter<float>("yaw", install_yaw_);
  this->get_parameter<float>("pitch", install_pitch_);
  this->get_parameter<float>("roll", install_roll_);
  RCLCPP_INFO(this->get_logger(), "x: %f", install_x_);
  RCLCPP_INFO(this->get_logger(), "y: %f", install_y_);
  RCLCPP_INFO(this->get_logger(), "z: %f", install_z_);
  RCLCPP_INFO(this->get_logger(), "yaw: %f", install_yaw_);
  RCLCPP_INFO(this->get_logger(), "pitch: %f", install_pitch_);
  RCLCPP_INFO(this->get_logger(), "roll: %f", install_roll_);

  DcRegisterImuDataCallback(handle_, ImuDataCallback, this);
  if (!is_start_) {
    Start();
  }

}

LxCamera::~LxCamera() {
  if (device_open_) {
    DcUnregisterImuDataCallback(handle_);
    DcStopStream(handle_);
    DcCloseDevice(handle_);
  }
}

static void SetCameraCalibration(sensor_msgs::msg::CameraInfo &info,
                                 const CameraCalibMatrices &calib) {
  info.k = calib.K;
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {calib.K[0], calib.K[1], calib.K[2], 0.0,
            calib.K[3], calib.K[4], calib.K[5], 0.0,
            calib.K[6], calib.K[7], calib.K[8], 0.0};
  // Keep the SDK model identifier until its coefficients can be mapped exactly.
  switch (calib.distortion_model) {
    case LX_DISTORTION_RADTAN_5:
      info.distortion_model = "plumb_bob";
      info.d.assign(calib.D.begin(), calib.D.begin() + 5);
      break;
    case LX_DISTORTION_FISHEYE:
      info.distortion_model = "equidistant";
      info.d.assign(calib.D.begin(), calib.D.begin() + 4);
      break;
    case LX_DISTORTION_RADTAN_14:
      info.distortion_model = "LX_DISTORTION_RADTAN_14";
      info.d.assign(calib.D.begin(), calib.D.end());
      break;
    case LX_DISTORTION_SCARAMUZZA:
      info.distortion_model = "LX_DISTORTION_SCARAMUZZA";
      info.d.assign(calib.D.begin(), calib.D.end());
      break;
    default:
      info.distortion_model = "LX_DISTORTION_UNDEFINE";
      info.d.clear();
  }
}

int LxCamera::Start() {
  if (is_start_) return LX_SUCCESS;
  auto read_int = [this](int feature, int &value) {
    LxIntValueInfo info{};
    const auto result = DcGetIntValue(handle_, feature, &info);
    if (result != LX_SUCCESS) return false;
    value = info.cur_value;
    return true;
  };
  bool enabled = false;
  if (DcGetBoolValue(handle_, LX_BOOL_ENABLE_3D_DEPTH_STREAM, &enabled) == LX_SUCCESS)
    is_depth_ = enabled;
  if (DcGetBoolValue(handle_, LX_BOOL_ENABLE_3D_AMP_STREAM, &enabled) == LX_SUCCESS)
    is_amp_ = enabled;
  if (DcGetBoolValue(handle_, LX_BOOL_ENABLE_2D_STREAM, &enabled) == LX_SUCCESS)
    is_rgb_ = enabled;
  read_int(LX_INT_ALGORITHM_MODE, inside_app_);

  // Use the effective device settings, including values retained in firmware.
  if (!read_int(LX_INT_RGBD_ALIGN_MODE, lx_rgbd_align) ||
      lx_rgbd_align < 0 || lx_rgbd_align > 3) {
    RCLCPP_ERROR(get_logger(), "Cannot determine RGB-D alignment; refusing ambiguous frame IDs");
    return LX_ERROR;
  }
  depth_image_frame_ = frames_->DepthImage(lx_rgbd_align);
  color_image_frame_ = frames_->ColorImage(lx_rgbd_align);
  if (is_xyz_) {
    int coordinate = -1, unit = -1;
    if (!read_int(LX_INT_XYZ_COORDINATE, coordinate) ||
        !read_int(LX_INT_XYZ_UNIT, unit) ||
        (coordinate != 0 && coordinate != 1) || (unit != 0 && unit != 1)) {
      RCLCPP_ERROR(get_logger(), "Cannot determine point cloud axes/units; refusing ambiguous geometry");
      return LX_ERROR;
    }
    cloud_frame_ = frames_->Cloud(lx_rgbd_align, coordinate);
    cloud_unit_scale_ = unit == 0 ? 0.001 : 1.0;
  }

  auto read_camera_info = [&](bool color, sensor_msgs::msg::CameraInfo &info) {
    info = sensor_msgs::msg::CameraInfo();
    int width = 0, height = 0;
    read_int(color ? LX_INT_2D_IMAGE_WIDTH : LX_INT_3D_IMAGE_WIDTH, width);
    read_int(color ? LX_INT_2D_IMAGE_HEIGHT : LX_INT_3D_IMAGE_HEIGHT, height);
    info.width = width;
    info.height = height;
    // The SDK provides intrinsics for the configured output resolution.
    // Do not apply ROI/binning a second time in ROS consumers.
    CameraCalibMatrices calib;
    const bool target_color = color ? lx_rgbd_align != 2
                                   : (lx_rgbd_align == 1 || lx_rgbd_align == 3);
    if (GetCameraCalibration(handle_, target_color ? 0 : 1, calib)) {
      SetCameraCalibration(info, calib);
    } else {
      RCLCPP_WARN(get_logger(), "Missing camera intrinsics; publishing uncalibrated CameraInfo");
    }
    info.header.frame_id = color ? color_image_frame_ : depth_image_frame_;
  };
  if (is_depth_ || is_amp_ || is_xyz_) read_camera_info(false, tof_info_);
  if (is_rgb_) {
    read_camera_info(true, rgb_info_);
    read_int(LX_INT_2D_IMAGE_DATA_TYPE, rgb_type_);
    read_int(LX_INT_2D_IMAGE_CHANNEL, rgb_channel_);
  }
  PublishStaticTransforms();
  const auto ret = DcStartStream(handle_);
  is_start_ = (ret == LX_SUCCESS);
  return static_cast<int>(ret);
}

int LxCamera::Stop() {
  auto ret = DcStopStream(handle_);
  if (is_start_ && ret == LX_SUCCESS) {
    is_start_ = false;
  }
  return static_cast<int>(ret);
}

void LxCamera::Run() {
  if (!is_start_) {
    RCLCPP_ERROR(get_logger(), "Camera stream did not start");
    return;
  }
  long frame_count=0;
  rclcpp::Rate rate(20);
  auto node = shared_from_this();
  while (rclcpp::ok()) {
    if (!is_start_) {
      rclcpp::spin_some(node);
      rate.sleep();
      continue;
    }
    FrameInfo *one_frame = nullptr;
    auto sret = DcSetCmd(handle_, LX_CMD_GET_NEW_FRAME);
    if ((LX_SUCCESS != sret) && (LX_E_FRAME_ID_NOT_MATCH != sret) &&
        (LX_E_FRAME_MULTI_MACHINE != sret)) {
      Check("LX_CMD_GET_NEW_FRAME",sret);
      rclcpp::spin_some(node);
      rate.sleep();
      continue;
    }
    if (Check("LX_PTR_FRAME_DATA",
              DcGetPtrValue(handle_, LX_PTR_FRAME_DATA, (void **)&one_frame))) {
      rclcpp::spin_some(node);
      rate.sleep();
      continue;
    }
    if (!one_frame) continue;
    rclcpp::Time now = this->get_clock()->now();
    float dep_fps = 0.0, amp_fps = 0.0, rgb_fps = 0.0, temp = 0.0;
    LxFloatValueInfo f_val;

    if (is_depth_ || is_xyz_) {
      void *dep_data = one_frame->depth_data.frame_data;

      if (dep_data) {
        cv_bridge::CvImage cv_img;
        sensor_msgs::msg::Image msg_depth;

        cv::Mat dep_img(one_frame->depth_data.frame_height,
                        one_frame->depth_data.frame_width,
                        CV_MAKETYPE(one_frame->depth_data.frame_data_type,
                                    one_frame->depth_data.frame_channel),
                        dep_data);

        cv::Mat dist_img;
        if (dep_img.type() == CV_32F) {
          cv::normalize(dep_img, dist_img, 0, 65535, cv::NORM_MINMAX);
          dist_img.convertTo(dist_img, CV_16UC1);
          cv_img.image = dist_img;
        } else {
          cv_img.image = dep_img;
        }

        int64_t nanoseconds =
            static_cast<int64_t>(one_frame->depth_data.sensor_timestamp * 1e3);
        cv_img.header.stamp.sec = nanoseconds / 1e9;
        cv_img.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);

        cv_img.header.frame_id = depth_image_frame_;
        cv_img.encoding = "mono16";
        cv_img.toImageMsg(msg_depth);
        pub_depth_->publish(msg_depth);
        tof_info_.header = msg_depth.header;
        tof_info_.width = msg_depth.width;
        tof_info_.height = msg_depth.height;
        pub_tof_info_->publish(tof_info_);

        if(is_xyz_ == 2){
          void* xyzirt_data = nullptr;
          if (DcGetPtrValue(handle_, LX_PTR_XYZIRT_DATA, (void**)&xyzirt_data) == LX_SUCCESS){
            sensor_msgs::msg::PointCloud2 msg_lidarCloud;
            LxPointCloudData* data = (LxPointCloudData*)xyzirt_data;
            pcl::PointCloud<PointXYZIT>::Ptr lidarCloud(new pcl::PointCloud<PointXYZIT>);
            const uint32_t total_points = data->point_num;
            lidarCloud->points.reserve(total_points);
            for (uint32_t i = 0; i < total_points; ++i) {
              const LxPointXYZIRT& point = data->points[i];
              double lidar_timestamp =
                  static_cast<double>(data->timebase) + static_cast<double>(point.offset_time);
              lidarCloud->points.emplace_back(
                  point.x * cloud_unit_scale_,
                  point.y * cloud_unit_scale_,
                  point.z * cloud_unit_scale_,
                  point.intensity,
                  lidar_timestamp,
                  point.row_pos,
                  point.col_pos
              );
            }
            lidarCloud->width = lidarCloud->points.size();
            lidarCloud->height = 1;
            lidarCloud->is_dense = true;  
            pcl::toROSMsg(*lidarCloud, msg_lidarCloud);
            int64_t ns = static_cast<int64_t>(data->timebase) * 1000;
            msg_lidarCloud.header.stamp.sec = ns / 1000000000LL;
            msg_lidarCloud.header.stamp.nanosec = ns % 1000000000LL;
            msg_lidarCloud.header.frame_id = cloud_frame_;
            pub_cloud_->publish(msg_lidarCloud);
          }

        }
        if (is_xyz_ == 1) {
          float *xyz_data = nullptr;
          if (DcGetPtrValue(handle_, LX_PTR_XYZ_DATA, (void **)&xyz_data) == LX_SUCCESS) {
            sensor_msgs::msg::PointCloud2 msg_cloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            const auto buff_len = tof_info_.width * tof_info_.height;
            cloud->points.reserve(buff_len);
            uint8_t* rgb_data = static_cast<uint8_t*>(one_frame->rgb_data.frame_data);
            for (long i = 0; i < buff_len; i++)
            {
                long index = 3 * i;
                if(xyz_data[index] == 0.0f && xyz_data[index+1] == 0.0f && xyz_data[index+2] == 0.0f) {
                    continue;
                }
                pcl::PointXYZRGB point;
                point.x = xyz_data[index] * cloud_unit_scale_;
                point.y = xyz_data[index + 1] * cloud_unit_scale_;
                point.z = xyz_data[index + 2] * cloud_unit_scale_;

                if (!lx_rgbd_align || rgb_data == nullptr || rgb_channel_ != 3) {
                    point.b = 255;
                    point.g = 255;
                    point.r = 255;
                }
                else {
                    point.b = rgb_data[index];
                    point.g = rgb_data[index + 1];
                    point.r = rgb_data[index + 2];
                }
                cloud->points.emplace_back(point);
            }
            cloud->width = cloud->points.size();
            cloud->height = 1; 
            cloud->is_dense = true; 
            pcl::toROSMsg(*cloud, msg_cloud);
            int64_t nanoseconds = static_cast<int64_t>(
                one_frame->depth_data.sensor_timestamp * 1e3);
            msg_cloud.header.stamp.sec = nanoseconds / 1e9;

            msg_cloud.header.stamp.nanosec =
                nanoseconds % static_cast<int64_t>(1e9);
            msg_cloud.header.frame_id = cloud_frame_;
            pub_cloud_->publish(msg_cloud);
          }

        }
      }

      Check("LX_FLOAT_3D_DEPTH_FPS",
            DcGetFloatValue(handle_, LX_FLOAT_3D_DEPTH_FPS, &f_val));
      dep_fps = f_val.cur_value;
    }

    if (is_amp_) {
      void *amp_data = one_frame->amp_data.frame_data;
      if (amp_data) {
        cv_bridge::CvImage cv_img;
        sensor_msgs::msg::Image msg_amp;

        cv::Mat amp_img(one_frame->amp_data.frame_height,
                        one_frame->amp_data.frame_width,
                        CV_MAKETYPE(one_frame->amp_data.frame_data_type,
                                    one_frame->amp_data.frame_channel),
                        amp_data);
        if (amp_img.type() == CV_8UC1) {
          cv_img.encoding = "mono8";
        } else {
          cv_img.encoding = "mono16";
        }

        int64_t nanoseconds =
            static_cast<int64_t>(one_frame->amp_data.sensor_timestamp * 1e3);
        cv_img.header.stamp.sec = nanoseconds / 1e9;
        cv_img.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
        cv_img.header.frame_id = depth_image_frame_;
        cv_img.image = amp_img;
        cv_img.toImageMsg(msg_amp);
        pub_amp_->publish(msg_amp);
        auto amp_info = tof_info_;
        amp_info.header = msg_amp.header;
        amp_info.width = msg_amp.width;
        amp_info.height = msg_amp.height;
        pub_amp_info_->publish(amp_info);
        if (!is_depth_ && !is_xyz_) pub_tof_info_->publish(amp_info);
      }

      Check("LX_FLOAT_3D_AMPLITUDE_FPS",
            DcGetFloatValue(handle_, LX_FLOAT_3D_AMPLITUDE_FPS, &f_val));
      amp_fps = f_val.cur_value;
    }

    if (is_rgb_) {
      void *rgb_data = one_frame->rgb_data.frame_data;
      if (rgb_data) {
        cv::Mat rgb_pub;
        cv_bridge::CvImage cv_img;
        sensor_msgs::msg::Image msg_rgb;
        auto type = CV_MAKETYPE(rgb_type_, rgb_channel_);
        cv::Mat rgb_img(one_frame->rgb_data.frame_height,
                        one_frame->rgb_data.frame_width, type, rgb_data);
        rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type_ == CV_16U ? 0.25 : 1);
        std::string rgb_type_ = rgb_channel_ == 3 ? "bgr8" : "mono8";
        int64_t nanoseconds =
            static_cast<int64_t>(one_frame->rgb_data.sensor_timestamp * 1e3);
        cv_img.header.stamp.sec = nanoseconds / 1e9;
        cv_img.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
        cv_img.header.frame_id = color_image_frame_;
        cv_img.encoding = rgb_type_;
        cv_img.image = rgb_pub;
        cv_img.toImageMsg(msg_rgb);
        pub_rgb_->publish(msg_rgb);
        rgb_info_.header = msg_rgb.header;
        rgb_info_.width = msg_rgb.width;
        rgb_info_.height = msg_rgb.height;
        pub_rgb_info_->publish(rgb_info_);
      }

      Check("LX_FLOAT_2D_IMAGE_FPS",
            DcGetFloatValue(handle_, LX_FLOAT_2D_IMAGE_FPS, &f_val));
      rgb_fps = f_val.cur_value;
    }

    Check("LX_FLOAT_DEVICE_TEMPERATURE",
          DcGetFloatValue(handle_, LX_FLOAT_DEVICE_TEMPERATURE, &f_val));
    temp = f_val.cur_value;
    lx_camera_ros::msg::FrameRate fr;
    fr.header.frame_id = frames_->link;
    fr.header.stamp = now;
    fr.amp = amp_fps;
    fr.rgb = rgb_fps;
    fr.depth = dep_fps;
    fr.temperature = temp;
    pub_temper_->publish(fr);

    int ret = 0;
    void *app_ptr = one_frame->app_data.frame_data;
    switch (inside_app_) {
    case MODE_AVOID_OBSTACLE: {
      lx_camera_ros::msg::Obstacle result;
      result.header.frame_id = frames_->prefix + "/application";
      int64_t nanoseconds =
          static_cast<int64_t>(one_frame->app_data.sensor_timestamp * 1e3);
      result.header.stamp.sec = nanoseconds / 1e9;
      result.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
      Check("GetObstacleIO", DcSpecialControl(handle_, "GetObstacleIO",
                                              (void *)&result.io_output));
      if (ret || !app_ptr) {
        result.status = -1;
        pub_obstacle_->publish(result);
        break;
      }
      LxAvoidanceOutput *lao = (LxAvoidanceOutput *)app_ptr;
      result.status = lao->state;
      result.box_number = lao->number_box;
      for (int i = 0; i < lao->number_box; i++) {
        auto raw_box = lao->obstacleBoxs[i];
        lx_camera_ros::msg::ObstacleBox box;
        box.width = raw_box.width;
        box.depth = raw_box.depth;
        box.height = raw_box.height;
        for (int t = 0; t < 3; t++)
          box.center[t] = raw_box.center[t];
        for (int t = 0; t < 9; t++)
          box.rotation[t] = raw_box.pose.R[t];
        for (int t = 0; t < 3; t++)
          box.translation[t] = raw_box.pose.T[t];
        result.box.push_back(box);
      }
      pub_obstacle_->publish(result);
      break;
    }
    case MODE_PALLET_LOCATE: {
      if (ret || !app_ptr) {
        break;
      }
      lx_camera_ros::msg::Pallet result;
      result.header.frame_id = frames_->prefix + "/application";
      int64_t nanoseconds =
          static_cast<int64_t>(one_frame->app_data.sensor_timestamp * 1e3);
      result.header.stamp.sec = nanoseconds / 1e9;
      result.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
      LxPalletPose *lao = (LxPalletPose *)app_ptr;
      result.status = lao->return_val;
      result.x = lao->x;
      result.y = lao->y;
      result.yaw = lao->yaw;
      pub_pallet_->publish(result);
      break;
    }
    case MODE_VISION_LOCATION: {
      if (ret || !app_ptr) {
        break;
      }
      LxLocation *val = (LxLocation *)app_ptr;
      if (!val->status) {
        geometry_msgs::msg::PoseStamped alg_val;
        int64_t nanoseconds =
            static_cast<int64_t>(one_frame->app_data.sensor_timestamp * 1e3);
        alg_val.header.stamp.sec = nanoseconds / 1e9;
        alg_val.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
        alg_val.header.frame_id = frames_->prefix + "/application";
        auto qua_res = ToQuaternion(val->theta, 0, 0);
        alg_val.pose.position.x = val->x;
        alg_val.pose.position.y = val->y;
        alg_val.pose.position.z = 0;
        alg_val.pose.orientation.x = qua_res.x;
        alg_val.pose.orientation.y = qua_res.y;
        alg_val.pose.orientation.z = qua_res.z;
        alg_val.pose.orientation.w = qua_res.w;
        pub_location_->publish(alg_val);
      }
      break;
    }
    case MODE_AVOID_OBSTACLE2: {
      lx_camera_ros::msg::Obstacle result;
      result.header.frame_id = frames_->prefix + "/application";
      int64_t nanoseconds =
          static_cast<int64_t>(one_frame->app_data.sensor_timestamp * 1e3);
      result.header.stamp.sec = nanoseconds / 1e9;
      result.header.stamp.nanosec = nanoseconds % static_cast<int64_t>(1e9);
      Check("GetObstacleIO", DcSpecialControl(handle_, "GetObstacleIO",
                                              (void *)&result.io_output));
      if (ret || !app_ptr) {
        result.status = -1;
        pub_obstacle_->publish(result);
        break;
      }
      LxAvoidanceOutputN *lao = (LxAvoidanceOutputN *)app_ptr;
      result.status = lao->state;
      result.box_number = lao->number_box;
      for (int i = 0; i < lao->number_box; i++) {
        auto raw_box = lao->obstacleBoxs[i];
        lx_camera_ros::msg::ObstacleBox box;
        box.width = raw_box.width;
        box.depth = raw_box.depth;
        box.height = raw_box.height;
        for (int t = 0; t < 3; t++)
          box.center[t] = raw_box.center[t];
        for (int t = 0; t < 9; t++)
          box.rotation[t] = raw_box.pose.R[t];
        for (int t = 0; t < 3; t++)
          box.translation[t] = raw_box.pose.T[t];
        result.box.push_back(box);
      }
      pub_obstacle_->publish(result);
      break;
    }
    }
    rclcpp::spin_some(node);
    rate.sleep();
  }
}

bool LxCamera::SearchAndOpenDevice() {
  // find device
  int devnum = 0;
  LxDeviceInfo *devlist = nullptr;
  while (true) {
    Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
	if(devnum) break;
    RCLCPP_ERROR(this->get_logger(), "Found device faild. retry...");
    std::this_thread::sleep_for(std::chrono::milliseconds(5 * 1000));
  }

  // open device
  LxDeviceInfo info;
  if (ip_.empty()) {
    ip_ = "0";
  }
  auto mode =
      ip_.size() < 8 ? LX_OPEN_MODE::OPEN_BY_INDEX : LX_OPEN_MODE::OPEN_BY_IP;
  if (LX_SUCCESS != DcOpenDevice(mode, ip_.c_str(), &handle_, &info)) {
    RCLCPP_ERROR(this->get_logger(), "Open device failed!");
    return false;
  }
  device_open_ = true;
  RCLCPP_INFO(this->get_logger(),
              "Open device success:"
              "\ndevice handle:             %lld"
              "\ndevice name:               %s"
              "\ndevice id:                 %s"
              "\ndevice ip:                 %s"
              "\ndevice sn:                 %s"
              "\ndevice mac:                %s"
              "\ndevice firmware version:   %s"
              "\ndevice algorithm version:  %s",
              handle_, info.name, info.id, info.ip, info.sn, info.mac,
              info.firmware_ver, info.algor_ver);
  return true;
}

int LxCamera::Check(std::string command, int state) {
  LX_STATE lx_state = static_cast<LX_STATE>(state);
  if (LX_SUCCESS == lx_state) {
    return lx_state;
  }
  const char *m = DcGetErrorString(lx_state); // 获取错误信息
  std_msgs::msg::String msg;
  setlocale(LC_ALL, "");
  msg.data = "#command: " + command +
             " #error code: " + std::to_string(lx_state) + " #report: " + m;
  pub_error_->publish(msg); // 推送错误信息
  RCLCPP_ERROR(this->get_logger(), "%s", msg.data.c_str());
  return state;
}

bool LxCamera::LxString(
    const lx_camera_ros::srv::LxString::Request::SharedPtr req,
    const lx_camera_ros::srv::LxString::Response::SharedPtr res) {
  if (req->is_set) {
    res->result.ret = DcSetStringValue(handle_, req->cmd, req->val.c_str());
  }
  char *buf = nullptr;
  if (!req->is_set) {
    res->result.ret = DcGetStringValue(handle_, req->cmd, &buf);
  }
  res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
  if (buf) {
    res->val = std::string(buf);
  }
  return true;
}

bool LxCamera::LxFloat(
    const lx_camera_ros::srv::LxFloat::Request::SharedPtr req,
    const lx_camera_ros::srv::LxFloat::Response::SharedPtr res) {
  if (req->is_set) {
    res->result.ret = DcSetFloatValue(handle_, req->cmd, req->val);
  }
  LxFloatValueInfo float_value{0, 0, 0, 0, 0};
  auto ret = DcGetFloatValue(handle_, req->cmd, &float_value);
  if (!req->is_set) {
    res->result.ret = ret;
  }
  res->cur_value = float_value.cur_value;
  res->max_value = float_value.max_value;
  res->min_value = float_value.min_value;
  res->available = float_value.set_available;
  res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
  return true;
}

bool LxCamera::LxBool(
    const lx_camera_ros::srv::LxBool::Request::SharedPtr req,
    const lx_camera_ros::srv::LxBool::Response::SharedPtr res) {
  if (req->is_set) {
    res->result.ret = DcSetBoolValue(handle_, req->cmd, req->val);
  }
  bool val;
  auto ret = DcGetBoolValue(handle_, req->cmd, &val);
  res->val = val;
  if (!req->is_set) {
    res->result.ret = ret;
  }
  res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
  return true;
}

bool LxCamera::LxCmd(const lx_camera_ros::srv::LxCmd::Request::SharedPtr req,
                     const lx_camera_ros::srv::LxCmd::Response::SharedPtr res) {
  auto Pub = [&](std::string msg, int ret) {
    lx_camera_ros::msg::Result result;
    result.ret = ret;
    result.msg = msg;
    res->result.push_back(result);
  };
  if (req->cmd == 1) {
    auto ret = static_cast<LX_STATE>(Start());
    Pub(DcGetErrorString(ret), ret);
  } else if (req->cmd == 2) {
    auto ret = static_cast<LX_STATE>(Stop());
    Pub(DcGetErrorString(ret), ret);
  } else if (req->cmd) {
    auto ret = static_cast<LX_STATE>(DcSetCmd(handle_, req->cmd));
    Pub(DcGetErrorString(ret), ret);
  } else {
    std::vector<std::pair<std::string, int>> cmd_vec;
    auto add = [&](std::string str, int cmd) {
      std::pair<std::string, int> val(str, cmd);
      cmd_vec.push_back(val);
    };
    add("INT      FIRST_EXPOSURE", 1001);
    add("INT      SECOND_EXPOSURE", 1002);
    add("INT      THIRD_EXPOSURE", 1003);
    add("INT      FOURTH_EXPOSURE", 1004);
    add("INT      GAIN", 1005);
    add("INT      MIN_DEPTH", 1011);
    add("INT      MAX_DEPTH", 1012);
    add("INT      MIN_AMPLITUDE", 1013);
    add("INT      MAX_AMPLITUDE", 1014);
    add("INT      CODE_MODE", 1016);
    add("INT      WORK_MODE", 1018);
    add("INT      LINK_SPEED", 1019);
    add("INT      3D_IMAGE_WIDTH", 1021);
    add("INT      3D_IMAGE_HEIGHT", 1022);
    add("INT      3D_IMAGE_OFFSET_X", 1023);
    add("INT      3D_IMAGE_OFFSET_Y", 1024);
    add("INT      3D_BINNING_MODE", 1025);
    add("INT      3D_DEPTH_DATA_TYPE", 1026);
    add("INT      3D_AMPLITUDE_CHANNEL", 1031);
    add("INT      3D_AMPLITUDE_GET_TYPE", 1032);
    add("INT      3D_AMPLITUDE_EXPOSURE", 1033);
    add("INT      3D_AMPLITUDE_INTENSITY", 1034);
    add("INT      3D_AMPLITUDE_DATA_TYPE", 1035);
    add("INT      3D_AUTO_EXPOSURE_LEVEL", 1036);
    add("INT      3D_AUTO_EXPOSURE_MAX", 1037);
    add("INT      3D_AUTO_EXPOSURE_MIN", 1038);
    add("INT      2D_IMAGE_WIDTH", 1041);
    add("INT      2D_IMAGE_HEIGHT", 1042);
    add("INT      2D_IMAGE_OFFSET_X", 1043);
    add("INT      2D_IMAGE_OFFSET_Y", 1044);
    add("INT      2D_BINNING_MODE", 1045);
    add("INT      2D_IMAGE_CHANNEL", 1046);
    add("INT      2D_IMAGE_DATA_TYPE", 1047);
    add("INT      2D_MANUAL_EXPOSURE", 1051);
    add("INT      2D_MANUAL_GAIN", 1052);
    add("INT      2D_ENCODE_TYPE", 1053);
    add("INT      2D_AUTO_EXPOSURE_LEVEL", 1054);
    add("INT      TOF_GLOBAL_OFFSET", 1061);
    add("INT      3D_UNDISTORT_SCALE", 1062);
    add("INT      ALGORITHM_MODE", 1065);
    add("INT      MODBUS_ADDR", 1066);
    add("INT      HEART_TIME", 1067);
    add("INT      GVSP_PACKET_SIZE", 1068);
    add("INT      TRIGGER_MODE", 1069);
    add("INT      CALCULATE_UP", 1070);
    add("INT      CAN_BAUD_RATE", 1072);
    add("INT      CUSTOM_PARAM_GROUP", 1075);
    add("FLOAT    FILTER_LEVEL", 2001);
    add("FLOAT    EST_OUT_EXPOSURE", 2002);
    add("FLOAT    LIGHT_INTENSITY", 2003);
    add("FLOAT    3D_DEPTH_FPS", 2004);
    add("FLOAT    3D_AMPLITUDE_FPS", 2005);
    add("FLOAT    2D_IMAGE_FPS", 2006);
    add("FLOAT    DEVICE_TEMPERATURE", 2007);
    add("BOOL     CONNECT_STATE", 3001);
    add("BOOL     ENABLE_3D_DEPTH_STREAM", 3002);
    add("BOOL     ENABLE_3D_AMP_STREAM", 3003);
    add("BOOL     ENABLE_3D_AUTO_EXPOSURE", 3006);
    add("BOOL     ENABLE_3D_UNDISTORT", 3007);
    add("BOOL     ENABLE_ANTI_FLICKER", 3008);
    add("BOOL     ENABLE_2D_STREAM", 3011);
    add("BOOL     ENABLE_2D_AUTO_EXPOSURE", 3012);
    add("BOOL     ENABLE_2D_UNDISTORT", 3015);
    add("BOOL     ENABLE_2D_TO_DEPTH", 3016);
    add("BOOL     ENABLE_BACKGROUND_AMP", 3017);
    add("BOOL     ENABLE_MULTI_MACHINE", 3018);
    add("BOOL     ENABLE_MULTI_EXPOSURE_HDR", 3019);
    add("BOOL     ENABLE_SYNC_FRAME", 3020);
    add("STRING   DEVICE_VERSION", 4001);
    add("STRING   DEVICE_LOG_NAME", 4002);
    add("STRING   FIRMWARE_NAME", 4003);
    add("STRING   FILTER_PARAMS", 4004);
    add("STRING   ALGORITHM_PARAMS", 4005);
    add("STRING   ALGORITHM_VERSION", 4006);
    add("STRING   DEVICE_OS_VERSION", 4007);
    add("CMD      GET_PARAM_LIST", 0);
    add("CMD      START_STREAM", 1);
    add("CMD      STOP_STREAM", 2);
    add("CMD      GET_NEW_FRAME", 5001);
    add("CMD      RETURN_VERSION", 5002);
    add("CMD      RESTART_DEVICE", 5003);
    add("CMD      WHITE_BALANCE", 5004);
    add("CMD      RESET_PARAM", 5007);
    add("CMD      CALIB_EXTRIC", 5008);
    for (auto &i : cmd_vec)
      while (i.first.length() < 40)
        i.first.push_back(' ');
    for (auto &i : cmd_vec)
      Pub(i.first, i.second);
  }
  return true;
}

bool LxCamera::LxInt(const lx_camera_ros::srv::LxInt::Request::SharedPtr req,
                     const lx_camera_ros::srv::LxInt::Response::SharedPtr res) {
  if (req->is_set) {
    res->result.ret = DcSetIntValue(handle_, req->cmd, req->val);
  }
  LxIntValueInfo int_value{0, 0, 0, 0, 0};
  auto ret = DcGetIntValue(handle_, req->cmd, &int_value);
  if (!req->is_set) {
    res->result.ret = ret;
  }
  res->cur_value = int_value.cur_value;
  res->max_value = int_value.max_value;
  res->min_value = int_value.min_value;
  res->available = int_value.set_available;
  res->result.msg = DcGetErrorString((LX_STATE)res->result.ret);
  return true;
}

void LxCamera::ConfigureFrames() {
  const auto base = declare_parameter<std::string>("base_frame_id", "link");
  frames_.reset(new lx_camera::FrameNames(get_namespace(), get_name(), base));
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  publish_mount_tf_ = declare_parameter<bool>("publish_mount_tf", false);
  const auto ns = lx_camera::NormalizeFrame(get_namespace());
  parent_frame_id_ = lx_camera::NormalizeFrame(declare_parameter<std::string>(
      "parent_frame_id", ns.empty() ? "fcu" : ns + "/fcu"));
  if (publish_mount_tf_ && (parent_frame_id_.empty() ||
      parent_frame_id_ == frames_->prefix ||
      parent_frame_id_.compare(0, frames_->prefix.size() + 1, frames_->prefix + "/") == 0)) {
    throw std::invalid_argument("parent_frame_id must be outside the camera frame tree");
  }
  // Pose of the SDK's native IMU axes in camera link coordinates. No optical
  // rotation is assumed: the installed SDK does not document IMU extrinsic axes.
  imu_pose_ = declare_parameter<std::vector<double>>("imu_pose", std::vector<double>{});
  if (!imu_pose_.empty()) lx_camera::PoseFromXyzRpy(imu_pose_);
}

void LxCamera::PublishStaticTransforms() {
  if (!publish_tf_) return;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  const auto stamp = now();
  auto append = [&](const std::string &parent, const std::string &child,
                    const Eigen::Isometry3d &pose) {
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = parent;
    message.child_frame_id = child;
    message.transform.translation.x = pose.translation().x();
    message.transform.translation.y = pose.translation().y();
    message.transform.translation.z = pose.translation().z();
    const Eigen::Quaterniond rotation(pose.linear());
    const auto normalized = rotation.normalized();
    message.transform.rotation.x = normalized.x();
    message.transform.rotation.y = normalized.y();
    message.transform.rotation.z = normalized.z();
    message.transform.rotation.w = normalized.w();
    transforms.push_back(message);
  };

  if (publish_mount_tf_) {
    append(parent_frame_id_, frames_->link, lx_camera::PoseFromXyzRpy(
        {install_x_, install_y_, install_z_, install_roll_, install_pitch_, install_yaw_}));
  }
  // The camera link origin is the physical depth sensor origin, with body axes.
  // Keep the depth reference even in RGB-only operation.
  append(frames_->link, frames_->depth, Eigen::Isometry3d::Identity());
  append(frames_->depth, frames_->depth_optical, lx_camera::OpticalToBody());
  if (is_rgb_ || lx_rgbd_align == 1 || lx_rgbd_align == 3) {
    float *extrinsics = nullptr;
    const auto result = DcGetPtrValue(handle_, LX_PTR_3D_EXTRIC_PARAM,
                                     reinterpret_cast<void **>(&extrinsics));
    try {
      if (result != LX_SUCCESS) throw std::runtime_error("SDK extrinsics query failed");
      append(frames_->link, frames_->color, lx_camera::ColorPoseInDepth(extrinsics));
      append(frames_->color, frames_->color_optical, lx_camera::OpticalToBody());
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Color TF unavailable: %s. No identity fallback is published.",
                   error.what());
    }
  }

  bool imu_enabled = false;
  if (DcGetBoolValue(handle_, LX_BOOL_ENABLE_IMU, &imu_enabled) == LX_SUCCESS && imu_enabled) {
    if (!imu_pose_.empty()) {
      append(frames_->link, frames_->imu, lx_camera::PoseFromXyzRpy(imu_pose_));
    } else {
      RCLCPP_WARN(get_logger(),
          "IMU messages use %s in native SDK axes. Set calibrated imu_pose or provide external TF; "
          "SDK IMU extrinsic direction is undocumented, so no IMU TF is assumed.", frames_->imu.c_str());
    }
  }
  // Replace the retained sample when streams restart with different settings.
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  static_tf_broadcaster_->sendTransform(transforms);
  RCLCPP_INFO(get_logger(), "Published %zu static transforms under %s",
              transforms.size(), frames_->link.c_str());
}

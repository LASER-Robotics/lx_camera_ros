#ifndef LX_CAMERA_FRAME_GEOMETRY_H_
#define LX_CAMERA_FRAME_GEOMETRY_H_

#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace lx_camera {

// TF poses map child coordinates into parent coordinates.
inline Eigen::Isometry3d OpticalToBody() {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  return pose;
}

inline bool IsRigidTransform(const Eigen::Isometry3d &pose) {
  return pose.matrix().allFinite() &&
         (pose.linear().transpose() * pose.linear()).isApprox(
             Eigen::Matrix3d::Identity(), 1e-3) &&
         std::abs(pose.linear().determinant() - 1.0) < 1e-3;
}

// MRDVS CameraSDK's ROS2 sample reads a row-major depth-to-color rotation,
// followed by translation in millimeters, then inverts it for the TF pose.
inline Eigen::Isometry3d ColorPoseInDepth(const float *extrinsics) {
  if (!extrinsics) {
    throw std::invalid_argument("Missing depth-to-color extrinsics");
  }
  Eigen::Isometry3d color_from_depth = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      color_from_depth.linear()(row, col) = extrinsics[3 * row + col];
    }
    color_from_depth.translation()(row) = extrinsics[9 + row] * 0.001;
  }
  if (!IsRigidTransform(color_from_depth)) {
    throw std::invalid_argument("Invalid depth-to-color calibration matrix");
  }
  return OpticalToBody() * color_from_depth.inverse() * OpticalToBody().inverse();
}

inline Eigen::Isometry3d PoseFromXyzRpy(const std::vector<double> &xyz_rpy) {
  if (xyz_rpy.size() != 6) {
    throw std::invalid_argument("Expected [x, y, z, roll, pitch, yaw]");
  }
  for (double value : xyz_rpy) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Pose values must be finite");
    }
  }
  constexpr double radians_per_degree = 3.14159265358979323846 / 180.0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(xyz_rpy[0], xyz_rpy[1], xyz_rpy[2]);
  pose.linear() = (Eigen::AngleAxisd(xyz_rpy[5] * radians_per_degree, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(xyz_rpy[4] * radians_per_degree, Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(xyz_rpy[3] * radians_per_degree, Eigen::Vector3d::UnitX()))
                      .toRotationMatrix();
  return pose;
}

inline std::string NormalizeFrame(const std::string &name) {
  const auto first = name.find_first_not_of('/');
  if (first == std::string::npos) {
    return "";
  }
  const auto last = name.find_last_not_of('/');
  const auto result = name.substr(first, last - first + 1);
  if (result.find("//") != std::string::npos ||
      result.find_first_of(" \t\r\n") != std::string::npos) {
    throw std::invalid_argument("Invalid TF frame name: " + name);
  }
  return result;
}

struct FrameNames {
  std::string prefix, link, depth, depth_optical, color, color_optical, imu;

  FrameNames(const std::string &ns, const std::string &camera,
             const std::string &base = "link") {
    const auto namespace_name = NormalizeFrame(ns);
    const auto camera_name = NormalizeFrame(camera);
    const auto base_name = NormalizeFrame(base);
    if (camera_name.empty() || base_name.empty() || base_name.find('/') != std::string::npos ||
        base_name == "depth" || base_name == "depth_optical" || base_name == "color" ||
        base_name == "color_optical" || base_name == "imu") {
      throw std::invalid_argument("base_frame_id must be a unique camera-local frame name");
    }
    prefix = (namespace_name.empty() ? "" : namespace_name + "/") + camera_name;
    link = prefix + "/" + base_name;
    depth = prefix + "/depth";
    depth_optical = prefix + "/depth_optical";
    color = prefix + "/color";
    color_optical = prefix + "/color_optical";
    imu = prefix + "/imu";
  }

  std::string DepthImage(int alignment) const {
    ValidateAlignment(alignment);
    return (alignment == 1 || alignment == 3) ? color_optical : depth_optical;
  }

  std::string ColorImage(int alignment) const {
    ValidateAlignment(alignment);
    return alignment == 2 ? depth_optical : color_optical;
  }

  std::string Cloud(int alignment, int coordinate) const {
    ValidateAlignment(alignment);
    if (coordinate != 0 && coordinate != 1) {
      throw std::invalid_argument("Unknown SDK point cloud coordinate system");
    }
    if (coordinate == 0) {
      return DepthImage(alignment);
    }
    return (alignment == 1 || alignment == 3) ? color : depth;
  }

  static void ValidateAlignment(int alignment) {
    if (alignment < 0 || alignment > 3) {
      throw std::invalid_argument("Unknown SDK RGB-D alignment mode");
    }
  }
};

}  // namespace lx_camera

#endif  // LX_CAMERA_FRAME_GEOMETRY_H_

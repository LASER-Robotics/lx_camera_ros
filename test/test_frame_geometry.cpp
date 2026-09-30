#include "lx_camera/frame_geometry.h"
#include <gtest/gtest.h>
#include <limits>

using lx_camera::FrameNames;

TEST(FrameGeometry, OpticalAxesMapToForwardLeftUp) {
  const auto pose = lx_camera::OpticalToBody();
  EXPECT_TRUE((pose * Eigen::Vector3d(0, 0, 1)).isApprox(Eigen::Vector3d(1, 0, 0)));
  EXPECT_TRUE((pose * Eigen::Vector3d(1, 0, 0)).isApprox(Eigen::Vector3d(0, -1, 0)));
  EXPECT_TRUE((pose * Eigen::Vector3d(0, 1, 0)).isApprox(Eigen::Vector3d(0, 0, -1)));
  EXPECT_TRUE(lx_camera::IsRigidTransform(pose));
}

TEST(FrameGeometry, RgbBaselineIsInvertedAndConvertedToMeters) {
  const float calibration[] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 50, 0, 0};
  const auto color_pose = lx_camera::ColorPoseInDepth(calibration);
  // p_color = p_depth + [50,0,0] mm: color origin is 50 mm left of depth.
  EXPECT_TRUE(color_pose.translation().isApprox(Eigen::Vector3d(0, 0.05, 0), 1e-6));
  EXPECT_TRUE(color_pose.linear().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(FrameGeometry, NonSymmetricRotationAndTranslationPreserveOpticalMapping) {
  // +90 degrees about optical Z, with a nonzero translation on all axes.
  const float calibration[] = {0, -1, 0, 1, 0, 0, 0, 0, 1, 10, 20, 30};
  const auto depth_from_color = lx_camera::ColorPoseInDepth(calibration);
  const auto body_from_optical = lx_camera::OpticalToBody();
  const Eigen::Vector3d depth_point(0.1, 0.2, 1.0);
  const Eigen::Vector3d color_point(-0.19, 0.12, 1.03);
  // Compare full paths, detecting row/column order and inversion errors.
  EXPECT_TRUE((depth_from_color * body_from_optical * color_point).isApprox(
      body_from_optical * depth_point, 1e-6));
}

TEST(FrameGeometry, RejectsMissingNonFiniteAndNonRigidCalibration) {
  EXPECT_THROW(lx_camera::ColorPoseInDepth(nullptr), std::invalid_argument);
  float calibration[] = {1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0};
  EXPECT_THROW(lx_camera::ColorPoseInDepth(calibration), std::invalid_argument);
  calibration[8] = 1;
  calibration[9] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(lx_camera::ColorPoseInDepth(calibration), std::invalid_argument);
  calibration[9] = 0;
  calibration[0] = 0;
  EXPECT_THROW(lx_camera::ColorPoseInDepth(calibration), std::invalid_argument);
}

TEST(FrameGeometry, NamesFollowNamespaceAndNodeWithoutLeadingSlash) {
  const FrameNames a("/uav1", "rgbd");
  const FrameNames b("/fleet/uav2/", "rear", "mount");
  EXPECT_EQ(a.link, "uav1/rgbd/link");
  EXPECT_EQ(b.link, "fleet/uav2/rear/mount");
  EXPECT_EQ(b.depth_optical, "fleet/uav2/rear/depth_optical");
  EXPECT_NE(a.depth, b.depth);
  EXPECT_EQ(FrameNames("/", "rgbd").link, "rgbd/link");
  EXPECT_THROW(FrameNames("/uav1", "rgbd", "depth"), std::invalid_argument);
  EXPECT_THROW(FrameNames("/uav1", "rgbd", ""), std::invalid_argument);
  EXPECT_THROW(FrameNames("/uav1", "rgbd", "bad frame"), std::invalid_argument);
}

TEST(FrameGeometry, AlignmentUsesDestinationOpticalFrame) {
  const FrameNames frames("uav1", "rgbd");
  EXPECT_EQ(frames.DepthImage(0), frames.depth_optical);
  EXPECT_EQ(frames.ColorImage(0), frames.color_optical);
  for (int alignment : {1, 3}) {
    EXPECT_EQ(frames.DepthImage(alignment), frames.color_optical);
    EXPECT_EQ(frames.ColorImage(alignment), frames.color_optical);
    EXPECT_EQ(frames.Cloud(alignment, 0), frames.color_optical);
    EXPECT_EQ(frames.Cloud(alignment, 1), frames.color);
  }
  EXPECT_EQ(frames.ColorImage(2), frames.depth_optical);
  for (int alignment : {0, 2}) {
    EXPECT_EQ(frames.Cloud(alignment, 0), frames.depth_optical);
    EXPECT_EQ(frames.Cloud(alignment, 1), frames.depth);
  }
  EXPECT_THROW(frames.Cloud(0, -1), std::invalid_argument);
  EXPECT_THROW(frames.DepthImage(4), std::invalid_argument);
}

TEST(FrameGeometry, MountPoseUsesMetersAndDegreesInRzRyRxOrder) {
  const auto pose = lx_camera::PoseFromXyzRpy({1, 2, 3, 90, 0, 90});
  EXPECT_TRUE(pose.translation().isApprox(Eigen::Vector3d(1, 2, 3)));
  // Rx(90) sends Y to Z, then Rz(90) leaves Z unchanged.
  EXPECT_TRUE((pose * Eigen::Vector3d(0, 1, 0)).isApprox(Eigen::Vector3d(1, 2, 4)));
  EXPECT_THROW(lx_camera::PoseFromXyzRpy({0, 0}), std::invalid_argument);
  EXPECT_THROW(lx_camera::PoseFromXyzRpy({0, 0, 0, 0, 0,
      std::numeric_limits<double>::infinity()}), std::invalid_argument);
}

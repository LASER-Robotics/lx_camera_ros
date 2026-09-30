# mrdvs_wrapper

The main launch file follows the arguments used in `rs_camera.launch.py`:

| Argument | Default | Purpose |
| --- | --- | --- |
| `uav_name` | `UAV_NAME` environment variable, or `uav1` | Node and topic namespace |
| `camera_name` | `rgbd` | Camera node name |
| `use_sim_time` | `true` if `REAL_UAV=false`; otherwise, `false` | ROS clock for the driver and RViz |
| `camera_params_file` | `params/default.yaml` from the installed package | Driver settings |
| `enable_rviz` | `false` | Opens RViz with topics in the selected namespace |
| `rviz_frame` | `<uav_name>/<camera_name>/link` | RViz fixed frame |

Build the package in the workspace to install the `params` directory as well:

```bash
cd ~/laser_uav_system_ws
colcon build --packages-select lx_camera_ros --symlink-install
source install/setup.bash
ros2 launch lx_camera_ros lx_camera_ros.launch.py \
  uav_name:=uav1 camera_name:=rgbd use_sim_time:=false enable_rviz:=true
```

To use another configuration, pass `camera_params_file:=/path/to/camera.yaml`.
The YAML supports substitutions such as `$(var uav_name)` and `$(var camera_name)`.
The previous parameters were moved to `params/default.yaml`, preserving
`LX_INT_XYZ_UNIT: 1` (meters) and the commented device configuration options.

With the defaults, the node is named `/uav1/rgbd` and topics include
`/uav1/LxCamera_Depth` and `/uav1/LxCamera_Cloud`. The LX driver publishes topics
relative to the namespace: `camera_name` changes the node name without adding
a level to the topic paths.

`use_sim_time` configures the ROS clock, while data timestamped directly by the
SDK continues to use sensor timestamps.

The driver publishes the internal camera geometry on `/tf_static`:

```text
uav1/fcu                         (external mounting TF by default)
└── uav1/rgbd/link               (depth sensor origin, body axes)
    ├── uav1/rgbd/depth
    │   └── uav1/rgbd/depth_optical
    └── uav1/rgbd/color          (requires valid SDK extrinsics)
        └── uav1/rgbd/color_optical
```

Body frames use X forward, Y left, Z up. Optical frames use X right, Y down,
Z forward. Frame IDs are derived from the actual node namespace and name, with
no leading slash. `base_frame_id` changes the `link` suffix. The link origin
is a sensor reference, not the mechanical center of the S11 housing.

The existing deployment owns `uav1/fcu -> uav1/rgbd/link`. To publish this edge
from the driver instead, set `publish_mount_tf: true` and configure
`parent_frame_id` plus `x`, `y`, `z` in meters and `roll`, `pitch`, `yaw` in
degrees. Keep one publisher per edge. `publish_tf: false` disables all driver
TF publication. RViz uses the camera link by default, so it also works without
an external mounting transform. Override `rviz_frame` if the base name changes.

Images and their `CameraInfo` messages share the same optical frame and sensor
timestamp. `LxCamera_AmpInfo` provides matching information for the amplitude
image. RGB-D alignment selects the destination optical frame: modes 1/3 use
color for depth; mode 2 uses depth for RGB. Point clouds select optical or body
axes using the effective `LX_INT_XYZ_COORDINATE` setting and are always published
in meters, including when the SDK returns millimeters.

RGB extrinsics follow the convention in the manufacturer's
[ROS2 sample](https://github.com/Lanxin-MRDVS/CameraSDK/blob/master/Sample/ros2/lx_camera_node_ws/src/lx_camera_ros/src/lx_camera/lx_camera.cpp):
read a row-major matrix, convert the translation from millimeters to meters,
invert the depth-to-color coordinate mapping, then change from optical to body
bases. Missing, non-finite, or non-rigid calibration produces an error and no
color TF; the driver does not substitute an identity calibration.

The installed SDK does not document the reference axes/direction of the IMU
extrinsics. When the IMU is enabled, messages use `<prefix>/imu` in native SDK
axes. Provide a measured `imu_pose: [x, y, z, roll, pitch, yaw]` (meters/degrees)
relative to camera link, or publish that transform externally. Without either,
the IMU frame remains unconnected. No unverified IMU extrinsic is broadcast.

The legacy `base_link`, `mrdvs_*`, `intrinsic_*` frames and `LxCamera_TF` topic
are no longer published by this node. Algorithm results use a separate
`<prefix>/application` frame without an assumed TF: their SDK coordinate contract
must be established before connecting them to the sensor tree. Hardware
validation of RGB/depth overlap and S11 mounting/calibration remains necessary.

Geometry tests cover optical axes, SDK matrix direction/layout, unit conversion,
alignment frame selection, frame naming, invalid calibration, and mounting angles.
ROS integration tests use a fake SDK (no physical camera) to check retained static
TF, mounting/IMU options, disabled TF, and image/CameraInfo/cloud consistency:

```bash
colcon test --packages-select lx_camera_ros --ctest-args -R 'test_(frame_geometry|driver_frames)'
colcon test-result --verbose
```

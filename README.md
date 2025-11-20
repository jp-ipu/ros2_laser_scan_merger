# ROS2 Laser Scan Merger

A modern C++20 ROS2 package to merge multiple laser scan topics into a single virtual laser scan using TF2 transforms. Features a testable architecture with core algorithms separated from ROS2 infrastructure.

![laser scan merger configurator](https://github.com/mich1342/ros2_laser_scan_merger/blob/main/LidarCallbration.png)

## Key Features

✨ **Modern Architecture**
- Testable core library with zero external dependencies (no PCL required)
- Comprehensive unit tests for all algorithms
- Clean separation between business logic and ROS2 integration
- Uses pure ROS2 sensor_msgs for point cloud construction

🚀 **Performance Optimized**
- TF2 transform caching for ~1000x speedup on fixed geometry
- Thread-safe with mutex protection
- Configurable publishing rates and synchronization modes

🎯 **Easy Configuration**
- Auto-detects source frames from scan headers
- Global parameters for same-type lidars
- Only configure topics and colors per laser
- Comprehensive YAML configuration with examples

🔧 **Flexible**
- Supports N laser scanners dynamically
- Optional scan synchronization (timer-based or scan-triggered)
- Dynamic TF support for moving sensors
- Per-laser angle filtering and visualization colors

## Prerequisites

- **ROS2 Jazzy** (tested) or Humble (should work)
- **C++20 compatible compiler** (GCC 10+, Clang 10+)
- Your laser scan drivers
- [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan) package
- RVIZ2 (for visualization)

## Installation

1. **Clone to your ROS2 workspace:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/mich1342/ros2_laser_scan_merger.git
```

2. **Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_laser_scan_merger
source install/setup.bash
```

## Quick Start

### 1. Set Up TF Tree

The package requires proper TF transforms. You must publish transforms from your `destination_frame` to each laser's frame.

**Using static_transform_publisher:**
```bash
# Base to merged laser frame
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser

# Laser frame to individual lidars
ros2 run tf2_ros static_transform_publisher -0.3 -0.475 0.176 0 0 0 laser lidar_1
ros2 run tf2_ros static_transform_publisher 0.3 0.475 0.176 0 0 3.14159 laser lidar_2
```

**Or use URDF/robot_state_publisher** (recommended for robots)

### 2. Configure Parameters

Edit `config/params.yaml`:

```yaml
/ros2_laser_scan_merger:
  ros__parameters:
    # How many lasers to merge
    num_lasers: 2

    # Target frame for merged output
    destination_frame: laser

    # Global parameters (shared by all same-type lidars)
    angle_min: -4.0      # degrees
    angle_max: 94.0      # degrees
    flip: false
    inverse: true

    # Per-laser configuration (only unique settings)
    laser0:
      topic: /lidar_1/scan
      show: true
      r: 255  # Red color
      g: 0
      b: 0

    laser1:
      topic: /lidar_2/scan
      show: true
      r: 0
      g: 0
      b: 255  # Blue color
```

**Note:** Source frames are automatically detected from `scan->header.frame_id`

### 3. Launch

**Without visualization:**
```bash
ros2 launch ros2_laser_scan_merger laserscan_multi_merger.launch.py
```

**With RVIZ2 visualization:**
```bash
ros2 launch ros2_laser_scan_merger laserscan_visualizer.launch.py
```

Both launch files include the `pointcloud_to_laserscan` node for LaserScan output.

## Configuration Guide

### Global Parameters

These apply to **all lasers** (assumes same lidar model):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_lasers` | 2 | Number of laser scanners to merge |
| `destination_frame` | laser | Target frame for merged cloud |
| `pointCloudTopic` | cloud_in | Output point cloud topic |
| `publish_rate` | 30.0 | Publishing rate in Hz |
| `angle_min` | -4.0 | Minimum angle to include (degrees) |
| `angle_max` | 94.0 | Maximum angle to include (degrees) |
| `flip` | false | Flip scan data order |
| `inverse` | true | Inverse angle filtering logic |

### Per-Laser Parameters

Configure for **each laser** (laser0, laser1, laser2, ...):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `topic` | /scan_N | Input laser scan topic (**required**) |
| `show` | true | Enable/disable this laser |
| `r`, `g`, `b` | 255,0,0 | RGB color for visualization (0-255) |
| `angle_min` | (global) | Optional: Override global angle_min for this laser (degrees) |
| `angle_max` | (global) | Optional: Override global angle_max for this laser (degrees) |

**Notes:**
- `source_frame` is **automatically detected** from `scan->header.frame_id`
- `angle_min`/`angle_max` are optional - if not specified, uses global values
- Useful for lasers at different orientations or requiring different FOV

### Performance & Synchronization

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_fixed_transforms` | true | Cache TF transforms (recommended for fixed sensors) |
| `tf_timeout` | 1.0 | TF lookup timeout (seconds) |
| `use_scan_triggering` | false | Publish on scan arrival vs timer |
| `require_all_scans` | false | Wait for all lasers (hard sync) |
| `max_scan_age` | 1.0 | Warn about stale scans (seconds) |
| `skip_stale_scans` | false | Exclude old scans from output |

## Running Tests

The package includes comprehensive unit tests for all core algorithms.

### Run All Tests
```bash
cd ~/ros2_ws
colcon test --packages-select ros2_laser_scan_merger
colcon test-result --verbose
```

### Test Coverage
- ✅ Quaternion to rotation matrix conversion
- ✅ 3D point transformations
- ✅ Polar to Cartesian conversions
- ✅ Angle filtering (normal and inverse modes)
- ✅ Scan flipping
- ✅ Invalid range filtering
- ✅ Processing statistics

Tests run **without ROS2 nodes** for fast execution.

## Architecture

### Testable Core Library (`laser_scan_merger_core`)
**No ROS2 dependencies** - pure C++20 algorithms:
- `math_utils.cpp` - Pure math functions
  - QuaternionToTransform()
  - ApplyTransform()
  - PolarToCartesian()
  - ShouldIncludePoint()
- `scan_processor.cpp` - Core scan processing
  - LaserScanProcessor::ProcessScan()

### ROS2 Wrapper (`ros2_laser_scan_merger` node)
Thin adapter handling:
- Publishers/subscribers
- TF2 lookups and caching
- Parameter loading
- ROS2 logging
- Direct PointCloud2 construction (no PCL dependency)

### Benefits
- 🧪 100% unit testable core algorithms
- ⚡ Fast test execution (no ROS2 overhead)
- 🔍 Easy to debug and verify
- 📦 Can be used in non-ROS projects

## Advanced Usage

### Adding More Lasers

1. Edit `params.yaml`:
```yaml
num_lasers: 3

laser2:
  topic: /lidar_3/scan
  show: true
  r: 0
  g: 255
  b: 0  # Green color
  # Optional: custom angle filtering for this laser
  angle_min: -10.0  # Override global angle_min
  angle_max: 100.0  # Override global angle_max
```

2. Add TF transform for the new laser:
```bash
ros2 run tf2_ros static_transform_publisher X Y Z R P Y laser lidar_3
```

**Note:** Per-laser `angle_min`/`angle_max` are optional. If not specified, the laser uses global values.

### Using Dynamic Transforms

For moving sensors (PTZ mounts, articulated platforms):

```yaml
use_fixed_transforms: false  # Lookup TF every scan
```

**Note:** This is slower but handles dynamic geometry.

### Hard Synchronization

To wait for all lasers before publishing (like ira_laser_tools):

```yaml
use_scan_triggering: true
require_all_scans: true
```

## Comparison with ira_laser_tools

| Feature | ros2_laser_scan_merger | ira_laser_tools |
|---------|----------------------|-----------------|
| ROS2 Native | ✅ Yes | ❌ No (port) |
| TF Caching | ✅ ~1000x faster | ❌ No |
| Testable Core | ✅ 100% unit tested | ❌ Tightly coupled |
| Thread Safe | ✅ Mutex protected | ⚠️ Limited |
| Modern C++ | ✅ C++20 | ⚠️ C++11 |
| No PCL Dependency | ✅ Pure ROS2 msgs | ❌ Requires PCL |
| Auto Frame Detection | ✅ From scan header | ❌ Manual config |
| Flexible Sync | ✅ Timer + triggered | ⚠️ Timer only |

## Troubleshooting

### "TF lookup failed" errors

**Problem:** Missing TF transforms

**Solution:**
1. Check your TF tree: `ros2 run tf2_tools view_frames`
2. Verify frame IDs match scan headers: `ros2 topic echo /lidar_1/scan --field header.frame_id`
3. Publish required transforms (see Quick Start)

### No output on `/cloud_in`

**Problem:** Scans not being received or all filtered out

**Solution:**
1. Check input topics: `ros2 topic list | grep scan`
2. Verify `show` parameter is `true` for lasers
3. Check angle filtering isn't excluding all points
4. Monitor: `ros2 topic hz /cloud_in`

### Build errors

**Problem:** Missing dependencies

**Solution:**
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ros2_laser_scan_merger
```

## Contributing

Contributions welcome! The testable architecture makes it easy to add features:

1. Add new algorithm → `src/math_utils.cpp` or `src/scan_processor.cpp`
2. Add tests → `test/test_math_utils.cpp` or `test/test_scan_processor.cpp`
3. Verify with `colcon test`
4. Update ROS2 wrapper if needed → `src/main.cpp`

## License

Apache License 2.0

## Credits

- Original package: [Michael Jonathan (mich1342)](https://github.com/mich1342)
- Inspired by: [ira_laser_tools](https://github.com/iralabdisco/ira_laser_tools)
- Refactored for testability, TF2 integration, and modern C++20

## Citation

If you use this package in your research, please cite:

```bibtex
@software{ros2_laser_scan_merger,
  author = {Jonathan, Michael},
  title = {ROS2 Laser Scan Merger},
  year = {2022},
  url = {https://github.com/mich1342/ros2_laser_scan_merger}
}
```

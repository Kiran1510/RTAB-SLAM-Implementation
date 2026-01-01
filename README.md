# RTAB-SLAM with IMU Integration for Indoor Mapping

A ROS2 implementation of RGB-D SLAM using RTAB-MAP with Intel RealSense D435 camera and VectorNav VN-100 IMU for indoor environment mapping and localization.

## Project Overview

This project implements Real-Time Appearance-Based Mapping (RTAB-MAP) for 3D reconstruction and simultaneous localization and mapping (SLAM) in residential indoor environments. The system combines RGB-D visual data with IMU measurements to achieve robust pose estimation and dense point cloud reconstruction.

**Authors:** Kiran Sairam Bethi Balagangadaran, Abhinav Vaddiraju, Prasath Saravanan  
**Institution:** Northeastern University  
**Course:** EECE5554 - Robot Sensing and Navigation  
**Date:** December 2024

## Key Features

- **Dense 3D Reconstruction:** Point cloud mapping with 150-250 tracked features in textured regions
- **Loop Closure Detection:** Bag-of-words based approach with 4/6 successful detections
- **Low Drift:** 2-3% translational drift over traveled distance
- **Real-Time Performance:** 40-60ms processing time per frame at 15Hz
- **Sensor Fusion:** RGB-D camera with IMU integration for improved pose estimation

## Hardware Requirements

### Required Components
- **Intel RealSense D435** RGB-D Camera (1280×720, 30Hz, 0.3-3m range)
- **VectorNav VN-100** IMU (9-DOF, 100Hz native)
- **Computing Platform:** 
  - CPU: Intel i7-13750H or equivalent
  - GPU: RTX 4070 or equivalent (optional, CPU-bound operation)
  - RAM: 16GB minimum
  - USB 3.2 ports (dedicated controller recommended)

### Custom Mounting
- 3D-printed bracket for rigid camera-IMU mounting (design files included)
- Ensures fixed spatial relationship between sensors for accurate calibration

## Software Requirements

### Operating System
- Ubuntu 24.04 LTS (recommended)
- Ubuntu 22.04 LTS (compatible)

### Dependencies
```bash
# ROS2 Installation (Jazzy for Ubuntu 24.04)
sudo apt install ros-jazzy-desktop

# RTAB-MAP
sudo apt install ros-jazzy-rtabmap-ros

# RealSense SDK
sudo apt install ros-jazzy-realsense2-camera

# VectorNav ROS2 Driver
# Clone from: https://github.com/dawonn/vectornav

# Additional Tools
sudo apt install ros-jazzy-tf2-tools
sudo apt install ros-jazzy-rqt-*
```

## Installation

1. **Clone the repository:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/Kiran1510/RTAB-SLAM-Implementation.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

2. **Verify sensor connections:**
```bash
# Check RealSense
rs-enumerate-devices

# Check VectorNav (replace /dev/ttyUSB0 with your device)
ls -l /dev/ttyUSB*
```

3. **Configure udev rules for VectorNav:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for group changes to take effect
```

## Usage

### Quick Start

**Launch RTAB-MAP with sensors:**
```bash
ros2 launch rtabmap_ros rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  depth_topic:=/camera/depth/image_rect_raw \
  rgb_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  approx_sync:=true \
  frame_id:=base_link \
  queue_size:=30
```

### Recording Data

**Capture rosbag for offline processing:**
```bash
ros2 bag record -o indoor_mapping \
  /camera/color/image_raw \
  /camera/depth/image_rect_raw \
  /camera/color/camera_info \
  /imu/data
```

### Visualization

**Launch RViz with RTAB-MAP visualization:**
```bash
rviz2 -d $(ros2 pkg prefix rtabmap_ros)/share/rtabmap_ros/launch/config/rgbd.rviz
```

**View real-time point cloud:**
- Add PointCloud2 display
- Set topic to `/rtabmap/cloud_map`
- Set Fixed Frame to `map`

## Configuration

### Camera Settings
Located in `config/realsense_config.yaml`:
```yaml
camera:
  resolution: 1280x720
  fps: 15
  depth_range: [0.3, 4.0]
  enable_ir_emitter: true
  enable_auto_exposure: true
```

### IMU Settings
Located in `config/vectornav_config.yaml`:
```yaml
imu:
  output_rate: 15  # Hz (downsampled from 100Hz)
  use_hardware_timestamps: true
```

### RTAB-MAP Parameters
Key parameters in launch file:
- `approx_sync: true` - Enable approximate time synchronization (60ms tolerance)
- `queue_size: 30` - Message buffer size
- `Rtabmap/DetectionRate: 1.0` - Loop closure detection frequency
- `Vis/MinInliers: 15` - Minimum feature matches for odometry
- `GFTT/MinDistance: 7` - Feature detector spacing

## Project Structure

```
RTAB-SLAM-Implementation/
├── config/
│   ├── realsense_config.yaml
│   ├── vectornav_config.yaml
│   └── rtabmap_params.yaml
├── launch/
│   ├── rtabmap_rgbd.launch.py
│   └── sensors.launch.py
├── src/
│   ├── timestamp_diagnostic.cpp
│   └── sensor_sync_node.cpp
├── docs/
│   ├── RTAB_Report_RSN.pdf
│   └── hardware_setup.md
├── cad/
│   └── camera_imu_mount.stl
├── results/
│   ├── point_cloud_map.png
│   └── trajectory_visualization.png
└── README.md
```

## Results

### Achieved Performance
- **Mapping Quality:** Dense 3D reconstruction with clear room geometry
- **Translational Drift:** 2-3% of traveled distance without loop closures
- **Rotational Drift:** 1-2° per 360° rotation
- **Loop Closure Success:** 4/6 intentional loops detected (0 false positives)
- **Feature Tracking:** 150-250 features in textured areas, 50-80 in plain regions
- **Processing Speed:** Real-time at 15Hz (40-60ms per frame)

### Known Limitations
- **Depth Quality:** Degrades beyond 2.5m and in bright lighting
- **Frame Dropping:** Variable publishing rate (2.7-5.7Hz actual vs. 15Hz target)
- **Featureless Regions:** Tracking loss in plain walls/uniform surfaces
- **USB Bandwidth:** Frame corruption under heavy data load

## Troubleshooting

### Frame Dropping Issues
**Problem:** RTAB-MAP warnings about missing frames
```
[WARN] [rtabmap]: Frame dropped (X frames since last update)
```

**Solutions:**
1. Reduce camera resolution: `1280×720 → 640×480`
2. Lower frame rate: `30Hz → 15Hz`
3. Use dedicated USB 3.2 controller
4. Increase `approx_sync` tolerance to 60-80ms

### Timestamp Synchronization
**Problem:** IMU and camera timestamps misaligned

**Solutions:**
1. Enable hardware timestamps for both sensors
2. Check clock source synchronization
3. Increase `queue_size` parameter
4. Monitor with: `ros2 topic hz /camera/color/image_raw`

### Poor Tracking in Featureless Areas
**Problem:** Odometry fails on plain walls

**Solutions:**
1. Add visual markers or textured objects to environment
2. Reduce movement speed (< 0.3 m/s)
3. Increase `GFTT/QualityLevel` parameter
4. Enable IMU integration for better pose prediction

## Demo & Resources

- **Video Demonstration:** [Real-time Mapping Process](https://drive.google.com/file/d/1CPGT6lmnXAXy45TrEUCbpqtfO6Q4qTmB/view?usp=sharing)
- **Technical Report:** `docs/RTAB_Report_RSN.pdf`
- **Hardware Setup Guide:** `docs/hardware_setup.md`

## Future Improvements

I have acquired a Zed 2i Stereo Depth Camera with an inbuilt IMU, which eliminates the need for the RealSense and VN-100 jig/contraption. This will also make coordinate frame transforms much more simplified and straightforward. I will be doing this same implementation on the Zed Camera, with hopefully better results this time.

- [ ] Hardware GPIO triggering for perfect sensor synchronization
- [ ] Implement tighter loop closure detection (reduce missed loops)
- [ ] Add depth image preprocessing to handle bright lighting
- [ ] Optimize memory management for longer mapping sessions (>10 minutes)
- [ ] Integrate wheel odometry for better drift estimation
- [ ] Deploy on mobile robot platform for autonomous navigation

## Citation

If you use this work in your research, please cite:

```bibtex
@techreport{rtabslam2024,
  author = {Bethi Balagangadaran, Kiran Sairam and Vaddiraju, Abhinav and Saravanan, Prasath},
  title = {RTAB-SLAM with Integration of IMU Data for Indoor Mapping},
  institution = {Northeastern University},
  year = {2024},
  type = {Course Project Report},
  number = {EECE5554}
}
```

## References

1. M. Labbé and F. Michaud, "RTAB-Map as an Open-Source Lidar and Visual SLAM Library," *Journal of Field Robotics*, 2019
2. Intel RealSense D435 [Product Datasheet](https://www.intelrealsense.com/depth-camera-d435/)
3. VectorNav VN-100 [IMU Documentation](https://www.vectornav.com/products/vn-100)

## License

This project is released under the MIT License. See `LICENSE` file for details.

## Acknowledgments

- Dr. Kris Dorsey and EECE5554 teaching staff at Northeastern University
- RTAB-MAP development team
- ROS2 and Intel RealSense communities

## Contact

**Kiran Sairam Bethi Balagangadaran**  
Email: bethi.k@northeastern.edu  
GitHub: [@Kiran1510](https://github.com/Kiran1510)

---

*Last Updated: December 2024*

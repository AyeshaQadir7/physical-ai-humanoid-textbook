---
id: hw-recommended-setup
title: "Recommended Setup (Jetson + Hardware)"
sidebar_position: 2
sidebar_label: "Recommended Setup"
description: "Optimal hardware configuration for deploying to real robots: Jetson Orin Nano, RealSense D435i, ReSpeaker"
keywords: [Jetson, RealSense, ReSpeaker, hardware, edge AI, embodied AI]
---

## Overview

This chapter describes the **recommended hardware setup** for students who want to:
- Run perception and control algorithms on edge hardware
- Interface with real sensors (cameras, microphones)
- Prepare for optional physical robot deployment
- Develop VLA systems with multimodal input

This setup enables both simulation-based learning AND edge hardware development, providing a bridge between simulation and physical robots.

---

## Bill of Materials (Recommended Kit)

### Core Components

| Item | Model | Qty | Cost (USD) | Purpose |
| --- | --- | --- | --- | --- |
| **Compute** | NVIDIA Jetson Orin Nano | 1 | $199 | Edge AI accelerator for perception |
| **Compute Power** | Official Jetson Power Adapter (25W) | 1 | $35 | Stable power supply |
| **Storage** | Samsung 128GB NVMe SSD | 1 | $25 | OS + code storage |
| **Camera** | Intel RealSense D435i | 1 | $165 | RGB-D depth sensing |
| **Microphone Array** | ReSpeaker Mic Array v2.0 | 1 | $70 | Voice input, multi-channel audio |
| **Networking** | WiFi USB adapter (optional) | 1 | $20 | Wireless connectivity if needed |
| **Cooling** | Heatsink + fan for Jetson | 1 | $15 | Thermal management |
| **Accessories** | USB cables, brackets | - | $30 | Assembly & connectivity |

**Total Estimated Cost**: $559 USD

### Optional Additions

| Item | Cost | Reason |
| --- | --- | --- |
| 4G LTE modem | $40-60 | Remote robot connectivity |
| GPIO breakout board | $10 | PWM servo control |
| Robot platform (e.g., ROS Robot Base) | $300-500 | Physical mobility (optional) |
| Second RealSense camera | $165 | Stereo vision or full surround |

---

## Hardware Component Details

### 1. NVIDIA Jetson Orin Nano Developer Kit

**What it is**: A single-board computer with NVIDIA's Orin Nano GPU, designed for edge AI inference and real-time processing.

**Specifications**:
- **CPU**: 6-core ARM Cortex-A78AE @ 2.0 GHz
- **GPU**: 1024-CUDA-core NVIDIA Orin Nano
- **RAM**: 8 GB LPDDR5 (shared)
- **Storage**: microSD or NVMe SSD via USB
- **Power**: 5-25W (passive to active cooling)
- **Interfaces**: USB 3.0, USB Type-C, Ethernet, HDMI, GPIO pins
- **AI Performance**: 8-40 TFLOPS (FP32/INT8) for inference

**Why Choose Jetson Orin Nano**:
- ✅ ROS 2 Humble officially supported
- ✅ Pre-built Isaac ROS packages
- ✅ GPU-accelerated SLAM and perception
- ✅ Low power consumption (fits on mobile robots)
- ✅ Cost-effective for educational use

**Installation**:
```bash
# Flash JetPack OS on microSD
# 1. Download JetPack 5.1.2 (supports Humble)
# 2. Use NVIDIA SDK Manager or balena Etcher
# 3. Boot Jetson from microSD
# 4. Complete onboard setup wizard

# Verify after booting
cat /etc/nv_tegra_release
# Should show: R36 (release) or later
```

---

### 2. Intel RealSense D435i Depth Camera

**What it is**: A compact RGB-D camera combining color (1280×720) and depth (1280×720) sensors.

**Specifications**:
- **RGB Sensor**: 1280×720 @ 30 FPS
- **Depth Sensor**: 1280×720 @ 30 FPS (5m range)
- **Depth Technology**: Structured light (active) + stereo (passive fallback)
- **IMU**: 6-axis (accelerometer + gyroscope)
- **Interface**: USB 3.0
- **Power**: 380 mA @ 5V (powered via USB)

**Why RealSense D435i**:
- ✅ Depth + color in one device
- ✅ High accuracy for SLAM (5-10cm over 5m)
- ✅ ROS 2 drivers readily available
- ✅ IMU useful for odometry
- ✅ Well-supported in Isaac ROS perception

**ROS 2 Integration**:
```bash
# Install realsense2 package
sudo apt install ros-humble-realsense2-camera

# Launch camera
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 rgb_camera.profile:=1280x720x30

# Verify topics
ros2 topic list | grep camera
# /camera/color/image_raw
# /camera/depth/image_rect_raw
# /camera/imu
```

---

### 3. ReSpeaker Mic Array v2.0

**What it is**: A 6-microphone circular array optimized for voice recognition in noisy environments.

**Specifications**:
- **Microphones**: 6× MEMS omnidirectional mics
- **Beamforming**: 4 directional pickup patterns
- **Audio Format**: 16-bit, 16 kHz mono output
- **Interface**: USB
- **Power**: 100 mA @ 5V
- **LED Ring**: 12-LED RGB indicator for feedback

**Why ReSpeaker**:
- ✅ Beamforming reduces background noise
- ✅ Compact and easy to integrate
- ✅ ROS 2 drivers available
- ✅ Supports multiple voice commands simultaneously
- ✅ LED feedback for visual status

**ROS 2 Integration**:
```bash
# Install ReSpeaker driver
pip install respeaker

# Or use raw audio via pulseaudio
pactl list sources | grep respeaker

# ROS 2 audio bridge
ros2 run audio_common_nodes audio_node --ros-args -p device:=/dev/snd/by-id/usb-Respeaker_Mic_Array_v2.0*

# Verify
ros2 topic list | grep audio
# /audio (audio_common_msgs/AudioData)
```

---

## Complete Hardware Setup Guide

### Physical Assembly

**Step 1: Jetson Preparation**

1. Attach heatsink + fan to Jetson:
   - Apply thermal paste to GPU
   - Screw heatsink firmly
   - Connect fan header

2. Insert NVMe SSD:
   - Open M.2 slot cover
   - Insert SSD at 45° angle
   - Press down and screw

3. Optional: Mount in case with cooling

**Step 2: Camera & Microphone Mounting**

1. **RealSense D435i**:
   - Mount on robot front
   - Connect USB 3.0 to Jetson
   - Test initial alignment

2. **ReSpeaker Mic Array**:
   - Mount on robot top or body
   - Connect USB to Jetson
   - Position for omnidirectional input

**Step 3: Power Delivery**

1. Official Jetson power adapter → USB-C on Jetson
2. RealSense powered via USB 3.0 port
3. ReSpeaker powered via USB port
4. **Total power draw**: ~15W nominal; budget for 25W peaks

**Step 4: Network Connectivity**

```bash
# If using Ethernet
sudo apt install network-manager
nmtui  # Configure network

# If using WiFi
sudo apt install wpasupplicant wireless-tools
# ... configure via NetworkManager
```

---

## Software Installation on Jetson

### Step 1: Flash JetPack OS

1. Download NVIDIA SDK Manager on host PC
2. Select Jetson Orin Nano + JetPack 5.1.2
3. Follow guided flashing process
4. Complete onboard setup on Jetson

### Step 2: Install ROS 2 Humble

```bash
# Add ROS 2 repository (same as Ubuntu)
sudo apt install curl gnupg lsb-release ubuntu-keyring
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install ros-humble-desktop
```

### Step 3: Install Isaac ROS (Perception Accelerated)

```bash
# Clone Isaac ROS repository
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
# ... other Isaac ROS packages

# Build
cd ~/isaac_ros_ws
colcon build
```

### Step 4: Install Sensor Drivers

```bash
# RealSense
sudo apt install ros-humble-realsense2-camera librealsense2 librealsense2-dev

# ReSpeaker (via pip in Python environment)
pip install respeaker

# Audio common
sudo apt install ros-humble-audio-common
```

### Step 5: Verify Hardware Integration

```bash
# Test RealSense
ros2 launch realsense2_camera rs_launch.py &
ros2 topic echo /camera/color/image_raw | head -10

# Test ReSpeaker
ros2 run audio_common_nodes audio_node &
ros2 topic echo /audio | head -10

# Test Isaac ROS VSLAM (if installed)
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_jetson.launch.py
```

---

## Performance Benchmarks

### Jetson Orin Nano Performance

| Task | Performance | Notes |
| --- | --- | --- |
| Gazebo simulation | 20-30 FPS | Single 640×480 world |
| Isaac ROS VSLAM | 30 FPS @ 640×480 | Real-time visual odometry |
| YOLO v8 inference | 10-15 FPS | On 1280×720 RGB stream |
| ROS 2 node spin rate | 100-1000 Hz | Depending on workload |
| Whisper ASR (small) | ~2 sec per 30 sec audio | GPU accelerated |

### Power Consumption

| Workload | Power (W) | Duration |
| --- | --- | --- |
| Idle (desktop) | 5 | Continuous |
| ROS 2 idle spin | 7 | Continuous |
| Gazebo simulation | 12-15 | During simulation |
| Camera + SLAM | 15-18 | Real-time perception |
| Peak (all sensors + SLAM + inference) | 20-25 | During heavy compute |

---

## Cost-Performance Tradeoffs

### Why This Specific Configuration?

**Jetson Orin Nano vs. Alternatives**:

| Platform | Cost | GPU | ROS 2 Support | Best For |
| --- | --- | --- | --- | --- |
| **Jetson Orin Nano** | $199 | 1024 CUDA | ✅ Excellent | Learning + edge deployment |
| Jetson Orin Nano Developer Kit | $249 | 1024 CUDA | ✅ Excellent | Same, with accessories |
| Raspberry Pi 4 (8GB) | $75 | None (CPU) | ✅ Good | Low power; no GPU inference |
| Laptop (Intel i7) | $800+ | Optional | ✅ Excellent | Development; not portable |
| Jetson Orin NX | $149 | 512 CUDA | ✅ Excellent | Budget; half the performance |
| Jetson AGX Orin | $999+ | 12288 CUDA | ✅ Excellent | Industry-grade; excessive for learning |

**RealSense D435i vs. Alternatives**:

| Camera | Cost | Depth Tech | Accuracy | ROS 2 |
| --- | --- | --- | --- | --- |
| **RealSense D435i** | $165 | Structured light | 5-10 cm @ 5m | ✅ Official drivers |
| RealSense D455 | $199 | Stereo | 5-10 cm @ 5m | ✅ Official drivers |
| Azure Kinect DK | $299 | ToF | 5 cm @ 5m | ✅ Community drivers |
| ZED 2i | $399 | Stereo | 10 cm @ 10m | ✅ Official support |
| OAK-D | $100 | Stereo (mono+spatial) | 10-20 cm | ✅ Community |

**ReSpeaker vs. Alternatives**:

| Mic Array | Cost | Channels | Beamforming | ROS 2 |
| --- | --- | --- | --- | --- |
| **ReSpeaker v2.0** | $70 | 6 | ✅ Yes | ✅ Community |
| ReSpeaker USB | $50 | 4 | ✅ Yes | ✅ Community |
| Respeaker Core+ | $120 | 6 | ✅ Yes | ✅ Community |
| Blue Yeti Pro | $130 | 4 | ❌ No | ❌ Limited |
| Generic USB mic | $20 | 1 | ❌ No | ✅ Native audio |

---

## Integration Checklist

- [ ] Jetson Orin Nano flashed with JetPack 5.1.2
- [ ] ROS 2 Humble installed and verified
- [ ] RealSense camera detected: `realsense-viewer` shows streams
- [ ] ReSpeaker mic array detected: `pulseaudio` lists device
- [ ] All 3 devices accessible via ROS 2 topics
- [ ] Power supply rated for 25W sustained
- [ ] Wireless or Ethernet connectivity verified

---

## Troubleshooting

### Jetson Not Booting After JetPack Flash

**Solution**:
```bash
# Reflash using SDK Manager on host PC
# Select "Force Recovery Mode" if needed
```

---

### RealSense Camera Not Detected

**Solution**:
```bash
# Check USB connection
lsusb | grep Intel

# Install librealsense
sudo apt install librealsense2 librealsense2-dev

# Test
realsense-viewer
```

---

### ReSpeaker No Audio Input

**Solution**:
```bash
# List audio devices
pactl list sources | grep respeaker

# Test via PulseAudio
parecord -d <device-name> | aplay -f cd
```

---

## Next Steps

Once this hardware is set up:

1. **Complete Modules 0–2** on desktop/laptop (faster iteration)
2. **Deploy to Jetson** for Module 3 (Isaac ROS, SLAM acceleration)
3. **Integrate VLA** on Jetson for Module 4 (voice + perception)
4. **Capstone on Jetson**: Voice-controlled perception system

**Estimated time to first working example on Jetson**: 2-3 hours (after JetPack flashing)

---

## Cost Breakdown

| Category | Items | Cost |
| --- | --- | --- |
| **Compute** | Jetson + power + storage | $259 |
| **Perception** | RealSense D435i | $165 |
| **Audio** | ReSpeaker Mic Array | $70 |
| **Accessories** | Cables, cooling, mounting | $65 |
| **Total** | **All-in-one edge AI kit** | **$559** |

Compare to typical robotics systems: $2,000–$10,000 for equivalent hardware-enabled platform.

---

## Further Resources

- **Jetson Orin Nano Documentation**: [https://developer.nvidia.com/jetson-orin-nano](https://developer.nvidia.com/jetson-orin-nano)
- **RealSense Documentation**: [https://github.com/IntelRealSense/librealsense](https://github.com/IntelRealSense/librealsense)
- **ReSpeaker Documentation**: [https://wiki.seeedstudio.com/ReSpeaker-Mic-Array-v2.0/](https://wiki.seeedstudio.com/ReSpeaker-Mic-Array-v2.0/)
- **Isaac ROS**: [https://github.com/NVIDIA-ISAAC-ROS](https://github.com/NVIDIA-ISAAC-ROS)

---

**Last Updated**: 2025-12-10
**Relevant For**: Modules 2–4 (Simulation, Isaac, VLA)
**Capstone Connection**: Enables real-world perception and voice input for capstone integration

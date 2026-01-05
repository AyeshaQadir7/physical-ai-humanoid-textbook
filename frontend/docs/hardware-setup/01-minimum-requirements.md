---
id: hw-minimum-requirements
title: "Minimum Requirements (Simulator-Only Path)"
sidebar_position: 1
sidebar_label: "Minimum Requirements"
description: "Minimal system specs for learning ROS 2, Gazebo, and Isaac without physical hardware"
keywords: [hardware, requirements, simulator, Ubuntu, ROS 2, Gazebo]
---

## Overview

This chapter describes the **minimum system requirements** for completing the entire 13-week robotics course using simulation only. **No physical hardware is required**—all labs and the capstone project can be completed in software simulation environments.

---

## Recommended Operating System

### Ubuntu 22.04 LTS (Primary Recommendation)

**Why Ubuntu?**
- Official ROS 2 Humble supported platform
- Pre-packaged binaries for ROS 2, Gazebo, and Isaac tools
- Large community support and extensive documentation

**System Requirements**:
- **Processor**: Intel/AMD x86-64 CPU (2+ cores recommended)
- **RAM**: 4 GB minimum; 8 GB recommended for comfortable simulation
- **Storage**: 20 GB free space for OS, ROS 2, Gazebo, and dependencies
- **Internet**: Broadband connection for downloading packages

### Alternative Operating Systems

**macOS 12+ (Intel or Apple Silicon)**:
- ROS 2 Humble supported
- Gazebo available via Homebrew
- **Limitation**: Isaac Sim requires Linux; must use cloud-based Isaac Sim or skip Module 3 labs on Mac
- **RAM**: 8 GB minimum; 16 GB recommended

**Windows 11 + WSL2 (Windows Subsystem for Linux)**:
- Run Ubuntu 22.04 in WSL2 for full ROS 2/Gazebo support
- **Performance**: Slightly slower than native Linux; acceptable for learning
- **Storage**: 30 GB for WSL2 + Ubuntu + tools
- **Limitation**: GUI applications require X11 server (VcXsrv, WSLg)

---

## Hardware Specifications (Simulator-Only)

### Minimum Configuration

| Component | Specification | Notes |
| --- | --- | --- |
| **CPU** | 2+ cores @ 2 GHz | Single-core insufficient for running multiple nodes + Gazebo |
| **RAM** | 4 GB total | Gazebo ~1 GB; ROS 2 nodes ~0.5-1 GB each; OS ~1 GB |
| **Storage** | 20 GB free | 10 GB OS + 10 GB tools/datasets |
| **GPU** | Optional | CPU physics sufficient for learning; GPU accelerates rendering only |
| **Network** | 10+ Mbps | For downloading packages and cloud services |

### Recommended Configuration (Better Experience)

| Component | Specification | Benefits |
| --- | --- | --- |
| **CPU** | 4+ cores @ 2.5+ GHz | Parallel simulation, faster colcon builds |
| **RAM** | 8-16 GB | Multiple Gazebo instances, Isaac Sim (if available) |
| **Storage** | 50 GB SSD | Faster builds, smoother Gazebo performance |
| **GPU** | Discrete GPU (GTX 1050+) | Faster Gazebo rendering, near real-time simulation |
| **Network** | Broadband 50+ Mbps | Quick package downloads, cloud Isaac Sim |

### Real Hardware Laptop/Desktop Examples

**Example 1: Budget Laptop (Suitable for Simulator)**
- Lenovo ThinkPad E-series (2018+)
- CPU: Intel Core i5 @ 2.6 GHz, 4 cores
- RAM: 8 GB
- Storage: 256 GB SSD
- **Performance**: Acceptable for this course; Gazebo may run at 20-30 FPS

**Example 2: Developer Workstation (Optimal for Simulator)**
- MacBook Pro 16" M2/M3
- CPU: Apple M2/M3 (8+ cores)
- RAM: 16 GB
- Storage: 512 GB SSD
- **Performance**: Excellent; Gazebo runs at 60 FPS; Isaac Sim cloud-accessible

**Example 3: Desktop Gaming PC**
- CPU: Ryzen 5 5600X @ 3.7 GHz, 6 cores
- GPU: RTX 3060 Ti
- RAM: 32 GB
- Storage: 1 TB NVMe SSD
- **Performance**: Exceptional; Isaac Sim local with smooth photorealistic rendering

---

## Software Stack (Minimum)

### Core ROS 2 Installation

**ROS 2 Humble** (current distribution; LTS support until 2027):

```bash
# Install ROS 2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --version
```

### Gazebo

**Gazebo (latest compatible with ROS 2 Humble)**:

```bash
sudo apt install gazebo

# Verify
gazebo --version
```

### Python & Development Tools

```bash
# Python 3.10+ (typically pre-installed on Ubuntu 22.04)
python3 --version

# Essential build tools
sudo apt install build-essential cmake git python3-pip python3-dev

# colcon build tool (ROS 2 build system)
sudo apt install python3-colcon-common

# Text editor
sudo apt install code    # VS Code recommended
# OR: sudo apt install vim
```

### Optional but Recommended

```bash
# ROS 2 development tools
sudo apt install ros-humble-rosdep ros-humble-rqt

# Python packages for labs
pip install numpy scipy matplotlib opencv-python

# For capstone VLA development (later)
pip install openai whisper
```

---

## Installation Step-by-Step

### Step 1: Install Ubuntu 22.04 LTS

**Option A**: Native Installation
- Download ISO from [ubuntu.com](https://ubuntu.com)
- Create bootable USB
- Install on machine or partition

**Option B**: Virtual Machine
- Download VirtualBox or VMware Player
- Create VM with 20+ GB storage, 8 GB RAM allocation
- Install Ubuntu 22.04 in VM

**Option C**: WSL2 on Windows
```bash
# In Windows PowerShell (as Administrator)
wsl --install -d Ubuntu-22.04

# After reboot, Ubuntu terminal opens; complete setup
```

### Step 2: Update System Packages

```bash
sudo apt update
sudo apt upgrade -y
```

### Step 3: Install ROS 2 Humble

```bash
# Add ROS 2 repository
sudo apt install curl gnupg lsb-release ubuntu-keyring
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

# Add repository
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

### Step 4: Set Up Workspace

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone example package (optional)
git clone https://github.com/ros2/examples src/examples

# Build workspace
colcon build

# Source setup script
source install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 5: Verify Installation

```bash
# Test ROS 2
ros2 node list       # Should show no error

# Start a simple publisher/subscriber demo
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_cpp listener
```

---

## Disk Space & Bandwidth Estimates

| Component | Space | Download Time (50 Mbps) |
| --- | --- | --- |
| Ubuntu 22.04 LTS | 3 GB | 8 minutes |
| ROS 2 Humble (binary) | 1.5 GB | 4 minutes |
| Gazebo + dependencies | 2 GB | 5 minutes |
| Development tools (Python, GCC, CMake) | 2 GB | 5 minutes |
| **Workspace + labs** | 3-5 GB | 8-12 minutes |
| **Isaac Sim (if installing locally on Linux)** | 50+ GB | 30+ minutes |
| **Total for course (simulator only)** | ~12-15 GB | ~30-40 minutes |

---

## Network Requirements

### Bandwidth
- **Minimum**: 5 Mbps (for text-based packages)
- **Recommended**: 25+ Mbps (for faster downloads)

### Internet Connectivity
Required for:
- Downloading OS and package repositories
- Installing ROS 2 and Gazebo via `apt`
- Downloading example code from GitHub
- API calls to LLM services (Module 4)
- Cloud-based Isaac Sim (if not local)

### Offline Setup
If offline installation is needed:
1. Download all packages on a machine with internet
2. Use `apt-offline` to create offline package cache
3. Transfer cache to target machine via USB
4. Follow installation steps with cached packages

---

## Troubleshooting Common Setup Issues

### Issue: ROS 2 commands not found

**Symptom**: `ros2: command not found`

**Solution**:
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Or add permanently to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Issue: Gazebo won't start or crashes

**Symptom**: `gazebo: command not found` or segmentation fault

**Solution**:
```bash
# Reinstall Gazebo
sudo apt remove gazebo* -y
sudo apt install gazebo

# Check graphics drivers
glxinfo | grep -i opengl
```

---

### Issue: colcon build fails with missing dependencies

**Symptom**: `Package X is not found`

**Solution**:
```bash
# Install rosdep and resolve dependencies
sudo rosdep init
rosdep update

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build
```

---

### Issue: Slow simulation or rendering lag

**Symptom**: Gazebo runs at under 10 FPS; simulation feels choppy

**Solutions**:
1. Reduce physics complexity:
   ```xml
   <!-- In world.sdf -->
   <physics name="default_physics" default="0" type="ode">
     <max_step_size>0.01</max_step_size>
     <real_time_factor>0.9</real_time_factor>
   </physics>
   ```

2. Use "faster than real-time" simulation:
   ```bash
   gazebo --verbose my_world.sdf
   # Check "Real Time Factor" in console; aim for ~1.0 or higher
   ```

3. Upgrade RAM or close other applications

4. Switch to simpler physics engine or reduce sensor update rates

---

## What's NOT Included in Minimum Setup

- **Physical Hardware**: Jetson, robot, sensors, actuators (all optional)
- **Isaac Sim (local)**: Requires significant GPU resources; available via cloud
- **Real-time Operating System**: Linux desktop sufficient for learning
- **Industrial Tools**: RViz advanced plugins, Gazebo plugins (basic version included)

---

## Next Steps

Once your system is set up and verified:

1. **Proceed to Module 0**: Introduction to Physical AI concepts
2. **Start Module 1**: ROS 2 fundamentals with hands-on examples
3. **Lab 1.1**: Your first ROS 2 publisher/subscriber

**Time to first working example**: ~30 minutes after installation

---

## Additional Resources

- **ROS 2 Humble Installation**: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)
- **Gazebo Installation**: [https://gazebosim.org/docs/latest/install/](https://gazebosim.org/docs/latest/install/)
- **Ubuntu 22.04 Download**: [https://ubuntu.com/download](https://ubuntu.com/download)
- **WSL2 Setup Guide**: [https://learn.microsoft.com/en-us/windows/wsl/install](https://learn.microsoft.com/en-us/windows/wsl/install)

---

**Last Updated**: 2025-12-10
**Relevant For**: Modules 0–4 (all modules)
**Capstone Connection**: Provides foundation environment for all capstone development

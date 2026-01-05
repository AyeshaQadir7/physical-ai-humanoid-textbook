---
sidebar_position: 4
title: Learning Path & Prerequisites
description: Assess your readiness, choose your learning path, and set up your environment for the Physical AI robotics curriculum.
keywords: [prerequisites, learning paths, environment setup, ROS 2, self-assessment]
---

# Module 0.4: Learning Path & Prerequisites

## Introduction

Welcome to the Physical AI robotics curriculum! Before you embark on this journey, you need to ensure you have the foundational knowledge, appropriate learning path, and properly configured environment. This module is your launchpad—a comprehensive guide to self-assessment, prerequisite verification, and environment setup.

Over the next 13 weeks, you'll invest 8-10 hours per week building hands-on robotics skills. The payoff? A deep understanding of how autonomous systems perceive, think, and act in the physical world. But success requires honest self-assessment and proper preparation.

This chapter will help you:
- Evaluate your current knowledge and skills
- Understand what you need to know before starting
- Choose the learning path that matches your goals and resources
- Configure your development environment correctly
- Know where to turn when you get stuck

Let's get started.

## Learning Outcomes

By the end of this chapter, you should be able to:

1. **Assess your readiness** for the course by identifying gaps in programming, Linux, and robotics knowledge
2. **Understand prerequisite knowledge and skills** required before proceeding to Module 1
3. **Choose your learning path** (simulation-first, physical-first, or hybrid) based on your resources and goals
4. **Prepare your hardware and software environment** with Ubuntu 22.04, ROS 2 Humble, and required development tools

## Section 1: Self-Assessment

### Why Self-Assessment Matters

Honest self-assessment prevents frustration downstream. This course builds rapidly; if you're missing foundational concepts, you'll struggle with later modules. Taking 30 minutes now to identify gaps saves you hours of debugging later.

The Physical AI curriculum assumes you have certain baseline knowledge. This doesn't mean you need to be an expert—it means you should be comfortable with the basics.

### Self-Assessment Quiz

Complete the following quiz honestly. For each statement, rate yourself on a scale of 1-5:
- **1:** I've never done this
- **2:** I've done this once or twice
- **3:** I'm comfortable with this
- **4:** I do this regularly
- **5:** I could teach this to others

#### Programming Fundamentals

| Statement | Rating |
|-----------|--------|
| I can write a Python script that reads a file, processes data, and writes results | 1-5 |
| I understand what a function is and can write one with parameters and return values | 1-5 |
| I know the difference between lists, dictionaries, and tuples in Python | 1-5 |
| I can debug a Python script using print statements or a debugger | 1-5 |
| I understand what object-oriented programming is and can create a simple class | 1-5 |

**Guidance:** You should average **3 or higher** on these questions. If you're averaging below 3, review the Python fundamentals resources linked at the end of this chapter before proceeding.

#### Linux and Command Line

| Statement | Rating |
|-----------|--------|
| I can navigate the filesystem using `cd`, `ls`, and understand absolute vs. relative paths | 1-5 |
| I can create, edit, and delete files and directories from the terminal | 1-5 |
| I understand what environment variables are and how to set them | 1-5 |
| I can use a terminal text editor (vim, nano, or VS Code) comfortably | 1-5 |
| I understand what file permissions are and can use `chmod` to change them | 1-5 |

**Guidance:** You should average **3 or higher**. Linux fluency is essential; ROS 2 development happens entirely in the terminal. If you're below 3, spend a few hours on Linux fundamentals.

#### Robotics and ROS Knowledge

| Statement | Rating |
|-----------|--------|
| I understand what a robot is and the difference between simulation and hardware | 1-5 |
| I've heard of ROS (Robot Operating System) and know it's a middleware framework | 1-5 |
| I understand what "publishing" and "subscribing" mean in the context of messaging | 1-5 |
| I've run a simulator before or worked with a robotics framework | 1-5 |
| I'm comfortable with the idea of debugging hardware/software integration issues | 1-5 |

**Guidance:** You should average **2 or higher**. These topics are introduced in Module 1; don't worry if you're new to robotics. Module 1 teaches you what you need to know.

#### Debugging and Problem-Solving Mindset

| Statement | Rating |
|-----------|--------|
| When code breaks, I methodically narrow down the problem rather than guessing | 1-5 |
| I read error messages carefully and search for solutions online | 1-5 |
| I'm comfortable with the idea that things might not work on the first try | 1-5 |
| I can follow troubleshooting steps in documentation and adapt them to my situation | 1-5 |
| I'm willing to ask for help when I'm stuck (in forums, on Discord, etc.) | 1-5 |

**Guidance:** This is the most important section. You should average **3 or higher**. Robotics development is inherently experimental; the ability to systematically debug and ask good questions is your superpower.

### Interpreting Your Results

**Scenario 1: I averaged 3+ on all sections**
You're ready to proceed directly to Module 1. You have the foundational knowledge to succeed.

**Scenario 2: I averaged 2-3 on some sections**
You can start Module 1, but be prepared to refresh your knowledge in weak areas. Use the prerequisites section below to identify specific resources. Don't let gaps accumulate—address them within the first 1-2 weeks.

**Scenario 3: I averaged below 2 on multiple sections**
We recommend spending 1-2 weeks on prerequisite material before starting Module 1. This is an investment in your success, not a barrier. Everyone starts somewhere.

## Section 2: Prerequisite Knowledge and Skills

This section details what you need to know and provides resources if you're gaps.

### Programming: Python 3.10+

**What you need:** Ability to write functions, use control flow (if/else, loops), work with data structures, and understand basic OOP.

**Why it matters:** All examples in this course use Python. You'll write ROS 2 nodes, control simulated robots, and process sensor data in Python.

**Resources:**
- [Python Official Tutorial](https://docs.python.org/3/tutorial/) — Start here if you're new to Python
- [Real Python Tutorials](https://realpython.com/) — Excellent practical guides (classes, debugging, etc.)
- [Automate the Boring Stuff with Python](https://automatetheboringstuff.com/) — Free book, great for practical programming

**Estimated time:** 20-40 hours if starting from scratch; 5-10 hours if brushing up.

### Linux and Command Line

**What you need:** Comfort navigating filesystems, editing files, understanding file permissions, and using package managers.

**Why it matters:** ROS 2 development happens in Linux terminals. You'll install packages, launch nodes, and debug systems from the command line. There's no GUI to hide behind.

**Quick reference:**
```bash
# Navigation
pwd                    # Print working directory
cd /path/to/dir       # Change directory
ls -la                # List files with permissions
tree                  # Visualize directory structure (install if needed: sudo apt install tree)

# File operations
touch filename.txt     # Create empty file
mkdir dirname          # Create directory
cp source destination  # Copy file
mv source destination  # Move/rename file
rm filename            # Delete file (be careful!)
cat filename           # View file contents
nano filename          # Edit file (easy editor)

# Permissions
chmod +x script.py     # Make executable
chmod 755 script.py    # Owner: read/write/execute, others: read/execute
chown user:group file  # Change owner

# Package management (Ubuntu/Debian)
sudo apt update        # Update package list
sudo apt install pkg   # Install package
sudo apt upgrade       # Upgrade installed packages
```

**Resources:**
- [The Linux Command Line (free book)](https://linuxcommand.org/lc3_learning_the_shell.php)
- [Ubuntu Server Documentation](https://ubuntu.com/server/docs)
- Practice: Spend 1-2 hours experimenting with these commands on your system.

**Estimated time:** 10-20 hours if new to Linux; 2-5 hours to refresh.

### Git and Version Control

**What you need:** Ability to clone repositories, understand basic branching, and commit changes.

**Why it matters:** All course materials are version-controlled. You'll download starter code via git, version your own work, and collaborate using git workflows.

**Quick reference:**
```bash
# Clone a repository
git clone https://github.com/user/repo.git

# Check status
git status

# Stage and commit changes
git add .
git commit -m "descriptive message"

# Push to remote
git push origin main

# Create and switch branches
git checkout -b feature-branch
git checkout main
```

**Resources:**
- [GitHub's Git Handbook](https://guides.github.com/introduction/git-handbook/)
- [Pro Git (free book)](https://git-scm.com/book/en/v2)

**Estimated time:** 3-5 hours.

### ROS 2 Fundamentals (Conceptual)

**What you need:** Basic understanding that ROS 2 is a robotics middleware, nodes communicate via topics/services, and there are standard message types.

**Why it matters:** Module 1 teaches you ROS 2 in depth, but arriving with conceptual familiarity helps tremendously.

**Key concepts to be familiar with:**
- **Nodes:** Independent programs that perform specific tasks (image processing, motor control, etc.)
- **Topics:** Named channels for one-way communication (e.g., `/camera/image` broadcasts camera data)
- **Services:** Request/response mechanism for two-way communication (e.g., "move arm to position X")
- **Messages:** Standardized data formats (e.g., sensor readings, images, odometry)

**Resources:**
- [ROS 2 Humble Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Concepts Overview](https://docs.ros.org/en/humble/Concepts.html)
- Watch: [ROS 2 Concepts Explained (YouTube)](https://www.youtube.com/watch?v=_QwKfqPvx0k) — 15 minutes to grasp the big picture

**Estimated time:** 5-10 hours for conceptual familiarity.

### Debugging Mindset

**What you need:** Willingness to systematically isolate problems, read error messages carefully, and iterate.

**Why it matters:** Robotics is inherently experimental. Hardware behaves unpredictably. Sensors fail. Timing issues emerge. Your success depends on methodical debugging, not perfection.

**Core debugging practices:**
- **Read error messages fully** — They usually tell you exactly what went wrong
- **Isolate variables** — Change one thing at a time and observe the effect
- **Check assumptions** — Is the sensor connected? Is the ROS 2 daemon running?
- **Use tools** — `rclpy list`, `rostopic echo`, logging, and print statements
- **Ask good questions** — When seeking help, include: what you tried, what happened, and what error message you saw
- **Document solutions** — Write down fixes so you remember them next time

**Resources:**
- [Debugging Guide (this course)](./module-0-debugging/) — Dedicated module on debugging strategies
- Attitude: Expect 30-50% of development time to be debugging. This is normal.

## Section 3: Learning Paths

Not everyone approaches learning the same way. We offer three paths through this curriculum. Choose based on your resources, goals, and learning preferences.

### Path 1: Simulation-First (Recommended for Beginners)

**Overview:** You start entirely in simulation (Gazebo, ROS 2) with no physical hardware. Once you've mastered concepts, you can optionally transition to hardware later.

**Hardware requirements:**
- Laptop or desktop computer (Windows/Mac/Linux)
- 15-20 GB free disk space
- Recommended: 8+ GB RAM, modern CPU (Intel i5+ or equivalent)
- No physical robot required

**Modules:** Complete Modules 1-4 entirely in simulation

**Time commitment:** 13 weeks at 8-10 hours/week

**Pros:**
- No hardware cost (free software: ROS 2, Gazebo, Ubuntu)
- Safe to experiment and break things
- Consistent, reproducible simulation environment
- Focus on core concepts without hardware complexity
- Easier collaboration (send code, not hardware)

**Cons:**
- Simulation doesn't capture all real-world complexity (physics delays, sensor noise, etc.)
- You miss the "aha moment" of a real robot moving
- Some concepts (tuning PIDs on actual hardware, handling real sensor noise) are harder to learn in pure simulation

**Who should choose this path:**
- Learning on a tight budget
- Not sure if robotics is for you yet
- Prefer to master theory before touching hardware
- Living situation (dorms, apartments) doesn't allow hardware

**Success stories:** Many professional roboticists start in simulation to learn ROS 2 and core algorithms before moving to hardware.

### Path 2: Simulation + Jetson Nano (Recommended for Most)

**Overview:** You learn in simulation for Modules 1-2, then deploy the same code to a NVIDIA Jetson Nano running Ubuntu and ROS 2.

**Hardware requirements:**
- Everything from Path 1, plus:
- NVIDIA Jetson Nano Developer Kit (~$100 USD)
- Jetson Nano power supply (5V/4A recommended)
- Micro SD card (64 GB recommended)
- USB keyboard, mouse, monitor for initial setup (can be borrowed)
- Basic sensors/actuators: camera, servo motors, distance sensors (~$50-100)

**Modules:** Modules 1-2 in simulation; Modules 3-4 with Jetson deployment

**Time commitment:** 13 weeks at 8-10 hours/week

**Pros:**
- Keeps costs moderate (~$150-250 total for hardware)
- Learn concepts safely in simulation before hardware complexity
- Jetson Nano is powerful enough for real ML/perception workloads
- Gradual transition: simulation → actual robot
- Great learning platform; industry-relevant hardware
- Jetson skills transfer to edge computing careers

**Cons:**
- Jetson Nano has limited I/O (fewer GPIO pins than hobbyist microcontrollers)
- Requires more setup time than pure simulation
- Some real-world issues appear (power management, networking, overheating)

**Who should choose this path:**
- Want to experience real hardware but keep costs reasonable
- Ready to commit to learning robotics
- Interested in edge AI and embedded systems
- Can dedicate time to hardware setup and debugging

**Success stories:** This is the most popular path in our pilot cohorts. Students gain simulation mastery while building real embedded systems skills.

### Path 3: Physical Robot (For the Committed)

**Overview:** You deploy code to a physical mobile robot (e.g., TurtleBot 3 Burger or similar) running ROS 2.

**Hardware requirements:**
- Everything from Path 1, plus:
- Physical robot platform (~$300-500; we support TurtleBot 3 Burger)
- Laptop for remote development (SSH into robot)
- Network setup (WiFi router for robot connectivity)

**Modules:** Compressed workflow; some simulation, but primary focus on physical deployment throughout

**Time commitment:** 13 weeks at 10-12 hours/week (more complex debugging)

**Pros:**
- Learn with "real" physics immediately
- Highest motivation: your robot moves!
- Develop intuition for real-world edge cases
- Most transferable to professional robotics roles
- Strong community and support for popular platforms

**Cons:**
- Highest cost ($400-600 total)
- Hardware failures and debugging add complexity
- Slower iteration cycle (reset physical state, etc.)
- Requires safe workspace
- Some modules are harder without simulation as a teaching tool

**Who should choose this path:**
- Have budget for hardware ($400-600)
- Committed to robotics as a career or serious hobby
- Have space to safely operate a mobile robot
- Enjoy troubleshooting hardware-software integration
- Want to learn immediately with real physics

**Success stories:** Students in this path often go on to advanced robotics competitions and professional roles.

### Choosing Your Path: Decision Matrix

| Factor | Path 1: Sim-Only | Path 2: Sim + Jetson | Path 3: Physical Robot |
|--------|------------------|----------------------|----------------------|
| **Cost** | Free | ~$150-250 | ~$400-600 |
| **Setup time** | 2-4 hours | 6-10 hours | 12-16 hours |
| **Safety concerns** | None | Minimal | Moderate (moving robot) |
| **Real-world complexity** | Low | Medium | High |
| **Debugging time** | Low | Medium | High |
| **Motivation** | Moderate | High | Very High |
| **Career relevance** | Intermediate | High | Very High |

**Our recommendation:** Start with Path 2 (Sim + Jetson). It balances learning velocity, cost, and real-world exposure. You can always downgrade to Path 1 if hardware proves challenging, or upgrade to Path 3 later.

## Section 4: Environment Setup

This section walks you through installing Ubuntu 22.04 and ROS 2 Humble. Follow these steps carefully; a correct environment setup prevents weeks of debugging frustration.

### Prerequisites Before You Start

- **Laptop/desktop with 20+ GB free disk space**
- **Internet connection** (ROS 2 downloads ~2-3 GB)
- **Estimated time:** 1-2 hours for first-time setup
- **Willingness to restart your computer** during Ubuntu installation (if dual-booting)

### Step 1: Install Ubuntu 22.04 LTS

#### Option 1A: Fresh Installation (Recommended if no OS preference)

If you don't have an existing OS you need to preserve, a fresh Ubuntu installation is cleanest.

**Hardware needed:**
- USB drive (8+ GB)
- Another computer to create the bootable USB

**Process:**

1. Download Ubuntu 22.04 LTS ISO from [ubuntu.com/download/desktop](https://ubuntu.com/download/desktop)

2. Create a bootable USB using:
   - **Windows/Mac:** [Balena Etcher](https://www.balena.io/etcher/) — drag and drop, very user-friendly
   - **Linux:**
   ```bash
   lsblk                                    # Find USB device (e.g., /dev/sdc)
   sudo dd if=~/Downloads/ubuntu-22.04.iso of=/dev/sdc bs=4M status=progress
   sudo sync
   ```

3. Boot from USB (restart computer, press F12/F2/Delete during boot to enter BIOS/UEFI)

4. Follow the Ubuntu installer:
   - Select "Install Ubuntu"
   - Choose your language and keyboard layout
   - Under "Installation type," select "Erase disk and install Ubuntu" (this erases the disk—make sure you want this)
   - Create user account (remember your username and password)
   - Wait for installation to complete (~10-15 minutes)
   - Restart when prompted

5. After restart, open a terminal and verify:
   ```bash
   lsb_release -a
   # Should show: Ubuntu 22.04.x LTS
   ```

#### Option 1B: Virtual Machine (If You Want to Keep Your OS)

Use VirtualBox or VMware to run Ubuntu in a virtual machine within your existing OS.

**Steps:**
1. Download [VirtualBox](https://www.virtualbox.org/wiki/Downloads) (free)
2. Create a new VM: allocate at least 4 CPU cores, 8 GB RAM, 50 GB disk
3. Attach Ubuntu 22.04 ISO and boot the VM
4. Follow the Ubuntu installer steps above (inside the VM)

**Trade-off:** VMs are slower than native installation, but safer if you're not ready to commit to Ubuntu.

#### Option 1C: Dual Boot (If You Have Multiple OSes)

If you need to keep Windows or macOS, install Ubuntu alongside them. This is more complex; consult [Ubuntu Dual Boot Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview).

**Recommendation:** If you're new to Linux, go with Option 1A (fresh install) or 1B (VM). Dual boot is powerful but risky if you make mistakes.

### Step 2: Update Your System

After Ubuntu is installed and running, open a terminal and update packages:

```bash
sudo apt update
sudo apt upgrade
sudo apt autoremove
```

This ensures you have the latest security patches and software versions.

### Step 3: Install ROS 2 Humble

#### Step 3.1: Set Up ROS 2 Repositories

```bash
# Set locale and ensure UTF-8 support
locale  # verify UTF-8 is available
export LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 GPG key
sudo apt install curl gnupg2 lsb-release ubuntu-keyring
curl -sSL https://repo.ros.org/ros.key | sudo apt-key add -

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index
sudo apt update
```

#### Step 3.2: Install ROS 2 Humble

```bash
sudo apt install ros-humble-desktop
```

This installs:
- ROS 2 core libraries
- Gazebo simulator
- Visualization tools (RViz)
- Common message and service definitions

**Size:** ~5-6 GB. Be patient; this takes 10-15 minutes.

#### Step 3.3: Set Up Your Shell Environment

Add ROS 2 setup to your `.bashrc` file so it's automatically sourced when you open a terminal:

```bash
# Add this line to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Reload your shell
source ~/.bashrc
```

Verify:
```bash
echo $ROS_DISTRO
# Should output: humble
```

#### Step 3.4: Install Development Tools

Install essential tools for building ROS 2 packages:

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep2
sudo apt install git

# Initialize rosdep (ROS dependency manager)
sudo rosdep init
rosdep update
```

### Step 4: Verify Your Installation

Run a few tests to confirm everything works:

```bash
# Test 1: ROS 2 is in your PATH
which ros2
# Should return: /opt/ros/humble/bin/ros2

# Test 2: Start the ROS 2 daemon (required for all ROS 2 commands)
ros2 daemon start

# Test 3: List available ROS 2 commands
ros2 --help
# Should show available subcommands

# Test 4: Run a simple talker/listener demo
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2 (new terminal)
ros2 run demo_nodes_cpp listener
# You should see messages flowing between talker and listener
```

If all four tests pass, your ROS 2 installation is working correctly!

### Step 5: Set Up Your Development Workspace

Create a directory structure for your ROS 2 projects:

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace (initially empty)
colcon build

# Add workspace setup to bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Your workspace is now ready for development. In Module 1, you'll create ROS 2 packages inside `~/ros2_ws/src/`.

### Step 6: Install Optional but Recommended Tools

```bash
# Visual Studio Code with ROS 2 extensions
sudo apt install code
# Install extensions: C/C++, Python, ROS (ms-iot.vscode-ros)

# Git (already installed, but set up your user)
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Python package manager
sudo apt install python3-pip
pip install --upgrade pip

# Useful utilities
sudo apt install tmux              # Terminal multiplexer
sudo apt install htop              # System monitor
sudo apt install build-essential   # Compiler toolchain
```

## Section 5: Troubleshooting Common Setup Issues

Even with careful installation, issues can arise. This section covers the most common problems and their solutions.

### Issue: "Command 'ros2' not found"

**Problem:** After installing ROS 2, the `ros2` command doesn't work.

**Diagnosis:**
```bash
# Check if setup script was sourced
echo $ROS_DISTRO

# Check if ROS 2 is installed
ls /opt/ros/humble/bin/ros2
```

**Solution:**
```bash
# If ROS_DISTRO is empty, you haven't sourced the setup script
source /opt/ros/humble/setup.bash

# Make it permanent by adding to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Issue: "ERROR: Unable to locate package ros-humble-desktop"

**Problem:** The ROS 2 repository isn't added correctly.

**Diagnosis:**
```bash
cat /etc/apt/sources.list.d/ros2.list
# Check if the line looks correct
```

**Solution:**
```bash
# Remove broken repository
sudo rm /etc/apt/sources.list.d/ros2.list

# Follow Step 3.1 again carefully, exactly as written
# Then: sudo apt update
# Then: sudo apt install ros-humble-desktop
```

### Issue: Talker/Listener Demo Doesn't Work

**Problem:** Running the demo_nodes_cpp talker/listener produces no output or connection errors.

**Diagnosis:**
```bash
# Check if ROS 2 daemon is running
ps aux | grep rosmaster
# or for ROS 2:
ros2 daemon list

# Check ROS 2 graph
ros2 topic list
```

**Solution:**
```bash
# Start the daemon explicitly
ros2 daemon start

# If that doesn't work, check your environment
source /opt/ros/humble/setup.bash

# Retry the demo
ros2 run demo_nodes_cpp talker  # Terminal 1
ros2 run demo_nodes_cpp listener # Terminal 2
```

### Issue: Workspace Build Fails with "ModuleNotFoundError"

**Problem:** After `colcon build`, Python imports fail.

**Diagnosis:**
```bash
# Check if your workspace setup is sourced
echo $AMENT_PREFIX_PATH
```

**Solution:**
```bash
# Source your workspace setup after every colcon build
source ~/ros2_ws/install/setup.bash

# Make it automatic
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Issue: "Permission denied" When Running ROS 2 Commands

**Problem:** `Permission denied` errors when launching nodes or accessing topics.

**Diagnosis:**
```bash
# Check file permissions
ls -la ~/.ros/
# Check device permissions (if using hardware)
ls -la /dev/tty*
```

**Solution:**
```bash
# Usually not necessary, but if needed:
sudo chown -R $USER:$USER ~/.ros/

# For device access, add your user to dialout group
sudo usermod -aG dialout $USER
# Then log out and log back in for changes to take effect
```

### Issue: Ubuntu is Too Slow (VM Only)

**Problem:** Ubuntu in VirtualBox/VMware feels laggy.

**Diagnosis:** Check CPU and RAM allocation to VM.

**Solution:**
```bash
# In VirtualBox:
# Settings → System → Processor: allocate as many CPU cores as your host has
# Settings → System → Memory: allocate at least 6-8 GB RAM
# Settings → Display: increase Video Memory to 128 MB
# Enable 3D Acceleration if available
```

### Issue: Gazebo Simulator Won't Launch

**Problem:** Running `gazebo` command produces errors or nothing appears.

**Diagnosis:**
```bash
# Test if Gazebo is installed
which gazebo
# Test Gazebo directly
gazebo --version
```

**Solution:**
```bash
# Gazebo should have been installed with ros-humble-desktop
# If not, install it explicitly
sudo apt install ros-humble-gazebo-ros

# If that fails, try updating
sudo apt update
sudo apt upgrade gazebo*

# If still failing, check available GPU
glxinfo | grep "OpenGL"
# Gazebo needs OpenGL support; if minimal/none, it won't launch
```

### Issue: "Conflicting Package Versions"

**Problem:** `apt` complains about conflicting versions when installing packages.

**Diagnosis:**
```bash
apt search ros-humble | grep conflict
```

**Solution:**
```bash
# Update everything
sudo apt update
sudo apt upgrade

# If there are still conflicts, check what you're trying to install
sudo apt install --no-install-recommends ros-humble-desktop

# As a last resort, clean and reinstall
sudo apt clean
sudo apt autoclean
sudo apt autoremove
```

### Still Stuck?

If your issue isn't listed here:

1. **Search the error message** — Copy the exact error into Google or GitHub issues
2. **Check ROS 2 Humble Docs** — [docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
3. **Ask on ROS Discourse** — [discourse.ros.org](https://discourse.ros.org)
4. **Join course Discord** — Post your setup issue with: OS version, ROS 2 version, and full error message
5. **Reinstall if all else fails** — Sometimes a clean installation resolves subtle environment issues

Remember: You are not alone. Every roboticist has been stuck on setup. Patience and systematic debugging will get you through.

## Section 6: Time Commitment and Course Structure

Understanding the time investment and overall course structure sets realistic expectations.

### Weekly Time Commitment

This course requires **8-10 hours per week** for 13 weeks. Here's a typical breakdown:

| Activity | Hours |
|----------|-------|
| Watching lecture videos | 1-2 |
| Reading this textbook | 1-2 |
| Coding and experimentation | 3-4 |
| Debugging and troubleshooting | 1-2 |
| Quizzes and assessments | 0.5-1 |
| **Total per week** | **8-10** |

### Course Structure

The curriculum spans 4 main modules (Modules 1-4) over 13 weeks:

| Module | Title | Duration | Focus |
|--------|-------|----------|-------|
| **1** | ROS 2 Fundamentals | Weeks 1-3 | Nodes, topics, services; building your first ROS 2 package |
| **2** | Perception: Sensors and Data | Weeks 4-6 | Cameras, LIDAR, odometry; processing sensor data |
| **3** | Planning and Navigation | Weeks 7-9 | Path planning, motion planning, autonomous navigation |
| **4** | Integration and Deployment | Weeks 10-13 | Deploy complete system; handle real-world edge cases; capstone project |

Each module includes:
- **Conceptual readings** (this textbook)
- **Hands-on coding projects** (simulation or hardware)
- **Debugging challenges** (intentional bugs to find and fix)
- **Quizzes and self-assessment** (track your learning)
- **Peer collaboration** (optional; Discord community)

### Pacing Tips

- **Don't fall behind** — ROS 2 concepts build on each other. Missing Week 2 makes Week 3 harder.
- **Allocate debugging time** — If something doesn't work, you might spend 1-2 hours debugging. This is normal, not a sign you're slow.
- **Batch similar tasks** — Code in focused 2-3 hour blocks, not scattered minutes throughout the day. Deep work matters.
- **Use the Discord** — Stuck on something for >30 minutes? Ask the community. You might save hours.
- **Build real projects** — The best learning happens when you're solving real problems, not abstract exercises.

## Section 7: Support Resources and Getting Help

Learning robotics is challenging. Knowing where to turn when you're stuck is critical.

### Online Resources

| Resource | Best For | URL |
|----------|----------|-----|
| **ROS 2 Official Docs** | API reference, architecture | [docs.ros.org/en/humble/](https://docs.ros.org/en/humble/) |
| **ROS 2 Humble Cheat Sheet** | Quick command lookup | [roboflow.com/blog/ros2-cheat-sheet](https://blog.roboflow.com/ros2-cheat-sheet/) |
| **ROS Discourse** | Asking questions, community | [discourse.ros.org](https://discourse.ros.org) |
| **Stack Overflow** | Debugging specific errors | [stackoverflow.com/questions/tagged/ros](https://stackoverflow.com/questions/tagged/ros) |
| **GitHub Issues** | Reporting bugs in ROS 2 | [github.com/ros2/ros2](https://github.com/ros2/ros2) |
| **Gazebo Docs** | Simulator issues | [gazebosim.org](https://gazebosim.org) |

### Course-Specific Support

- **Discord Community** — Real-time chat with instructors and peers
- **Office Hours** — Weekly sessions (times TBD) for live troubleshooting
- **Assignment Feedback** — Submit code for review; get detailed feedback
- **Peer Collaboration** — Optional: form study groups and debug together

### How to Ask Good Questions

When you're stuck, asking the right question matters. Here's the formula:

1. **Describe what you were trying to do**
   > "I was trying to make a ROS 2 node that subscribes to the `/camera/image` topic and prints the message type."

2. **Describe what happened**
   > "When I run the node, I get a ModuleNotFoundError."

3. **Include the full error message**
   > "```Traceback (most recent call last): ... ModuleNotFoundError: No module named 'sensor_msgs'```"

4. **Describe what you've already tried**
   > "I've tried: `pip install sensor-msgs` and sourcing my workspace setup again."

5. **Include your system info**
   > "I'm on Ubuntu 22.04, ROS 2 Humble, Python 3.10."

**Good question example:**
> "I'm trying to subscribe to `/camera/image` in my node (code: [gist link]). I get `ModuleNotFoundError: No module named 'sensor_msgs'`. I've sourced my workspace and reinstalled ROS 2. What am I missing?"

This question shows effort, includes specifics, and is answerable.

### Debugging Checklist

Before asking for help, try this checklist:

- [ ] Have you read the full error message? (Copy it into Google)
- [ ] Have you checked the relevant documentation?
- [ ] Have you tried restarting (your terminal, your computer, your workspace)?
- [ ] Have you re-sourced your setup scripts? (`source ~/.bashrc`)
- [ ] Have you tried the simplest possible version of what you're doing?
- [ ] Have you checked if someone else has reported this issue? (GitHub issues, Stack Overflow)
- [ ] Have you tried searching with different keywords?

Often, one of these steps reveals the problem. If you've done all of them and still stuck, ask the community.

## Section 8: Bridge to Module 1

You've now prepared yourself—you've self-assessed, verified prerequisites, chosen your learning path, and set up your environment. You're ready for Module 1.

### What Comes Next: ROS 2 Fundamentals

Module 1 introduces the core concepts you'll use throughout the curriculum:

- **Nodes:** How ROS 2 programs are structured
- **Topics and Services:** How ROS 2 programs communicate
- **The ROS 2 Graph:** Visualizing your system architecture
- **Writing your first ROS 2 package:** Hands-on coding with Python
- **Debugging ROS 2 systems:** Tools and techniques you'll use constantly

You'll come out of Module 1 comfortable with ROS 2 basics and ready to tackle perception and planning in Modules 2-4.

### Prerequisites for Module 1

Before you start Module 1, make sure you have:

- ✓ Completed this module (Module 0.4)
- ✓ Self-assessed and identified any knowledge gaps
- ✓ Installed Ubuntu 22.04 and ROS 2 Humble (verified with the talker/listener demo)
- ✓ Created your ROS 2 workspace (`~/ros2_ws/`)
- ✓ Reviewed the prerequisite resources for areas where you scored below 3

If you've completed all of these, you're ready to move forward.

### Recommended Pre-Reading for Module 1

To get a head start on Module 1, familiarize yourself with these concepts:

1. **ROS 2 Architecture** — Read [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts.html) (15 minutes)
2. **Python Classes** — You'll write classes in Module 1; refresh your OOP knowledge (1 hour)
3. **Gazebo Basics** — Watch a 5-minute overview of Gazebo simulator on YouTube
4. **Terminal Skills** — Practice navigating directories and editing files in your preferred text editor (30 minutes)

These won't be required, but they'll make Module 1 flow more smoothly.

## Summary

In this chapter, you've:

1. **Self-assessed** your readiness using an honest rubric across programming, Linux, and robotics domains
2. **Identified prerequisites** and located resources to fill any gaps
3. **Chosen your learning path** based on your resources, goals, and learning style
4. **Installed your environment:** Ubuntu 22.04, ROS 2 Humble, and development tools
5. **Learned troubleshooting strategies** for common setup issues
6. **Understood the course structure** and time commitment (13 weeks, 8-10 hours/week)
7. **Located support resources** and learned how to ask good questions

You now have everything you need to succeed in this curriculum. Module 1 awaits.

---

## Chapter Quiz

Test your understanding of the material in this chapter. Each question maps to one or more learning outcomes. Try to answer without looking back at the text first; then review to reinforce learning.

### Quiz Questions

**Question 1: Self-Assessment Interpretation**

You completed the self-assessment quiz and found:
- Programming: averaged 2.5/5
- Linux: averaged 3.5/5
- Robotics: averaged 2.0/5
- Debugging: averaged 4.5/5

What should you do before starting Module 1?

A) Start Module 1 immediately; you can catch up as needed
B) Spend time on programming and robotics fundamentals; Linux is strong enough
C) Pause the course until you're a Python expert
D) Your debugging skills are strong, which matters most; review programming fundamentals and robotics concepts before Module 1

**Correct answer:** D

**Explanation:** Your strong debugging mindset (4.5/5) is actually the most valuable skill for robotics. Your Linux foundation (3.5/5) is solid. The real gaps are in programming (2.5/5) and robotics familiarity (2.0/5). Spend 1-2 weeks brushing up on Python functions, classes, and ROS 2 basics using the resources provided. You don't need to be an expert—just solid enough to follow along in Module 1.

---

**Question 2: Learning Path Decision**

You have:
- A laptop with 8 GB RAM and 50 GB free disk space
- A budget of ~$200
- Interest in edge AI and autonomous systems
- 10 hours per week available
- No physical robot yet

Which learning path is best for you?

A) Path 1 (Simulation-Only): No hardware needed; learn safely
B) Path 2 (Simulation + Jetson): Combine conceptual learning with edge AI hardware
C) Path 3 (Physical Robot): Buy a TurtleBot 3 now; learn with real hardware
D) Pause until you have more budget for Path 3

**Correct answer:** B

**Explanation:** Path 2 is ideal for you. Your budget ($200) covers a Jetson Nano and basic sensors. Your interest in edge AI aligns perfectly with Jetson, an industry-standard edge AI platform. Your laptop specs (8 GB RAM, 50 GB space) are sufficient for simulation. Path 1 would work but misses the hardware exposure you're interested in. Path 3 exceeds your budget and is unnecessary until you've proven yourself in simulation. Path 2 gives you the best balance.

---

**Question 3: Environment Setup Troubleshooting**

You've installed Ubuntu 22.04 and ROS 2 Humble. When you run `ros2 daemon start`, nothing happens. When you then run `ros2 topic list`, you get the error "command not found: ros2".

What is the most likely problem, and how do you fix it?

A) ROS 2 wasn't installed correctly; reinstall from scratch
B) Your shell hasn't sourced the ROS 2 setup script; run `source /opt/ros/humble/setup.bash`
C) The daemon didn't start; restart your computer
D) You're missing Python 3.10; upgrade Python

**Correct answer:** B

**Explanation:** The "command not found" error indicates ROS 2 isn't in your PATH, which means your shell hasn't sourced `/opt/ros/humble/setup.bash`. This is the single most common setup issue. Reinstalling (A) is overkill and won't fix it. Restarting your computer (C) won't help unless you add the source command to your `.bashrc`. Python version (D) isn't the issue here. **Fix:** Run `source /opt/ros/humble/setup.bash` immediately, then add it to your `.bashrc` so it's automatic. Verify with `echo $ROS_DISTRO`.

---

**Question 4: Time Management and Pacing**

You're in Week 2 of Module 1. You're stuck on a ROS 2 node implementation and have already spent 3 hours debugging without progress. Your 10-hour weekly time budget is running low, and you're worried about falling behind.

What should you do?

A) Keep debugging alone for another 2 hours; asking for help shows weakness
B) Skip the problematic node and move on to the next topic
C) Ask for help on Discord immediately; post your code, error, and what you've tried
D) Drop out of the course; you're not good enough for robotics

**Correct answer:** C

**Explanation:** After 3 hours of debugging without progress, asking for help is the right call. Debugging is normal and expected in robotics; asking good questions is a core skill. Your community (Discord, office hours, etc.) exists specifically to unblock these situations. Posting your code, error, and troubleshooting steps shows professionalism and makes it easy for someone to help. You'll likely get unstuck in 10-20 minutes with fresh eyes. (A) perpetuates the false belief that asking for help is weak; professional engineers ask for help constantly. (B) creates knowledge gaps that hurt later modules. (D) is unnecessarily harsh; getting stuck is part of learning.

---

**Question 5: Prerequisites and Real Knowledge**

Looking at the prerequisite resources, which of the following is most critical to review before starting Module 1?

A) Python decorators (advanced Python feature)
B) How decorators work in ROS 2 services
C) Python functions, data structures (lists, dicts), and basic OOP (classes, methods)
D) Advanced Linux file permissions (chmod, ownership)

**Correct answer:** C

**Explanation:** Module 1 requires you to write ROS 2 nodes as Python classes with methods, understand data structures for messages, and write functions. These are core programming fundamentals you'll use constantly. Decorators (A) and advanced permissions (D) are nice-to-haves but not critical for getting started. The ROS 2 service decorator specifics (B) are taught in Module 1; you don't need to know them beforehand. Prioritize solid fundamentals over advanced topics.

---

### Challenge Question (Advanced)

You want to eventually deploy code to a physical robot (Path 3) but are starting with a tight budget. You've chosen Path 2 (Simulation + Jetson).

How would you structure your 13 weeks to maximize the reusability of code from simulation to physical hardware, minimizing refactoring work later?

**Hint:** Think about how closely your simulation matches the physical setup you'll eventually use.

**Suggested answer:**
- Weeks 1-4 (Modules 1-2): Learn ROS 2 and sensors in simulation, but **design your ROS 2 nodes to be hardware-agnostic** (use standard message types, avoid hardcoding paths)
- Weeks 5-8: Run the same nodes on Jetson Nano in simulation mode first, verifying they work on actual hardware
- Weeks 9-13: Gradually swap out simulated sensors with real ones, verifying code works without refactoring
- This approach ensures your code transfers directly to a physical robot later with minimal changes

---

## Conclusion

You now have the knowledge, environment, and community to start this robotics journey. Module 1 begins in the next chapter.

Welcome aboard. Let's build robots.

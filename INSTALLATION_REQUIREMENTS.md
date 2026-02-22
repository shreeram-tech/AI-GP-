# Drone Racing Simulation - Complete Installation & Requirements Guide

**Last Updated**: February 22, 2026

This document comprehensively lists all files, dependencies, and system configurations required for proper functioning of the autonomous drone racing simulation system.

---

## Table of Contents

1. [System Requirements](#system-requirements)
2. [OS & Environment Setup](#os--environment-setup)
3. [Package Dependencies](#package-dependencies)
4. [Python Dependencies](#python-dependencies)
5. [ROS2 & Gazebo Setup](#ros2--gazebo-setup)
6. [Project Files Structure](#project-files-structure)
7. [Build & Installation Steps](#build--installation-steps)
8. [Environment Variables](#environment-variables)
9. [Verification Checklist](#verification-checklist)
10. [Troubleshooting](#troubleshooting)

---

## System Requirements

### Hardware
- **CPU**: Quad-core or better (Intel i7/AMD Ryzen 5 or equivalent)
- **RAM**: Minimum 8 GB (16 GB recommended for smooth simulation)
- **Storage**: 20-30 GB available space (for ROS2, Gazebo, and simulation data)
- **GPU**: NVIDIA/AMD GPU recommended for physics simulation (optional but accelerates Gazebo)

### Software
- **OS**: Ubuntu 22.04 LTS (primary), Ubuntu 20.04 LTS (with adjustments)
- **Kernel**: Linux 5.13+ (standard Ubuntu 22.04 kernel)
- **Python**: 3.10 or 3.11 or 3.12
- **GCC/G++**: 11.0 or later
- **CMake**: 3.16 or later

---

## OS & Environment Setup

### Initial System Update

```bash
sudo apt update
sudo apt upgrade -y
```

### Required Build Tools

```bash
sudo apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    gnupg \
    lsb-release \
    ca-certificates \
    apt-transport-https \
    software-properties-common
```

### Python Development Tools

```bash
sudo apt install -y \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-apt
```

---

## Package Dependencies

### ROS2 System Packages

| Package | Version | Purpose |
|---------|---------|---------|
| ros-humble-desktop-full | Latest | ROS2 Humble complete installation |
| ros-humble-gazebo-ros-pkgs | Latest | ROS2-Gazebo integration |
| ros-humble-gazebo-plugins | Latest | Gazebo sensor/physics plugins |
| ros-humble-launch* | Latest | ROS2 launch framework |
| ros-humble-sensor-msgs | Latest | Sensor message definitions |
| ros-humble-std-msgs | Latest | Standard ROS2 messages |
| ros-humble-geometry-msgs | Latest | Geometric transformation messages |
| ros-humble-visualization-msgs | Latest | RViz visualization messages |
| ros-humble-tf2* | Latest | Transformation library |
| ros-humble-rviz2 | Latest | 3D visualization tool |

### Gazebo Packages

| Package | Version | Purpose |
|---------|---------|---------|
| ignition-gazebo-dev | 6.x or 7.x | Physics simulation engine |
| ignition-transport11-dev | Latest | Message transport |
| ignition-msgs8-dev | Latest | Ignition message definitions |
| libgz-physics* | Latest | Physics plugin library |

### System Libraries

| Package | Purpose |
|---------|---------|
| libxml2-dev | XML parsing |
| libtinyxml-dev | XML processing for URDF |
| libtinyxml2-dev | XML processing for SDF |
| libssl-dev | SSL/TLS support |
| libboost-all-dev | C++ boost libraries |
| libtiff-dev | Image processing |
| libqt5-dev | Qt5 GUI framework |
| libfreetype-dev | Font rendering |

### Installation Command

```bash
# Install ROS2 Humble key and repository
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS2
sudo apt update
sudo apt install -y ros-humble-desktop

# Install Gazebo Ignition
sudo apt install -y ignition-gazebo-dev

# Install required libraries
sudo apt install -y \
    libxml2-dev libtinyxml-dev libtinyxml2-dev \
    libssl-dev libboost-all-dev libtiff-dev \
    libqt5-dev libfreetype-dev
```

---

## Python Dependencies

### Core Python Packages (from requirements.txt)

| Package | Version | Purpose |
|---------|---------|---------|
| numpy | >=1.19.0 | Numerical computation |
| matplotlib | >=3.3.0 | Plotting and visualization |
| pandas | >=1.1.0 | Data analysis and CSV handling |
| scipy | >=1.5.0 | Scientific computing |

### ROS2 Python Packages

| Package | Purpose |
|---------|---------|
| rclpy | ROS2 Python client library |
| geometry2 | Transformation library (tf2) |
| sensor-msgs | Sensor message types |
| std-msgs | Standard message types |
| vision-msgs | Vision message definitions |
| tf2-geometry-msgs | TF2 geometry message support |
| tf2-py | Python TF2 bindings |

### Development & Testing Packages

| Package | Purpose |
|---------|---------|
| pytest | Unit testing framework |
| pytest-cov | Code coverage for pytest |
| flake8 | Python code style checker |
| black | Python code formatter |

### Installation Command

```bash
# Install Python packages
pip install numpy scipy matplotlib pandas

# Install ROS2 Python packages
sudo apt install -y \
    python3-rclpy \
    python3-geometry2 \
    python3-sensor-msgs \
    python3-std-msgs \
    python3-vision-msgs \
    python3-tf2-geometry-msgs \
    python3-tf2 \
    ros-humble-rclpy \
    ros-humble-std-msgs

# Install development packages
pip install pytest pytest-cov flake8 black
```

---

## ROS2 & Gazebo Setup

### ROS2 Humble Installation

```bash
# Setup ROS2 apt repository
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble full desktop
sudo apt update
sudo apt install -y ros-humble-desktop-full

# Install additional ROS2 tools
sudo apt install -y \
    ros-humble-colcon-common-extensions \
    ros-dev-tools
```

### Gazebo Ignition Installation

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo
sudo apt update
sudo apt install -y ignition-gazebo-dev ignition-gazebo

# Install Gazebo-ROS2 plugins
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### Colcon Build Tool

```bash
# Install colcon
sudo apt install -y python3-colcon-common-extensions

# Verify installation
colcon --version
```

---

## Project Files Structure

### Essential Source Files

#### Phase 1: Autonomy Stack (`/src/drone_racing_stack/`)

Required Python modules:
- `src/autonomy.py` - Main 50Hz control loop, state machine
- `src/control.py` - Cascaded PID controllers (position, velocity, attitude)
- `src/planning.py` - Pure Pursuit trajectory planning, gate sequencing
- `src/state_estimation.py` - EMA filtering, acceleration estimation
- `src/perception.py` - Sensor data validation and packaging
- `src/types.py` - Vector3 mathematical operations
- `src/__init__.py` - Package initialization

Configuration files:
- `config/default_config.py` - Default PID parameters and strategy settings
- `requirements.txt` - Python package dependencies
- `setup.py` - Package setup configuration

#### Phase 2: Simulation (`/drone_racing_sim/`)

ROS2 Package files:
- `src/drone_racing_core/package.xml` - ROS2 package manifest
- `src/drone_racing_core/setup.py` - ROS2 package setup

Core simulation modules:
- `src/drone_racing_core/drone_racing_core/gate_spawner.py` - Programmatic gate creation
- `src/drone_racing_core/drone_racing_core/autonomy_bridge.py` - Gazebo-Autonomy integration
- `src/drone_racing_core/drone_racing_core/gate_validator.py` - Sequence tracking, lap timing
- `src/drone_racing_core/drone_racing_core/visualization.py` - RViz markers, trajectory path
- `src/drone_racing_core/drone_racing_core/metrics_logger.py` - CSV/JSON data logging
- `src/drone_racing_core/drone_racing_core/simulator_node.py` - ROS2 simulator node

#### Simulation Assets

Robot Description:
- `urdf/quadrotor.urdf.xacro` - XACRO drone model (1.2kg quadrotor)
  - Mass: 1.2 kg
  - Arm length: 0.23 m
  - Propeller diameter: 0.25 m
  - Sensors: 6-DOF IMU, odometry

Environment Models:
- `models/gate/model.sdf` - Racing gate (5.4m tall, collision-enabled)

Simulation World:
- `worlds/racing_track.world` - Gazebo Ignition world file
  - ODE physics engine (0.001s timestep)
  - Ground plane with checkerboard texture
  - Sky dome (gray gradient)
  - Point light sources
  - Plugin configuration

Visualization:
- `configs/rviz_config.rviz` - RViz2 pre-configured display (gates, trajectory, markers)

Launch Files:
- `launches/racing_sim.launch.py` - ROS2 launch file for complete system

### Test Files

Unit & Integration Tests:
- `test/test_autonomy.py` - Autonomy module tests
- `tests/test_autonomy.py` - Additional autonomy tests

### Documentation Files

- `README.md` - Main project overview
- `QUICKSTART.md` - 5-minute quick start guide
- `BUILD.md` - Detailed build instructions
- `IMPLEMENTATION_SUMMARY.md` - Technical implementation details
- `COMPLETE_SYSTEM_SUMMARY.md` - Complete system architecture

---

## Build & Installation Steps

### Step 1: Install System Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2, Gazebo, and build tools (see section above)
source /opt/ros/humble/setup.bash
```

### Step 2: Install Python Dependencies

```bash
# Navigate to workspace
cd ~/aigp

# Install pip packages
pip install -r src/drone_racing_stack/requirements.txt

# Install additional ROS2 Python packages
pip install rclpy gazebo_ros tf2_geometry_msgs tf2-py vision-msgs
```

### Step 3: Build with Colcon

```bash
cd ~/aigp

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select drone_racing_core

# Build with logging (if needed)
colcon build --packages-select drone_racing_core --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Step 4: Source Setup Files

```bash
# Source the workspace setup
source ~/aigp/install/setup.bash

# Verify package is installed
ros2 pkg list | grep drone_racing
```

### Step 5: Verify Installation

```bash
# Check ROS2
ros2 --version

# Check Gazebo
gazebo --version

# Check Python packages
python3 -c "import rclpy; import numpy; import gazebo_msgs; print('All imports OK')"
```

---

## Environment Variables

### Critical Environment Variables

```bash
# ROS2 setup (add to ~/.bashrc)
source /opt/ros/humble/setup.bash

# Workspace setup (add to ~/.bashrc)
source ~/aigp/install/setup.bash

# Gazebo model path (add to ~/.bashrc)
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:~/aigp/drone_racing_sim/models"
export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH}:~/aigp/drone_racing_sim"

# ROS domain (optional, for multi-machine setup)
export ROS_DOMAIN_ID=0

# Gazebo verbose mode (for debugging)
export GAZEBO_VERBOSE=1  # Only when needed
```

### Adding to ~/.bashrc

```bash
cat >> ~/.bashrc << 'EOF'
# ROS2 Humble
source /opt/ros/humble/setup.bash

# Workspace setup
source ~/aigp/install/setup.bash

# Gazebo paths
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:~/aigp/drone_racing_sim/models"
export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH}:~/aigp/drone_racing_sim"
EOF

# Reload bashrc
source ~/.bashrc
```

---

## Verification Checklist

Use this checklist to verify all components are properly installed:

### System & Tools

- [ ] Ubuntu 22.04 LTS or compatible
- [ ] Python 3.10+ installed: `python3 --version`
- [ ] CMake 3.16+: `cmake --version`
- [ ] Git: `git --version`
- [ ] Build tools: `gcc --version`, `g++ --version`

### ROS2 Installation

- [ ] ROS2 installed: `echo $ROS_DISTRO` (should output `humble`)
- [ ] ROS2 commands available: `which ros2`
- [ ] ROS2 packages found: `ros2 pkg list | head`
- [ ] Test ROS2 communication:
  ```bash
  # Terminal 1
  source /opt/ros/humble/setup.bash
  ros2 run demo_nodes_cpp talker
  
  # Terminal 2
  source /opt/ros/humble/setup.bash
  ros2 run demo_nodes_py listener
  # Should receive messages
  ```

### Gazebo Installation

- [ ] Gazebo installed: `gazebo --version`
- [ ] Ignition Gazebo: `ign gazebo --version`
- [ ] Test Gazebo: `gazebo` (should open window)
- [ ] Gazebo-ROS2 integration: `ros2 pkg list | grep gazebo`

### Python Packages

- [ ] NumPy: `python3 -c "import numpy; print(numpy.__version__)"`
- [ ] SciPy: `python3 -c "import scipy; print(scipy.__version__)"`
- [ ] Pandas: `python3 -c "import pandas; print(pandas.__version__)"`
- [ ] Matplotlib: `python3 -c "import matplotlib; print(matplotlib.__version__)"`
- [ ] rclpy: `python3 -c "import rclpy; print(rclpy.__version__)"`

### Project Build

- [ ] Workspace exists: `ls -la ~/aigp/`
- [ ] Source files present: `ls ~/aigp/src/`
- [ ] Colcon build successful: `ls -la ~/aigp/build/`
- [ ] Install artifacts: `ls -la ~/aigp/install/`
- [ ] Package discoverable: `ros2 pkg list | grep drone_racing`

### Environment Setup

- [ ] ROS2 sourced: `echo $ROS_PACKAGE_PATH` (should contain paths)
- [ ] Workspace sourced: `which simulator_node` (should show path)
- [ ] Gazebo models path: `echo $GAZEBO_MODEL_PATH` (should include ~/aigp/drone_racing_sim/models)

### Simulation Specific

- [ ] URDF file exists: `cat ~/aigp/drone_racing_sim/urdf/quadrotor.urdf.xacro`
- [ ] Gate model exists: `cat ~/aigp/drone_racing_sim/models/gate/model.sdf`
- [ ] World file exists: `cat ~/aigp/drone_racing_sim/worlds/racing_track.world`
- [ ] Launch file exists: `cat ~/aigp/drone_racing_sim/launches/racing_sim.launch.py`
- [ ] RViz config exists: `cat ~/aigp/drone_racing_sim/configs/rviz_config.rviz`

---

## Troubleshooting

### Issue: "ROS2 command not found"

**Solution:**
```bash
# Check if ROS2 is installed
ls /opt/ros/

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Verify
echo $ROS_DISTRO
```

### Issue: "Gazebo models not found"

**Solution:**
```bash
# Set model path
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:~/aigp/drone_racing_sim/models"

# Verify Gazebo can find models
gazebo -e ode ~/aigp/drone_racing_sim/worlds/racing_track.world
```

### Issue: "Package 'drone_racing_core' not found"

**Solution:**
```bash
# Rebuild the package
cd ~/aigp
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_racing_core

# Source the install space
source ~/aigp/install/setup.bash

# Verify
ros2 pkg list | grep drone_racing
```

### Issue: Python import errors

**Solution:**
```bash
# Check Python version
python3 --version

# Reinstall pip packages
pip install --upgrade pip
pip install -r ~/aigp/src/drone_racing_stack/requirements.txt

# Install ROS2 Python packages
sudo apt install -y python3-rclpy python3-geometry2 python3-sensor-msgs
```

### Issue: Colcon build fails

**Solution:**
```bash
# Check dependencies with rosdep
rosdep install --from-paths ~/aigp/src --ignore-src -r -y

# Clean and rebuild
cd ~/aigp
rm -rf build install log
colcon build --packages-select drone_racing_core --symlink-install

# Check for CMake errors
colcon build --packages-select drone_racing_core --event-handler=console_direct+
```

### Issue: Gazebo crashes or fails to start

**Solution:**
```bash
# Clear Gazebo cache
rm -rf ~/.gazebo

# Verify Gazebo-ROS2 plugin
ros2 pkg list | grep gazebo

# Install missing packages
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# Try running simulation
source /opt/ros/humble/setup.bash
source ~/aigp/install/setup.bash
ros2 launch drone_racing_core racing_sim.launch.py
```

### Issue: RViz markers not appearing

**Solution:**
```bash
# Check visualization topic
ros2 topic list | grep marker

# Check topic has data
ros2 topic echo /visualization_marker_array

# Verify RViz is using correct fixed frame
# In RViz: Set "Fixed Frame" to "world"
```

### Issue: Drone not moving or receiving commands

**Solution:**
```bash
# Check autonomy bridge is running
ros2 node list | grep autonomy

# Monitor command topic
ros2 topic echo /cmd_vel

# Check drone state
ros2 topic echo /odom

# View simulation status
ros2 topic echo /gazebo/model_states
```

---

## Quick Reference: One-Line Setup

For a fresh Ubuntu 22.04 installation:

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release ca-certificates && sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions && source /opt/ros/humble/setup.bash && cd ~/aigp && pip install -r src/drone_racing_stack/requirements.txt && colcon build --packages-select drone_racing_core && source ~/aigp/install/setup.bash && export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:~/aigp/drone_racing_sim/models" && echo "Setup complete!"
```

---

## Support & Additional Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Ignition Documentation](https://gazebosim.org/)
- [Colcon Documentation](https://colcon.readthedocs.io/)
- Project README: `/home/shree/aigp/README.md`
- Quick Start Guide: `/home/shree/aigp/drone_racing_sim/QUICKSTART.md`
- Build Instructions: `/home/shree/aigp/drone_racing_sim/BUILD.md`

---

**End of Installation Requirements Document**

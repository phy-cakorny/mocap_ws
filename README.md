# Running MOCAP4ROS2 on the Workstation
Activate the ROS environment and source installation packages: 
- `cd ~/mocap_ws`
- `micromamba activate ros_env`
Repeat for two terminal windows.

Launch driver node (connects to Motive server): make sure Motive multicast is enabled (white stream icon)
- In terminal 1: `ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py`
- In terminal 2: `ros2 lifecycle set /mocap4r2_optitrack_driver_node activate`

To launch visualization step, in terminal 2: 
- `python3 ~/mocap_ws/plot_marks.py`


# MOCAP4ROS2 Workspace Setup Guide

This workspace contains the MOCAP4ROS2 (Motion Capture for ROS 2) system, including support for OptiTrack motion capture systems.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Workspace Setup](#workspace-setup)
- [Building the Workspace](#building-the-workspace)
- [Configuration](#configuration)
- [Running the System](#running-the-system)
- [Troubleshooting](#troubleshooting)

## Prerequisites

### System Requirements

- **ROS 2 Distribution**: Humble Hawksbill or Rolling
- **Operating System**: Ubuntu 22.04 (recommended) or Ubuntu 20.04
- **Python**: 3.8 or higher
- **Build Tools**: 
  - `colcon` build system
  - `vcs` (version control system) tool
  - `rosdep` dependency manager

## ROS environment setup

### Install micromamba

Download and install the static `micromamba` executable using the installation script:
`bash
curl -L micro.mamba.pm/install.sh | bash `

After installation, initialize your shell (e.g., bash, zsh) so that micromamba commands work correctly:
`eval "$(micromamba shell hook --shell=bash)"`

### Create a new ROS environment
Do not install ROS packages in the base environment.
Create a dedicated environment and specify both the conda-forge and robostack channels.

`micromamba create -n ros_env -c conda-forge -c robostack ros-kilted-desktop`

### Activate the environment
`micromamba activate ros_env`

### Install ROS 2 Dependencies

If you haven't already, install ROS 2 and the required tools:

```bash
# Install ROS 2 (Humble)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt update
sudo apt install ros-humble-desktop -y

# Install colcon and vcs
sudo apt install python3-colcon-common-extensions python3-vcstool -y

# Install rosdep
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
```

### Install Python Dependencies

```bash
pip3 install setuptools
```

## Workspace Setup

### 1. Create Workspace Structure

```bash
mkdir -p ~/mocap_ws/src
cd ~/mocap_ws/src
```

### 2. Clone Repositories

The workspace should contain the following packages:

- **mocap_msgs**: Message definitions for motion capture data
- **mocap4r2**: Core MOCAP4ROS2 packages (control system, CLI, visualization)
- **mocap4ros2_optitrack**: OptiTrack driver

If you're starting fresh, you can clone the repositories:

```bash
# Clone OptiTrack driver (includes dependency repos)
cd ~/mocap_ws/src
git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git

# Import dependencies using vcs
vcs import < mocap4ros2_optitrack/dependency_repos.repos
vcs import < mocap4r2/dependency_repos.repos  # If mocap4r2 exists
```

### 3. Install System Dependencies

```bash
cd ~/mocap_ws
rosdep install --from-paths src --ignore-src -r -y
```

This will install all required system dependencies for the packages in your workspace.

## Building the Workspace

### Build All Packages

```bash
cd ~/mocap_ws
colcon build --symlink-install
```

The `--symlink-install` flag creates symbolic links instead of copying files, which is useful during development.

### Build Specific Packages

To build only specific packages:

```bash
colcon build --symlink-install --packages-select mocap4r2_optitrack_driver
```

### Source the Workspace

After building, source the workspace:

```bash
source ~/mocap_ws/install/setup.bash
```

To make this permanent, add it to your `~/.bashrc`:

```bash
echo "source ~/mocap_ws/install/setup.bash" >> ~/.bashrc
```

## Configuration

### OptiTrack Driver Configuration

Edit the OptiTrack driver configuration file:

```bash
nano ~/mocap_ws/src/mocap4ros2_optitrack/mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml
```

Key parameters to configure:

```yaml
mocap4r2_optitrack_driver_node:
  ros__parameters:
    connection_type: "Unicast"  # or "Multicast"
    server_address: "192.168.1.100"    # IP address of Motive computer
    local_address: "192.168.1.50"      # IP address of this computer
    multicast_address: "239.255.42.99" # Only needed for Multicast
    server_command_port: 1510
    server_data_port: 1511
```

**Important**: 
- `server_address`: IP address of the computer running Motive
- `local_address`: IP address of this computer (where ROS 2 is running)
- Ensure both computers are on the same network
- For Unicast, only `server_address` and `local_address` are needed
- For Multicast, also configure `multicast_address`

### Find Your IP Address

To find your computer's IP address:

```bash
hostname -I
# or
ip addr show | grep "inet "
```

## Running the System

### 1. Launch OptiTrack Driver

```bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
```

The driver will:
- Configure itself automatically
- Attempt to connect to the Motive server
- Show connection status in the terminal

**Expected output on successful connection:**
```
[INFO] [mocap4r2_optitrack_driver_node]: Trying to connect to Optitrack NatNET SDK at <server_ip> ...
[INFO] [mocap4r2_optitrack_driver_node]: ... connected!
[INFO] [mocap4r2_optitrack_driver_node]: Application: Motive (ver. X.X.X.X)
[INFO] [mocap4r2_optitrack_driver_node]: Mocap Framerate : XXX.XX
```

### 2. Activate the Driver

The driver is a lifecycle node, so you need to activate it to start streaming:

```bash
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```

Alternatively, use the MOCAP4ROS2 CLI:

```bash
ros2 mocap4r2 start mocap4r2_optitrack_driver_node
```

### 3. Check System Status

View all available MOCAP4ROS2 systems:

```bash
ros2 mocap4r2 status
```

This will show:
- System name
- Current state (NOT READY, READY, ACTIVE, etc.)

### 4. Visualize Data

Launch the visualization tool:

```bash
ros2 launch mocap4r2_marker_viz mocap4r2_marker_viz.launch.py mocap4r2_system:=mocap4r2_optitrack_driver_node
```

This opens RViz with markers and rigid bodies visualization.

### 5. View Published Topics

Check what data is being published:

```bash
ros2 topic list
ros2 topic echo /markers
ros2 topic echo /rigid_bodies
```

## Troubleshooting

### Connection Issues

**Problem**: `"... not connected :(`

**Solutions**:
1. **Verify Motive is running**: Ensure Motive is running on the server computer
2. **Check streaming is enabled**: In Motive, go to View → Data Streaming and enable "Broadcast Frame Data"
3. **Verify IP addresses**: 
   ```bash
   ping <server_address>
   ```
   If ping fails, check network connectivity
4. **Check firewall**: Ensure ports 1510 (command) and 1511 (data) are not blocked
5. **Try Unicast instead of Multicast**: Multicast can be blocked by network switches

### Build Errors

**Problem**: Build fails with missing dependencies

**Solution**:
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**Problem**: `error: option --editable not recognized` (Python packages)

**Solution**: This has been fixed in the workspace. If you encounter it, ensure you're using the latest version of the packages.

### Runtime Errors

**Problem**: Node crashes or doesn't start

**Solution**: Check logs:
```bash
ros2 run mocap4r2_optitrack_driver mocap4r2_optitrack_driver_main
```

**Problem**: No data being published

**Solution**:
1. Ensure the node is activated: `ros2 lifecycle set /mocap4r2_optitrack_driver_node activate`
2. Check subscriptions: `ros2 topic info /markers`
3. Verify Motive is tracking objects and streaming data

### Network Configuration

**For Unicast**:
- Both computers must be on the same subnet
- Firewall must allow UDP ports 1510 and 1511
- Use direct IP addresses (not hostnames)

**For Multicast**:
- Network switches must support multicast
- May require IGMP snooping configuration
- Use multicast address in the 239.x.x.x range

## Package Overview

### Core Packages

- **mocap4r2_control**: Lifecycle node management and control system
- **mocap4r2_control_msgs**: Control and info messages
- **mocap4r2cli**: Command-line interface for managing MOCAP systems
- **mocap4r2_msgs**: Motion capture data messages (markers, rigid bodies)
- **mocap4r2_marker_viz**: Visualization tools for RViz
- **mocap4r2_optitrack_driver**: OptiTrack NatNet SDK driver

### Useful Commands

```bash
# List all MOCAP systems
ros2 mocap4r2 status

# Start a system
ros2 mocap4r2 start <system_name>

# Stop a system
ros2 mocap4r2 stop <system_name>

# View topics
ros2 topic list
ros2 topic echo /markers
ros2 topic echo /rigid_bodies

# Check node state
ros2 lifecycle get /mocap4r2_optitrack_driver_node
```

## Additional Resources

- [MOCAP4ROS2 Project](https://github.com/MOCAP4ROS2-Project)
- [OptiTrack NatNet SDK Documentation](https://optitrack.com/software/nat-net-sdk.html)
- [ROS 2 Documentation](https://docs.ros.org/)

## License

This workspace contains packages licensed under the Apache License 2.0. See individual package LICENSE files for details.


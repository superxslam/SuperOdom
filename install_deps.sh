#!/bin/bash

set -e  # Exit on any error

# SuperOdometry Dependencies Installation Script
# Place this file at: /ws/src/SuperOdom/install_deps.sh

echo "========================================="
echo "SuperOdometry Dependencies Installer"
echo "========================================="

# Get script directory and create deps folder
SCRIPT_DIR=$(dirname "$(realpath "$0")")
DEPS_DIR="$SCRIPT_DIR/deps"

echo "Creating dependencies directory: $DEPS_DIR"
mkdir -p "$DEPS_DIR"
cd "$DEPS_DIR"

# Update system packages
echo "Updating system packages..."
sudo apt update
sudo apt install -y build-essential cmake git python3-pip \
    libboost-all-dev libeigen3-dev libpcl-dev \
    pkg-config libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev

# Install Sophus
echo "Installing Sophus..."
if [ ! -d "Sophus" ]; then
    echo "  Cloning Sophus repository..."
    git clone https://github.com/strasdat/Sophus.git
    cd Sophus
    git checkout 97e7161
    mkdir -p build && cd build
    echo "  Building Sophus..."
    cmake .. -DBUILD_TESTS=OFF
    make -j$(nproc)
    echo "  Installing Sophus..."
    sudo make install
    cd "$DEPS_DIR"
    echo "  ✓ Sophus installed successfully"
else
    echo "  ✓ Sophus already exists, skipping..."
fi

# Install GTSAM
echo "Installing GTSAM..."
if [ ! -d "gtsam" ]; then
    echo "  Cloning GTSAM repository..."
    git clone https://github.com/borglab/gtsam.git
    cd gtsam
    git checkout 4abef92
    mkdir -p build && cd build
    echo "  Building GTSAM..."
    cmake \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        ..
    make -j$(nproc)
    echo "  Installing GTSAM..."
    sudo make install
    cd "$DEPS_DIR"
    echo "  ✓ GTSAM installed successfully"
else
    echo "  ✓ GTSAM already exists, skipping..."
fi

# Install Ceres Solver
echo "Installing Ceres Solver..."
if [ ! -d "ceres-solver" ]; then
    echo "  Cloning Ceres Solver repository..."
    git clone https://github.com/ceres-solver/ceres-solver.git
    cd ceres-solver
    git checkout f68321e7de8929fbcdb95dd42877531e64f72f66
    mkdir -p build && cd build
    echo "  Building Ceres Solver..."
    cmake ..
    make -j$(nproc)
    echo "  Installing Ceres Solver..."
    sudo make install
    cd "$DEPS_DIR"
    echo "  ✓ Ceres Solver installed successfully"
else
    echo "  ✓ Ceres Solver already exists, skipping..."
fi

# Install rviz_2d_overlay_plugins
echo "rviz_2d_overlay_plugins..."
if [ ! -d "rviz_2d_overlay_plugins" ]; then
    echo "  Cloning rviz_2d_overlay_plugins repository..."
    git clone https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins.git
    echo "  ✓rviz_2d_overlay_plugins downloaded successfully"
else
    echo "  ✓ rviz_2d_overlay_plugins already exists, skipping..."
fi

# Install Python packages
echo "Installing Python packages..."
pip3 install --user rerun-sdk
echo "  ✓ Rerun SDK installed successfully"

# Update library cache
echo "Updating library cache..."
sudo ldconfig

echo ""
echo "========================================="
echo "✓ All SuperOdometry dependencies installed successfully!"
echo "========================================="
echo ""
echo "Dependencies installed in: $DEPS_DIR"
echo ""
echo "Please follow instructions to install livox_ros2_driver"
echo "You can now build SuperOdometry using:"
echo "  cd ~/ros2_ws"
echo "  colcon build --packages-select super_odometry"
echo ""


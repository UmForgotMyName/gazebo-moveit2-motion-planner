#!/bin/bash

# X11 and GPU Setup Script for Docker
# This script prepares the host system for proper GPU rendering in Docker

echo "Setting up X11 and GPU access for Docker..."

# Create .docker.xauth file if it doesn't exist
if [ ! -f /tmp/.docker.xauth ]; then
    echo "Creating X11 authentication file..."
    touch /tmp/.docker.xauth
    chmod 666 /tmp/.docker.xauth
fi

# Add current X11 authentication to docker file
echo "Updating X11 authentication..."
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -

# Make sure X11 socket is accessible
echo "Setting X11 socket permissions..."
sudo chmod 666 /dev/dri/*
sudo chmod 666 /tmp/.X11-unix/*

# Check if NVIDIA runtime is available
if command -v nvidia-docker &> /dev/null; then
    echo "✓ NVIDIA Docker runtime detected"
else
    echo "⚠ NVIDIA Docker runtime not found. Install nvidia-container-toolkit if needed."
fi

# Check GPU availability
if nvidia-smi &> /dev/null; then
    echo "✓ NVIDIA GPU detected:"
    nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
else
    echo "⚠ No NVIDIA GPU detected or drivers not installed"
fi

# Test OpenGL
echo "Testing OpenGL availability..."
if glxinfo | grep -q "direct rendering: Yes"; then
    echo "✓ OpenGL direct rendering available"
else
    echo "⚠ OpenGL direct rendering not available"
fi

echo "X11 and GPU setup complete!"
echo "You can now run: docker compose up --build"

#!/usr/bin/env bash
# Host prep for GPU + X11 GUI apps in Docker (RViz2, Gazebo, etc.)
# Compatible with: docker-compose.yml (X socket only, no XAUTH) and Dockerfile (GLVND + X11 libs)
# Usage: ./setup_x11_gpu.sh

set -euo pipefail

# ---------- helpers ----------
info()    { printf "\033[1;34m[i]\033[0m %s\n" "$*"; }
ok()      { printf "\033[1;32m[✓]\033[0m %s\n" "$*"; }
warn()    { printf "\033[1;33m[!]\033[0m %s\n" "$*"; }
err()     { printf "\033[1;31m[x]\033[0m %s\n" "$*" >&2; }
need_cmd(){ command -v "$1" >/dev/null 2>&1 || { err "Required command not found: $1"; exit 1; }; }

# ---------- prerequisites ----------
need_cmd docker
need_cmd xhost

# Detect/normalize DISPLAY
if [[ -z "${DISPLAY:-}" ]]; then
  # Try common local display sockets
  if [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY=:0
  elif [[ -S /tmp/.X11-unix/X1 ]]; then
    export DISPLAY=:1
  fi
fi

if [[ -z "${DISPLAY:-}" ]]; then
  warn "DISPLAY is not set. If you're on Wayland, ensure Xwayland is running and re-run this script from an X11 terminal."
else
  ok "DISPLAY is ${DISPLAY}"
fi

# Verify X socket
if [[ ! -d /tmp/.X11-unix ]]; then
  err "/tmp/.X11-unix not found. Start an X session (or Xwayland) and try again."
  exit 1
fi

# ---------- X access (no XAUTH; matches compose) ----------
info "Granting X access to local root (container user) via xhost..."
# This is tighter than +local:docker and matches containers running as root
xhost +SI:localuser:root >/dev/null || true
ok "X access granted to localuser:root"

# ---------- NVIDIA driver + toolkit checks ----------
if command -v nvidia-smi >/dev/null 2>&1; then
  ok "NVIDIA GPU detected:"
  nvidia-smi --query-gpu=name,driver_version --format=csv,noheader || true
else
  err "nvidia-smi not found or NVIDIA driver not installed on host."
  err "Install the proprietary NVIDIA driver on the host, then re-run."
  exit 1
fi

# Check for NVIDIA Container Toolkit
if command -v nvidia-ctk >/dev/null 2>&1; then
  ok "nvidia-ctk found (NVIDIA Container Toolkit installed)"
  info "Ensuring Docker is configured for NVIDIA runtime (idempotent)..."
  # Configure docker to include the nvidia runtime; harmless if already set
  sudo nvidia-ctk runtime configure --runtime=docker >/dev/null 2>&1 || true
else
  warn "nvidia-ctk not found. Install the NVIDIA Container Toolkit:"
  warn "  sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit"
fi

# Verify Docker sees the nvidia runtime
if docker info 2>/dev/null | grep -qE 'Runtimes:.*nvidia'; then
  ok "Docker reports the NVIDIA runtime is available"
else
  warn "Docker does not list the NVIDIA runtime yet."
  warn "If you just installed the toolkit, restart Docker:"
  warn "  sudo systemctl restart docker"
fi

# ---------- quick runtime sanity tests ----------
info "Testing host OpenGL (optional)..."
if command -v glxinfo >/dev/null 2>&1; then
  if glxinfo | grep -q "direct rendering: Yes"; then
    ok "Host OpenGL direct rendering: Yes"
  else
    warn "Host OpenGL direct rendering: No (GUI apps may still work via Xwayland)"
  fi
else
  warn "glxinfo not installed on host; skipping host OpenGL test"
fi

info "Testing a throwaway CUDA container can see the GPU..."
if docker run --rm --gpus all nvidia/cuda:12.3.2-base-ubuntu22.04 nvidia-smi >/dev/null 2>&1; then
  ok "Container GPU access test passed (nvidia-smi)"
else
  warn "Container GPU access test failed. If you just configured the runtime, run: sudo systemctl restart docker"
fi

# ---------- summary ----------
cat <<EOF

$(printf "\033[1;36m== Ready to launch ==\033[0m")
Environment:
  DISPLAY=${DISPLAY}
X access mode:
  xhost +SI:localuser:root   (no Xauthority; matches docker-compose.yml)

Next steps:
  1) Build and start your stack:
       docker compose up --build -d
  2) Verify inside the running container:
       docker exec -it ros2-moveit2 bash
       nvidia-smi
       apt-get update && apt-get install -y mesa-utils
       glxinfo | grep -E "direct rendering|OpenGL renderer|OpenGL core|vendor|version"
     Expect:
       direct rendering: Yes
       OpenGL renderer string: NVIDIA <your GPU>

If renderer shows 'llvmpipe' or direct rendering is 'No':
  - Ensure docker-compose uses: runtime: nvidia
  - Ensure NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
  - Remove PRIME offload envs unless you're on a hybrid laptop
  - Confirm this Dockerfile includes GLVND/X11 libs (already added)

EOF

ok "Host is prepared for GPU-accelerated OpenGL in Docker."

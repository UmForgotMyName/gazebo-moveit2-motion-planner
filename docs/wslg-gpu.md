# WSLg display and GPU guide

This repo is tuned for WSLg (Wayland-first) to avoid common RViz and Gazebo rendering failures.

## Host setup (WSL Ubuntu)
Run this from WSL Ubuntu (not PowerShell):
```bash
xhost +local:docker
```

## Wayland-first environment
The compose defaults are intended to prefer Wayland, with Xwayland fallback.

Typical environment variables used:
- `WAYLAND_DISPLAY=wayland-0`
- `XDG_RUNTIME_DIR=/run/user/0`
- `QT_QPA_PLATFORM=wayland`

If you set Qt platform manually, use a safe fallback:
```bash
export QT_QPA_PLATFORM='wayland;xcb'
```

## Runtime directory and socket
If Qt complains about runtime directory permissions, check inside the container:
```bash
ls -ld /run/user/0
ls -l /run/user/0/wayland-0
```

The container entrypoint should create `/run/user/0` (0700) and expose the WSLg Wayland socket there.

## Verify GPU acceleration
Inside the container:
```bash
glxinfo -B
```

Under WSL2 GPU acceleration, you should see a D3D12 path.

## Force software rendering (debug only)
If you suspect a GL driver path issue, force software once to confirm:
```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

If software rendering works but normal rendering does not, the problem is almost always in:
- Mesa / d3d12 path
- WSLg socket or runtime dir mismatch
- Qt platform plugin availability

## Qt Wayland plugin errors
If you see:
- `Could not find the Qt platform plugin "wayland"`

Ensure the image includes the Qt Wayland runtime package (commonly `qtwayland5`).

## Common failure patterns
- RViz: "failed to create drawable"
  - Try Wayland-first, then software render to confirm it is graphics-related
- Gazebo crashes on startup
  - Try running with fewer GPU-heavy features to isolate rendering

# 🖥️ BotZilla NoMachine Remote Desktop & Headless GUI Guide

This guide covers how to set up, configure, and troubleshoot **NoMachine** for high-performance, GPU-accelerated remote development on the NVIDIA Jetson Orin Nano Super (and Ubuntu 24.04 LTS / ROS 2 Jazzy). 

NoMachine enables running heavy graphical ROS 2 applications—such as **RViz2**, **Gazebo Harmonic (`ros_gz_sim`)**, and **rqt**—over a local network or VPN with near-native OpenGL hardware acceleration, even when the Jetson has **no physical HDMI monitor attached (headless mode)**.

---

## 📦 1. Installation

### On the NVIDIA Jetson / Robot Server (Ubuntu 24.04 ARM64)
1. Download the ARMv8 (64-bit) DEB package from the official NoMachine site:
   ```bash
   wget https://download.nomachine.com/download/8.11/Arm/nomachine_8.11.3_4_arm64.deb
   ```
   *(Note: Check [nomachine.com/download](https://www.nomachine.com/download) for the exact latest release link).*
2. Install the package using `dpkg`:
   ```bash
   sudo dpkg -i nomachine_*_arm64.deb
   ```
3. Verify the NoMachine server service (`nxserver`) is running:
   ```bash
   sudo /usr/NX/bin/nxserver --status
   ```

### On Your Client Laptop (Windows / macOS / Linux)
1. Download and install the NoMachine Enterprise Client or standard Free Client from [nomachine.com](https://www.nomachine.com).
2. Open NoMachine on your laptop, click **Add**, and enter the Jetson's local IP address (e.g., `192.168.x.x` or VPN IP) using port `4000` (NX protocol) or `22` (SSH protocol).

---

## 🔌 2. Headless Virtual Desktop Configuration (`DISPLAY=:1001`)

### Why Headless Linux Fails by Default
By default, Ubuntu's Xorg / Wayland display manager (`gdm3`) queries the HDMI port for monitor EDID timings on boot. If no physical monitor is plugged in, Xorg either fails to boot entirely (`0x0` resolution) or refuses local hardware GPU rendering. When you try to run `rviz2` over SSH, it crashes with:
```text
qt.qpa.xcb: could not connect to display
```

### How NoMachine Solves This
When NoMachine detects that physical X11 display (`:0`) is unavailable or uninitialized because no monitor is connected, its embedded server automatically spins up a clean, GPU-accelerated **Virtual X11 Display Session** (typically assigned to **`DISPLAY=:1001`**).

#### Force/Restart the Headless Virtual Session:
If your screen is black or stuck at login when connecting headless:
```bash
# 1. Stop the physical display manager if it is stuck trying to find an HDMI monitor
sudo systemctl stop gdm3

# 2. Restart the NoMachine server to force creating a fresh virtual desktop session
sudo /usr/NX/bin/nxserver --restart
```
Now connect from your laptop via NoMachine. You will be greeted with a full Ubuntu graphical desktop running smoothly at `:1001`.

---

## 💻 3. Launching ROS 2 GUI Tools over SSH (VS Code / Terminal)

A common workflow is keeping the NoMachine window open on one screen to watch the robot visualization (`RViz2`/`Gazebo`), while typing commands inside an external SSH terminal or VS Code integrated terminal on your laptop.

### The Problem (`No protocol specified`)
An external SSH terminal session has no idea where your graphical window is located. Running `rviz2` directly inside an SSH terminal fails with:
```text
No protocol specified
qt.qpa.xcb: could not connect to display :1001
```

### The Solution: Exporting X11 Session Variables
Before launching any graphical ROS 2 tool (`ros2 run rviz2 rviz2` or `ros2 launch botzilla_bringup simulation.launch.py`) from an external SSH terminal, export the **Display** and **XAuthority cookie**:

```bash
# Point to NoMachine's virtual display (verify your exact display number inside NoMachine terminal by running `echo $DISPLAY`)
export DISPLAY=:1001

# Point to the X11 authorization cookie file in your home directory
export XAUTHORITY=$HOME/.Xauthority
```

Once exported, any GUI command executed in that terminal will render hardware-accelerated directly onto your open NoMachine desktop window!

#### ⚡ Auto-Detect Snippet for `~/.bashrc`
To avoid typing this every time you SSH into the Jetson, add this snippet to the bottom of your `~/.bashrc` file on the Jetson:

```bash
# Automagically link SSH sessions to NoMachine's virtual X11 display if active
if [ -z "$DISPLAY" ] && [ -f "$HOME/.Xauthority" ]; then
    # Check if NoMachine virtual display :1001 is running
    if pgrep -f "nxnode.*:1001" > /dev/null; then
        export DISPLAY=:1001
        export XAUTHORITY=$HOME/.Xauthority
    elif pgrep -f "nxnode.*:0" > /dev/null; then
        export DISPLAY=:0
        export XAUTHORITY=$HOME/.Xauthority
    fi
fi
```
Run `source ~/.bashrc` to apply. Now your SSH terminals automatically target the active NoMachine display.

---

## 🏎️ 4. Performance & RViz2 / Gazebo Optimization

### 1. Display Quality Settings inside NoMachine
Press **`Ctrl + Alt + 0`** inside the active NoMachine window to open the quick settings panel:
* **Display $\rightarrow$ Change Settings**: Set Quality slider to **Middle / High** (or adjust down if working over slow Wi-Fi).
* **Resolution**: Select **Match the client resolution** or choose a crisp standard resolution like `1920x1080`.
* **Audio**: Mute audio forwarding under **Audio settings** to save network bandwidth if robot audio is unneeded.

### 2. OpenGL & Hardware Acceleration Verification
To confirm RViz2 is utilizing the NVIDIA Jetson GPU (and not falling back to slow CPU software rendering), check the startup logs when launching RViz2:
```text
[INFO] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
```
*(Note: Warnings stating `Stereo is NOT SUPPORTED` are normal inside virtual sessions and can be safely ignored).*

If OpenGL fails or renders black viewports, ensure software override variables are unset:
```bash
unset LIBGL_ALWAYS_SOFTWARE
```

### 3. RViz2 QoS Durability for Latched Topics (`/map`)
When viewing SLAM maps generated by `rtabmap` inside RViz2 over NoMachine, if the map stays blank with a warning `Topic /map: QoS incompatibility`:
1. In RViz2, expand the **Map** display item on the left panel.
2. Expand **Topic $\rightarrow$ QoS Settings**.
3. Set **Durability Policy** to **`Transient Local`** (latched) and **Reliability** to **`Reliable`**.

---

## ❓ 5. Troubleshooting & FAQ

| Symptom / Error | Root Cause | Solution |
| :--- | :--- | :--- |
| **`No protocol specified`** when running `rviz2` from SSH | SSH session lacks authorization to draw on NoMachine's X11 screen. | Run `export XAUTHORITY=$HOME/.Xauthority` and verify `export DISPLAY=:1001` (or `:0`). |
| **`qt.qpa.xcb: could not connect to display`** | `DISPLAY` variable not set or targeting wrong/closed session number. | Check open NoMachine window terminal (`echo $DISPLAY`) and match it in your SSH terminal. |
| **Black Screen upon NoMachine connection** | Physical Xorg (`gdm3`) is frozen attempting to query a disconnected HDMI monitor. | SSH into the machine and run:<br>`sudo systemctl stop gdm3`<br>`sudo /usr/NX/bin/nxserver --restart` |
| **Resolution locked to `1024x768` or `640x480`** | NoMachine created a minimal fallback frame because no resolution was requested. | Press `Ctrl + Alt + 0` $\rightarrow$ **Display** $\rightarrow$ **Change Settings** $\rightarrow$ Select your preferred custom resolution or **Match client resolution**. |
| **High latency or mouse input lag** | Network congestion or uncompressed rendering over Wi-Fi. | Press `Ctrl + Alt + 0` $\rightarrow$ **Display** $\rightarrow$ lower the **Quality** slider one notch toward **Speed**, and disable **Audio forwarding**. |

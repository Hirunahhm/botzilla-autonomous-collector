# BotZilla Movement Controls

This document outlines how to control the BotZilla robot (Kobuki QBot) using ROS 2.

## 1. Launching the Controller
To enable keyboard control, ensure your workspace is sourced and run the following in a terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 2. Key Mappings

### Movement (Linear & Angular)
| Key | Action |
| :--- | :--- |
| **i** | Move Forward |
| **,** | Move Backward |
| **j** | Rotate Left (Counter-Clockwise) |
| **l** | Rotate Right (Clockwise) |
| **u** | Curve Forward-Left |
| **o** | Curve Forward-Right |
| **m** | Curve Backward-Left |
| **.** | Curve Backward-Right |
| **k** | **STOP** (Zero velocity) |

### Speed Adjustment
| Key | Action |
| :--- | :--- |
| **q / z** | Increase / Decrease overall speed by 10% |
| **w / x** | Increase / Decrease linear speed only by 10% |
| **e / c** | Increase / Decrease angular speed only by 10% |

## 3. Hardware Integration Logic
The `kobuki_base_node` translates these keyboard commands into specific wheel velocities for the hardware:

*   **Wheel Base**: 230mm (0.230m)
*   **Units**: Meters per second (m/s) from Teleop are converted to Millimeters per second (mm/s) for the `KobukiDriver`.
*   **Rotation Flag**: 
    *   `rotate = 1`: Used for pure rotations (Spinning in place).
    *   `rotate = 0`: Used for translation or curved movement.

## 4. Troubleshooting
If the robot does not move:
1. Verify the serial connection: `ls /dev/ttyUSB*`.
2. Ensure the user has dialout permissions: `sudo usermod -a -G dialout $USER`.
3. Check the ROS 2 graph to ensure `/cmd_vel` is being published: `ros2 topic echo /cmd_vel`.
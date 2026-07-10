# BotZilla Autonomous Delivery Algorithm

This document outlines the logic and perception pipeline used for detecting cubes, approaching them, and delivering them to designated drop-off zones identified by AprilTags.

## 1. Phase 1: Cube Acquisition (YOLO + Depth Focus)

### **A. Searching for Cubes**
1.  **State:** `SEARCHING`
2.  **Logic:** The robot rotates in place at a constant angular velocity (`0.4 rad/s`).
3.  **Vision:** The `yolo_node` processes the RGB stream from the Kinect.
4.  **Verification:** A "detection" is only valid if a cube is found within the Kinect's reliable depth range (0.5m to 4.0m).

### **B. Targeting and Alignment**
1.  **State:** `TARGETING`
2.  **Logic:** Uses a **Proportional (P) Controller** to center the cube.
3.  **Math:** `Angular_Velocity = -KP * Horizontal_Offset`
    *   `Offset` is normalized from `-1.0` (left) to `+1.0` (right).
    *   This ensures smooth bidirectional rotation to face the cube.

### **C. Approach and Capture**
1.  **State:** `APPROACHING`
2.  **Logic:** Robot moves forward at `0.08 m/s` while maintaining centering.
3.  **The "Blind Spot" Capture:** Kinect cannot see objects closer than **0.55m**.
    *   Once the reported distance `z` becomes `0.0` (or below threshold), the robot enters the `CAPTURING` state.
    *   **Blind Drive:** The robot drives forward blindly for **2.5 seconds** to ensure the cube is firmly pushed into the mechanical "pocket" or gripper area.

---

## 2. Phase 2: Delivery (AprilTag Navigation)

### **A. Searching for Drop-Off Zone**
1.  **State:** `DELIVERING` (Searching Sub-state)
2.  **Logic:** Once the cube is pocketed, the robot switches its perception focus to **AprilTag Detection**.
3.  **Target IDs:** Specifically looks for Tag **203** (Main Drop-off) or **113** (Secondary).
4.  **Mirror-Robustness:** The algorithm checks both raw and mirrored images to handle phone/tablet screen displays of the tags.

### **B. Approach via Tag-Following**
1.  **Alignment:** Similar to cube targeting, the robot centers the AprilTag using the horizontal offset from the camera frame.
2.  **Distance Proxy:** Instead of raw depth, the robot uses the **pixel height** of the tag:
    *   `Tag_Height = Average(Left_Side_Pixels, Right_Side_Pixels)`
    *   As the robot gets closer, the tag appears larger in the frame.

### **C. The "Arrived" and Drop Logic**
1.  **Stopping Condition:** When `Tag_Height > 300` pixels (indicating the robot is roughly 0.4m from the wall/zone), it triggers the `ARRIVED` state.
2.  **Dropping/Detaching:**
    *   The robot comes to a full stop.
    *   **Detaching:** It reverses away from the tag for **2.5 seconds**, leaving the cube behind in the drop-off zone.
    *   **Reset:** The state machine transitions back to `SEARCHING` to find the next cube.

---

## Technical Summary Table

| Task | Sensor/Model | Feedback Metric | Control Strategy |
| :--- | :--- | :--- | :--- |
| **Detection** | YOLOv8 | Class ID + BBox | Vision-based Scanning |
| **Distance** | Kinect Depth | Meter-scaled Mono8 | Depth Thresholding |
| **Centering** | Kinect RGB | Normalized X-Offset | Proportional (P) Control |
| **Drop-off** | ArUco/AprilTag | Pixel Height (Z-proxy) | Size-based Trigger |
| **Rotation** | Kobuki Base | Wheel Differential | Radius-based In-place Turn |

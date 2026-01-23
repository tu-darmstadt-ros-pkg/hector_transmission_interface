# hector_transmission_interface

![Lint, Build & Test](https://github.com/tu-darmstadt-ros-pkg/hector_transmission_interface/actions/workflows/lint_build_test.yaml/badge.svg)

Here is the comprehensive, final version of the `README.md`. It combines all the technical details, the persistence
logic, the `AdjustableOffsetManager` usage with mutex protection, and clear safety guidelines.

`hector_transmission_interface` provides an **Adjustable Offset Transmission** for `ros2_control`. It is designed for
systems where the relationship between an actuator and a joint can change during runtime (e.g., belt slip) or where the
relationship is ambiguous upon startup (e.g., relative encoders on an actuator).

---

## ⚙️ Adjustable Offset Transmission

### 💾 Persistence & Ambiguity Resolution

This package is critical for robots with **Relative Actuator Encoders** (e.g., motors with incremental encoders placed
before a high-ratio gearbox) that lack absolute joint sensing.

* **The Ambiguity Problem**: If a robot is moved while powered off, the relationship between the motor's zero-point and
  the joint's physical zero-point is lost. Standard transmissions assume the current position is "Zero" on startup,
  leading to kinematic errors.
* **The Solution**: This transmission loads the last known "Good" offset from
  `~/.ros/dynamic_offset_transmissions/<joint_name>.txt` on startup. If the robot was moved while off, a quick
  calibration (Visual, Laser, or Mechanical Jig) can be used to update the offset via service call without restarting.

### 🛠 How It Works

The transmission applies an offset to raw actuator data. New offsets are calculated as:

---

## 🚀 AdjustableOffsetManager

The `AdjustableOffsetManager` automates joint registration and provides a standardized ROS 2 service to calibrate
multiple joints at once.

### 🔒 Thread Safety (Recommended)

In `ros2_control`, the `read()`/`write()` loop (Real-Time) and ROS service callbacks (Non-RT) run in different threads.
To avoid data corruption or "jumps" during an update, the manager uses
`std::optional<std::reference_wrapper<std::mutex>>` to safely lock your hardware communication mutex.

### Integration in Hardware Interface

```cpp
#include "hector_transmission_interface/adjustable_offset_manager.hpp"

// 1. Initialize with a reference to your Hardware Interface mutex
// This ensures the service blocks the HW thread during the update.
offset_manager_ = std::make_unique<AdjustableOffsetManager>(node_ptr, comm_mutex_);

// 2. Register joints with a lambda getter for live state variables
for (auto& joint : my_joints) {
    offset_manager_->add_joint(
        joint.name,
        joint.state_transmission,
        joint.command_transmission,
        [&joint]() { return joint.position_state; }
    );
}

// make sure to block the hardware thread in your read()/write() methods
void MyHardwareInterface::read() {

  std::unique_lock<std::mutex> lock(dynamixel_comm_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    // Another operation is holding dynamixel_comm_mutex_; skipping write
    return hardware_interface::return_type::OK;
  }
  ... normal read/write logic ...
 }
```

---

## 📡 Service Definition & Usage

The manager exposes the following service:
`~/adjust_transmission_offsets` [`hector_transmission_interface_msgs/srv/AdjustTransmissionOffsets`]

### Request Fields

* `external_joint_measurements`: The "Ground Truth" positions (e.g., from a camera).
* `original_joint_state` (Optional): The internal positions recorded at the exact moment the external measurement was
  taken. If omitted, the manager uses the live values from the `PositionGetter`.

### Terminal Example

If an external visual system detects that your `flipper_joint` is actually at `1.5708` rad (90°), but the robot thinks
it is elsewhere:

```bash
ros2 service call /my_robot/adjust_transmission_offsets \
hector_transmission_interface_msgs/srv/AdjustTransmissionOffsets \
"{
  external_joint_measurements: {
    name: ['flipper_joint'],
    position: [1.5708]
  }
}"

```

---

## ⚠️ Safety & Best Practices

> **CONTROLLER CONFLICTS & DISCONTINUITIES**
> Adjusting an offset changes the "Ground Truth" of a joint's position instantly. If a controller (like a PID-based
> position controller) is active, it will perceive this change as a massive, instantaneous error (a "step change"), which
> can trigger an aggressive and potentially damaging motor response.

### Recommended Calibration Workflow

1. **Stop/Deactivate** active controllers for the joint.
2. **Perform** the external ground-truth measurement.
3. **Call** the adjustment service.
4. **Restart/Activate** the controllers.

---

## 🛠 Troubleshooting

| Issue                  | Possible Cause     | Solution                                                                          |
|------------------------|--------------------|-----------------------------------------------------------------------------------|
| Offset not persisting  | Permissions        | Check `~/.ros/dynamic_offset_transmissions/` write access.                        |
| Initial position wrong | Manual Move        | If the robot moved while OFF, the saved offset is invalid. Trigger a calibration. |
| Motor "Jerk"           | Active Controllers | Ensure controllers are stopped/restarted during the service call.                 |
| Service not found      | Executor           | Ensure the Node passed to the Manager is spinning in an executor.                 |

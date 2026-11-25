# hector_transmission_interface

![Lint, Build & Test](https://github.com/tu-darmstadt-ros-pkg/hector_transmission_interface/actions/workflows/lint_build_test.yaml/badge.svg)

`hector_transmission_interface` provides an **Adjustable Offset Transmission** for `ros2_control`. It is meant for
systems where the relationship between an actuator and a joint can change at runtime (e.g., belt slip) or is
ambiguous at startup (e.g., relative encoders on an actuator).

---

## ⚙️ Adjustable Offset Transmission

### 💾 Persistence & Ambiguity Resolution

This package is meant for robots with **relative actuator encoders** (e.g., motors with incremental encoders placed
before a high-ratio gearbox) that lack absolute joint sensing.

* **The ambiguity problem**: If a robot is moved while powered off, the relationship between the motor's zero point and
  the joint's physical zero point is lost. Standard transmissions assume the current position is zero on startup,
  which leads to kinematic errors.
* **The solution**: This transmission loads the last known offset from
  `~/.ros/dynamic_offset_transmissions/<joint_name>.txt` on startup. If the robot was moved while off, a quick
  calibration (visual, laser or a mechanical jig) updates the offset through a service call without a restart.

### 🛠 How It Works

The transmission maps `joint = actuator / reduction + offset`. A calibration sets the offset so that the joint
position matches an external measurement:

```
new_offset = measured_position - internal_position + current_offset
```

The state and the command transmission of a joint share one offset, so they always agree.

Each joint saves its offset on a writer thread of its own, so a change never waits for the disk, not even one made in
the control loop. Call `JointOffset::flush()` (through `jointOffset()`) when you need the file to be up to date, for
example before you read it in a test. The destructor writes whatever is still pending.

---

## 🚀 AdjustableOffsetManager

The `AdjustableOffsetManager` registers joints and offers two services to calibrate several joints at once,
`~/adjust_transmission_offsets` and `~/flip_by_pi`.

### 🔒 Thread Safety

The services run on a ROS executor thread, while `read()` and `write()` map values with the offsets. An adjustment must
not land in the middle of a read or write. The manager supports two ways to ensure that.

1. **Run the adjustment in the control loop** (preferred). Pass a function to `set_run_in_control_loop` that hands the
   adjustment to your `read()` and waits until it ran. It must return false only if the adjustment never ran and never
   will. No lock is needed, and the position getters and the post-callback run in the control loop too.
2. **Share a mutex.** Pass your hardware communication mutex to the constructor. The manager holds it during the
   adjustment, and `read()` and `write()` must skip their cycle while it is taken.

`set_may_adjust` lets the hardware refuse single joints, for example because a controller commands them and would see
its joint jump. Refused joints are skipped and listed in the response message after `refused:`, together with the
reason.

### Integration in a Hardware Interface

```cpp
#include "hector_transmission_interface/adjustable_offset_manager.hpp"

hardware_interface::CallbackReturn MyHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  ...
  offset_manager_ = std::make_shared<AdjustableOffsetManager>(node_);
  offset_manager_->set_run_in_control_loop(
      [this](const std::function<void()>& adjustment) { return runInControlLoop(adjustment); });
  offset_manager_->set_may_adjust([this](const std::string& joint) {
    return isCommanded(joint) ? std::string("a controller commands it") : std::string();
  });
  ...
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> MyHardwareInterface::on_export_state_interfaces()
{
  ...
  for (const auto& [name, joint] : joints_) {
    // Ignored unless the state transmission is an AdjustableOffsetTransmission
    offset_manager_->add_joint_state_interface(
        name, joint.state_transmission,
        [&joint]() { return joint.position; },
        joint.command_transmission);
  }
  return state_interfaces;
}
```

Pass the command transmission too. It keeps its own joint goal, and the manager re-derives that goal when the offset
changes. Without it the next `write()` moves the actuator by the offset change.

With the mutex model, construct the manager with `AdjustableOffsetManager(node, comm_mutex_)` instead. Optional pre- and
post-callbacks (`std::function<bool()>`) can be passed as well. The pre-callback runs before the mutex is taken, the
post-callback while it is held or in the control loop.

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

## 🔄 Automatic 2π Actuator Jump Correction

Dynamixel actuators can briefly lose power (e.g., due to voltage dips or connector issues) and lose their absolute
rotation tracking. When power returns, the actuator's internal position resets by a multiple of 2π. For joints **with a
transmission** (reduction ≠ 1), this causes the joint position to jump by `2π / reduction`, which is observable and
incorrect.

### How It Works

The `AdjustableOffsetTransmission` overrides `actuator_to_joint()` to automatically detect and correct these jumps:

1. On each read cycle, the actuator position delta from the previous cycle is checked.
2. If the delta is close to a multiple of 2π (within a configurable tolerance of 0.2π), a power-glitch jump is assumed.
3. The joint offset is adjusted by `-n × 2π / reduction` to cancel out the jump.
4. The corrected offset is handed to the joint's writer thread.
5. A warning is logged for diagnostics.

The command transmission shares the offset, so goals follow the correction without help from the hardware interface.


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

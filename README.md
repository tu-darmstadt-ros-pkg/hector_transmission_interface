# hector_transmission_interface
![Lint](https://github.com/tu-darmstadt-ros-pkg/hector_transmission_interface/actions/workflows/lint_build_test.yaml/badge.svg)

A ROS 2 package providing an adjustable offset transmission interface for handling transmission offsets that can change during runtime, such as when a belt or chain slips.

## Adjustable Offset Transmission

### Overview
Extends the standard `transmission_interface::SimpleTransmission` to support persistent storage and runtime adjustment of joint offsets. This is particularly useful for detecting and correcting slipping of transmissions (e.g., belts, chains) without restarting the system.

### Features
- **Persistent Storage**: Offsets are automatically saved to disk and restored on startup
- **Runtime Adjustment**: Offsets can be modified during operation via service calls
- **Per-Joint Configuration**: Each joint maintains its own independent offset file

### How It Works

#### Offset Storage Location
Offsets are stored persistently in the filesystem at:
```
~/.ros/dynamic_offset_transmissions/<joint_name>.txt
```

Each joint has its own text file containing a single floating-point value representing the current offset. The directory is automatically created if it doesn't exist.

#### Initialization Behavior
When a transmission is loaded:
1. If an offset file exists for the joint, it loads the saved offset value
2. If no file exists, it uses the offset specified in the URDF and creates a new file
3. The transmission logs any offset changes during initialization

#### Configuration in URDF
To use the adjustable offset transmission, specify it in your robot's URDF/xacro:

```xml
<ros2_control>
  <transmission name="my_joint_transmission">
    <plugin>hector_transmission_interface/AdjustableOffsetTransmissionLoader</plugin>
    <joint name="my_joint" role="joint1">
      <mechanical_reduction>1.0</mechanical_reduction>
      <offset>0.0</offset>  <!-- Initial offset, will be overridden by saved value if it exists -->
    </joint>
    <actuator name="my_actuator" role="actuator1"/>
  </transmission>
</ros2_control>
```

### Adjusting Offsets

#### Method 1: Service Call (Recommended)
Use the `AdjustTransmissionOffsets` service to adjust offsets at runtime:

```bash
ros2 service call /adjust_transmission_offsets hector_transmission_interface_msgs/srv/AdjustTransmissionOffsets \
  "{external_joint_measurements: {name: ['joint1', 'joint2'], position: [1.57, -0.785]}}"
```

The service compares external measurements with current joint states and calculates the required offset adjustments.

#### Method 2: Direct File Modification
Offsets can also be manually edited by modifying the text files in `~/.ros/dynamic_offset_transmissions/`, but this requires restarting the controller manager to take effect.

#### Method 3: Programmatic API
From C++ code, call `adjustTransmissionOffset(double offset)` on the transmission object:

```cpp
transmission->adjustTransmissionOffset(new_offset);
```

### Important Safety Notes

⚠️ **WARNING**: Always deactivate controllers claiming the affected joints before adjusting offsets! Changing offsets while controllers are active can cause unexpected behavior or damage.

**Best Practice Workflow:**
1. Stop all controllers using the affected joints
2. Adjust the transmission offset(s)
3. Restart the controllers

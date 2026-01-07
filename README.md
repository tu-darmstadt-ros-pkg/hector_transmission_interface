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

#### Method 1: Programmatic API (Primary Method)
Access the transmission object from your hardware interface and call `adjustTransmissionOffset()`:

```cpp
// Cast to AdjustableOffsetTransmission to access the method
auto* adjustable_transmission = dynamic_cast<hector_transmission_interface::AdjustableOffsetTransmission*>(transmission.get());
if (adjustable_transmission) {
  adjustable_transmission->adjustTransmissionOffset(new_offset);
}
```

**Example: Implementing a Service to Adjust Offsets**

The package provides the `AdjustTransmissionOffsets.srv` service definition, but you must implement the service server in your own code. Here's an example implementation:

```cpp
void adjustOffsetsCallback(
  const hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request::SharedPtr request,
  hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Response::SharedPtr response)
{
  // Get current internal joint position from your robot state
  double internal_joint_position = joint_state_interface->get_position();

  // Get external measurement from service request
  double external_joint_position = request->external_joint_measurements.position[joint_index];

  // Access the transmission and calculate corrected offset
  auto* adjustable_transmission = dynamic_cast<hector_transmission_interface::AdjustableOffsetTransmission*>(
    transmission_manager->get(joint_name).get());

  if (adjustable_transmission) {
    double current_offset = adjustable_transmission->get_joint_offset();
    double corrected_offset = external_joint_position - internal_joint_position + current_offset;

    adjustable_transmission->adjustTransmissionOffset(corrected_offset);

    response->adjusted_offsets.push_back(corrected_offset);
    response->success = true;
  } else {
    response->success = false;
    response->message = "Failed to cast transmission to AdjustableOffsetTransmission";
  }
}
```

Once your service is implemented, you can call it with:

```bash
ros2 service call /adjust_transmission_offsets hector_transmission_interface_msgs/srv/AdjustTransmissionOffsets \
  "{external_joint_measurements: {name: ['joint1', 'joint2'], position: [1.57, -0.785]}}"
```

#### Method 2: Direct File Modification
Offsets can be manually edited by modifying the text files in `~/.ros/dynamic_offset_transmissions/`. This requires restarting the controller manager to take effect and is not recommended for runtime adjustments.

### Important Safety Notes

⚠️ **WARNING**: Always deactivate controllers claiming the affected joints before adjusting offsets! Changing offsets while controllers are active can cause unexpected behavior or damage.

**Best Practice Workflow:**
1. Stop all controllers using the affected joints
2. Adjust the transmission offset(s)
3. Restart the controllers

## Service Message Definition

The `AdjustTransmissionOffsets.srv` provides a standard interface for adjusting offsets:

**Request:**
- `sensor_msgs/JointState external_joint_measurements` - External measurements of joint positions

**Response:**
- `float64[] adjusted_offsets` - The new offset values that were set
- `bool success` - Whether the operation succeeded
- `string message` - Status message or error description

**Note:** This library provides only the message definition. Service implementation is the responsibility of the library user and should be tailored to your specific hardware interface and calibration workflow.

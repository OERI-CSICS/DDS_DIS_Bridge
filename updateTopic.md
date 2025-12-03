# Updating DDS Topic Subscriptions

This guide explains how to modify the DDS to DIS bridge to connect to different ROS2 DDS topics and message types.

## Understanding the Subscription Code

The core subscription code in `dds_dis_bridge.cpp`:

```cpp
subscription_ = this->create_subscription<my_cpp_package::msg::ShipEntityState>(
  "ship_entity_states", 10,
  std::bind(&DDSToDISBridge::topic_callback, this, std::placeholders::_1));
```

### Code Breakdown

#### Template Parameter: `<my_cpp_package::msg::ShipEntityState>`
- **What it is**: The message type this subscription expects
- **Format**: `<package_name::msg::MessageType>`
- **Purpose**: Type safety - ensures you only receive the correct message format

#### Parameters:

1. **`"ship_entity_states"`** - The DDS topic name to subscribe to
2. **`10`** - Queue size (how many messages to buffer)
3. **`std::bind(...)`** - The callback function to execute when messages arrive

#### The Callback Binding:
- **`&DDSToDISBridge::topic_callback`** - Pointer to the member function
- **`this`** - The current object instance
- **`std::placeholders::_1`** - Placeholder for the message parameter

## Connecting to Different DDS Topics

### Scenario 1: Different Topic Name, Same Message Type

If you want to connect to a topic named `"naval_vessels"` instead:

```cpp
subscription_ = this->create_subscription<my_cpp_package::msg::ShipEntityState>(
  "naval_vessels", 10,  // <-- Just change the topic name
  std::bind(&DDSToDISBridge::topic_callback, this, std::placeholders::_1));
```

**Use Case**: When your ship data comes from a different publisher but uses the same message format.

### Scenario 2: Different Message Type

If you want to connect to a topic with a different message format, like `geometry_msgs/PoseStamped`:

```cpp
// At the top, add the include
#include "geometry_msgs/msg/pose_stamped.hpp"

// In the constructor, change the subscription
subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
  "ship_poses", 10,
  std::bind(&DDSToDISBridge::pose_callback, this, std::placeholders::_1));
```

**You'd need a new callback function:**

```cpp
void pose_callback(const geometry_msgs::msg::PoseStamped & msg) const
{
  RCLCPP_INFO(this->get_logger(), "Received pose data");
  
  // Convert PoseStamped to DIS format
  // Access fields like:
  // msg.pose.position.x
  // msg.pose.position.y
  // msg.pose.position.z
  // msg.pose.orientation.w, x, y, z (quaternion)
  
  // You'll need to convert this data to your ShipEntityState format
  // or directly create DIS packets from it
}
```

### Scenario 3: External System with Custom Message

If another system publishes ship data with their own message type:

```cpp
// Include their message type
#include "external_navy_package/msg/vessel_state.hpp"

// Update subscription
subscription_ = this->create_subscription<external_navy_package::msg::VesselState>(
  "fleet_tracking", 10,
  std::bind(&DDSToDISBridge::vessel_callback, this, std::placeholders::_1));
```

**Custom callback for external message:**

```cpp
void vessel_callback(const external_navy_package::msg::VesselState & msg) const
{
  RCLCPP_INFO(this->get_logger(), "Received vessel data for ID %d", msg.vessel_id);
  
  // Convert their format to DIS
  // Map their fields to DIS fields:
  // msg.vessel_id     -> entity_id
  // msg.gps_lat       -> latitude  
  // msg.gps_lon       -> longitude
  // msg.heading_deg   -> heading
  // msg.speed_knots   -> velocity calculation
  // etc.
  
  // Create and send DIS packet with converted data
}
```

## Real-World Integration Examples

### Connecting to AIS (Automatic Identification System) Data

```cpp
#include "marine_msgs/msg/ais_message.hpp"

// In constructor:
subscription_ = this->create_subscription<marine_msgs::msg::AisMessage>(
  "/ais/vessel_reports", 10,
  std::bind(&DDSToDISBridge::ais_callback, this, std::placeholders::_1));

// Callback implementation:
void ais_callback(const marine_msgs::msg::AisMessage & msg) const
{
  // AIS messages contain:
  // - MMSI (ship identifier)
  // - Latitude/Longitude
  // - Speed over ground
  // - Course over ground
  // - Ship type and dimensions
  
  // Convert to DIS EntityStatePdu
}
```

### Connecting to Maritime Radar Data

```cpp
#include "radar_msgs/msg/contact.hpp"

// In constructor:
subscription_ = this->create_subscription<radar_msgs::msg::Contact>(
  "/radar/sea_contacts", 10, 
  std::bind(&DDSToDISBridge::radar_callback, this, std::placeholders::_1));

// Callback implementation:
void radar_callback(const radar_msgs::msg::Contact & msg) const
{
  // Radar contacts typically have:
  // - Range and bearing
  // - Track number
  // - Estimated speed and course
  // - Contact classification
  
  // Convert radar coordinates to lat/lon and create DIS packet
}
```

## Multiple Topic Subscriptions

You can subscribe to multiple topics simultaneously:

```cpp
class DDSToDISBridge : public rclcpp::Node
{
private:
  // Multiple subscription members
  rclcpp::Subscription<my_cpp_package::msg::ShipEntityState>::SharedPtr ship_subscription_;
  rclcpp::Subscription<marine_msgs::msg::AisMessage>::SharedPtr ais_subscription_;
  rclcpp::Subscription<radar_msgs::msg::Contact>::SharedPtr radar_subscription_;
  
public:
  DDSToDISBridge() : Node("dds_dis_bridge")
  {
    // Subscribe to ship entity states
    ship_subscription_ = this->create_subscription<my_cpp_package::msg::ShipEntityState>(
      "ship_entity_states", 10,
      std::bind(&DDSToDISBridge::ship_callback, this, std::placeholders::_1));

    // Subscribe to AIS data
    ais_subscription_ = this->create_subscription<marine_msgs::msg::AisMessage>(
      "ais_data", 10,
      std::bind(&DDSToDISBridge::ais_callback, this, std::placeholders::_1));
      
    // Subscribe to radar contacts
    radar_subscription_ = this->create_subscription<radar_msgs::msg::Contact>(
      "radar_contacts", 10,
      std::bind(&DDSToDISBridge::radar_callback, this, std::placeholders::_1));
  }
  
  // Separate callbacks for each data type
  void ship_callback(const my_cpp_package::msg::ShipEntityState & msg) const { /* ... */ }
  void ais_callback(const marine_msgs::msg::AisMessage & msg) const { /* ... */ }
  void radar_callback(const radar_msgs::msg::Contact & msg) const { /* ... */ }
};
```

## Step-by-Step Modification Process

### 1. Identify Your Data Source
- What topic name does it publish on?
- What message type does it use?
- What fields contain the position, heading, velocity data you need?

### 2. Check Available Topics
```bash
# List all active topics
ros2 topic list

# See message type for a topic
ros2 topic info /your_topic_name

# View message structure
ros2 interface show your_package/msg/YourMessage
```

### 3. Update Dependencies
Add the message package to `package.xml`:
```xml
<depend>your_message_package</depend>
```

Add to `CMakeLists.txt`:
```cmake
find_package(your_message_package REQUIRED)
ament_target_dependencies(dds_dis_bridge rclcpp std_msgs your_message_package)
```

### 4. Modify the Code
1. Add the include for the new message type
2. Update the subscription template parameter
3. Change the topic name
4. Modify or create a new callback function
5. Map the incoming data fields to DIS packet fields

### 5. Test Your Changes
```bash
# Rebuild
colcon build --packages-select my_cpp_package

# Run your bridge
ros2 run my_cpp_package dds_dis_bridge

# In another terminal, check if it's subscribing correctly
ros2 node info /dds_dis_bridge
```

## Common Message Types for Maritime Data

| Message Type | Package | Typical Fields |
|--------------|---------|----------------|
| `ShipEntityState` | `my_cpp_package` | lat, lon, heading, velocity |
| `PoseStamped` | `geometry_msgs` | position, orientation |
| `NavSatFix` | `sensor_msgs` | latitude, longitude, altitude |
| `Twist` | `geometry_msgs` | linear/angular velocities |
| `AisMessage` | `marine_msgs` | MMSI, position, speed, course |
| `RadarContact` | `radar_msgs` | range, bearing, track info |

## Troubleshooting

### Build Errors
- **Missing includes**: Add `#include` for the message type
- **Dependency errors**: Update `package.xml` and `CMakeLists.txt`
- **Type mismatches**: Verify template parameter matches actual message type

### Runtime Issues
- **No messages received**: Check topic names with `ros2 topic list`
- **Wrong data**: Verify message structure with `ros2 topic echo`
- **Callback not called**: Ensure topic is being published

### Testing Tips
```bash
# Monitor your bridge's subscriptions
ros2 node info /dds_dis_bridge

# Echo the topic to see raw data
ros2 topic echo /your_topic_name

# Check message rates
ros2 topic hz /your_topic_name
```

## Best Practices

1. **Use descriptive callback names**: `ais_callback`, `radar_callback`, etc.
2. **Handle message validation**: Check for required fields before processing
3. **Log important events**: Use `RCLCPP_INFO` for debugging
4. **Consider data freshness**: Check timestamps and reject stale data
5. **Error handling**: Gracefully handle malformed or incomplete messages

This flexible architecture allows the DDS to DIS bridge to connect to virtually any ROS2 topic containing position or state information, making it adaptable to different maritime systems and data sources.
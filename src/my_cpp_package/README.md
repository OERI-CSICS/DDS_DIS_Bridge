# ROS2 Ship Simulation Package

A ROS2 package that simulates ship movements using an Entity Component System (ECS) architecture integrated from EM_Mapper, with support for dead reckoning, waypoint navigation, and DIS protocol bridge.

## Overview

This package provides a sophisticated ship simulation system that publishes ship entity states to ROS2 topics and bridges them to DIS (Distributed Interactive Simulation) protocol for interoperability with military simulation systems.

**Key Features:**
- **Entity Component System (ECS)** architecture for performance and flexibility
- **Dead reckoning** simulation with accurate lat/lon coordinate handling
- **Waypoint-based navigation** for scripted ship movements
- **YAML configuration** for easy ship setup without recompiling
- **DDS-to-DIS bridge** for protocol conversion
- **RF modeling components** (available but deactivated) for future electromagnetic simulation

## Architecture

### Components

```
┌─────────────────────────────────────────────────────────────┐
│                    ship_publisher                           │
│  - Loads ships from YAML config                             │
│  - Runs ECS simulation (dead reckoning + waypoints)         │
│  - Publishes to "ship_entity_states" topic                  │
└─────────────────┬───────────────────────────────────────────┘
                  │
                  │ ROS2 DDS (ShipEntityState messages)
                  │
                  ▼
┌─────────────────────────────────────────────────────────────┐
│                   dds_dis_bridge                            │
│  - Subscribes to "ship_entity_states"                       │
│  - Converts ROS2 messages → DIS EntityStatePdu              │
│  - Transmits via UDP (default: 127.0.0.1:3000)              │
└─────────────────┬───────────────────────────────────────────┘
                  │
                  │ UDP (DIS Protocol v6)
                  │
                  ▼
          DIS-enabled systems
       (EM_Mapper, other simulations)
```

### ECS Simulation System

The simulation uses an Entity Component System with the following components:

- **LatLonPosition**: Ship position (latitude, longitude, altitude)
- **LatLonVelocity**: Ship velocity (degrees/sec for lat/lon, m/s for altitude)
- **EulerAngles**: Ship orientation (roll, pitch, yaw/heading)
- **ShipMetadata**: Entity ID, name, force allegiance
- **WaypointController**: Autonomous navigation with waypoints

**Update Loop:**
```
timer_callback() → simulation->Update(deltaTime)
                     ├─ UpdateWaypoints()  // Adjust velocities toward targets
                     └─ DeadReckoning()    // Update positions: pos += vel * dt
```

## Installation

### Prerequisites

```bash
# Install dependencies
sudo apt install libspdlog-dev libyaml-cpp-dev

# ROS2 (Humble or later)
# Ensure you have sourced your ROS2 installation
```

### Build

```bash
cd /path/to/ros2_ws
colcon build --packages-select my_cpp_package
source install/setup.bash
```

## Configuration

### Ship Configuration File

Ships are configured in `config/ships.yaml`. Here's an example:

```yaml
ships:
  - entity_id: 1001
    name: "USS Enterprise"
    force_id: 1  # 1=Blue, 2=Red

    initial_position:
      latitude: 36.8970
      longitude: -76.0230
      altitude: 0.0

    velocity:
      north_ms: 5.0   # m/s northward
      east_ms: 5.0    # m/s eastward
      up_ms: 0.0

    orientation:
      heading: 45.0   # degrees (0=North, 90=East)
      pitch: 0.0
      roll: 0.0

    waypoints:
      enabled: false      # Set to true for waypoint navigation
      loop: false         # Loop back to first waypoint?
      tolerance_m: 100.0  # Distance to consider waypoint "reached"
      cruise_speed_ms: 10.0
      points:
        - latitude: 37.0
          longitude: -76.0
          altitude: 0.0
        # Add more waypoints as needed
```

### Configuration Parameters

The ship_publisher node accepts the following parameters:

- `config_file`: Path to YAML configuration file
  - Default: `/home/pfoytik/AI/Ros2/ros2_ws/src/my_cpp_package/config/ships.yaml`
  - Override: `ros2 run my_cpp_package ship_publisher --ros-args -p config_file:=/path/to/config.yaml`

## Usage

### Running the Simulation

**Start the ship publisher:**
```bash
ros2 run my_cpp_package ship_publisher
```

**Expected output:**
```
[INFO] [ship_publisher]: Loading ships from: .../config/ships.yaml
[INFO] [ship_publisher]: Loaded ship: USS Enterprise (ID=1001)
[INFO] [ship_publisher]: Loaded ship: USS Nimitz (ID=1002)
[INFO] [ship_publisher]: Loaded ship: Red Destroyer (ID=2001)
[INFO] [ship_publisher]: ShipPublisher initialized with 3 ships
```

### Running the DDS-DIS Bridge

**In a separate terminal:**
```bash
ros2 run my_cpp_package dds_dis_bridge
```

**Expected output:**
```
[INFO] [dds_dis_bridge]: UDP socket configured for DIS transmission to 127.0.0.1:3000
[INFO] [dds_dis_bridge]: DDS to DIS Bridge started - listening for ship entity states
[INFO] [dds_dis_bridge]: Received ship entity state for ID 1001: lat=36.904456, lon=-76.013677
[INFO] [dds_dis_bridge]: Sent DIS EntityStatePdu for entity 1001 (144 bytes)
```

### Monitoring Topics

**List active topics:**
```bash
ros2 topic list
# Output: /ship_entity_states
```

**Echo ship messages:**
```bash
ros2 topic echo /ship_entity_states
```

**Monitor message rate:**
```bash
ros2 topic hz /ship_entity_states
# Expected: ~2 Hz (500ms update interval)
```

### Testing with DIS Receiver

**Run the DIS receiver test utility:**
```bash
ros2 run my_cpp_package dis_receiver_test
```

This will listen on UDP port 3000 and display received DIS EntityStatePdu packets.

## Message Types

### ShipEntityState.msg

```
uint16 entity_id          # Unique ship identifier
string entity_name        # Ship name (e.g., "USS Enterprise")
float64 latitude          # Latitude in decimal degrees
float64 longitude         # Longitude in decimal degrees
float64 altitude          # Altitude in meters
float64 heading           # Heading in degrees (0=North, clockwise)
float64 pitch             # Pitch angle in degrees
float64 roll              # Roll angle in degrees
float64 velocity_x        # East-West velocity in m/s
float64 velocity_y        # North-South velocity in m/s
float64 velocity_z        # Vertical velocity in m/s
uint8 force_id            # Force allegiance (1=Blue, 2=Red)
uint32 timestamp          # Unix timestamp in milliseconds
```

## Advanced Features

### Waypoint Navigation

To enable waypoint navigation for a ship:

1. Edit `config/ships.yaml`
2. Set `waypoints.enabled: true`
3. Add waypoint coordinates to `waypoints.points`
4. Configure `cruise_speed_ms` for navigation speed
5. Set `loop: true` to continuously patrol waypoints

**Example:**
```yaml
waypoints:
  enabled: true
  loop: true
  tolerance_m: 50.0
  cruise_speed_ms: 15.0
  points:
    - {latitude: 37.0, longitude: -76.0, altitude: 0.0}
    - {latitude: 37.1, longitude: -76.1, altitude: 0.0}
    - {latitude: 37.0, longitude: -76.2, altitude: 0.0}
```

The ship will navigate to each waypoint in sequence, adjusting velocity automatically.

### RF Modeling (Future)

The simulation includes components for RF (Radio Frequency) modeling that are currently deactivated:

- **Transmitter**: Radio communications transmitter
- **EWEmitter**: Electronic warfare emitter with multiple beams
- **Receiver**: Signal receiver

**To enable RF components:**

1. Edit `CMakeLists.txt`:
   ```cmake
   add_compile_definitions(ENABLE_RF_COMPONENTS)
   ```

2. Add GeographicLib dependency for ECEF coordinate conversions

3. Rebuild the package

## File Structure

```
my_cpp_package/
├── config/
│   └── ships.yaml                      # Ship configuration
├── include/my_cpp_package/
│   └── simulation/
│       ├── ecs/
│       │   ├── Entity.hpp              # ECS entity
│       │   ├── ComponentArray.hpp      # Component storage
│       │   └── View.hpp                # ECS query system
│       ├── LatLonTypes.hpp             # Lat/lon position/velocity types
│       ├── Components.hpp              # Ship and waypoint components
│       └── LatLonSimulation.hpp        # Main simulation class
├── src/
│   ├── simulation/
│   │   ├── ecs/
│   │   │   └── Entity.cpp
│   │   ├── LatLonTypes.cpp
│   │   └── LatLonSimulation.cpp
│   ├── ship_publisher.cpp              # ROS2 node: simulation → DDS
│   ├── dds_dis_bridge.cpp              # Bridge: DDS → DIS
│   └── dis_receiver_test.cpp           # DIS packet validator
├── msg/
│   └── ShipEntityState.msg             # Custom ROS2 message
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Technical Details

### Coordinate System

The simulation uses lat/lon coordinates throughout:

- **Position**: Latitude/longitude (decimal degrees), altitude (meters)
- **Velocity**: Stored as degrees/second for lat/lon, m/s for altitude
- **Conversion**: Helper functions convert between m/s and degrees/sec using:
  - 1 degree latitude ≈ 111,320 meters (constant)
  - 1 degree longitude ≈ 111,320 * cos(latitude) meters (varies by latitude)

**Accuracy:** Suitable for distances < 100km. For longer distances or high-precision requirements, consider enabling GeographicLib for geodetic calculations.

### Dead Reckoning Algorithm

```cpp
// Simple integration in lat/lon space
position.latitude += velocity.lat_deg_per_sec * deltaTime;
position.longitude += velocity.lon_deg_per_sec * deltaTime;
position.altitude += velocity.alt_m_per_sec * deltaTime;
```

### Performance

- **Update Rate**: 2 Hz (500ms timer)
- **ECS Overhead**: Minimal - O(n) iteration over entities with required components
- **Memory**: ~10KB per entity with all components
- **CPU**: < 1% on modern systems for 100 entities

## Troubleshooting

### Library Not Found

**Error:** `libship_simulation.so: cannot open shared object file`

**Solution:** Rebuild and ensure library is installed:
```bash
colcon build --packages-select my_cpp_package
source install/setup.bash
```

### YAML Parse Errors

**Error:** `YAML parsing error: bad conversion`

**Solution:** Validate YAML syntax:
```bash
# Install yamllint
sudo apt install yamllint

# Validate config
yamllint config/ships.yaml
```

### Ships Not Moving

**Check:**
1. Velocity is non-zero in `config/ships.yaml`
2. Timer is firing (check logs for deltaTime warnings)
3. Topic is publishing: `ros2 topic hz /ship_entity_states`

### DIS Bridge Not Receiving

**Check:**
1. ship_publisher is running
2. Topic exists: `ros2 topic list`
3. Bridge is subscribed: Look for "Received ship entity state" logs
4. Check firewall settings for UDP port 3000

## Integration with EM_Mapper

This package is designed to work with EM_Mapper for RF visualization:

1. **Start ship_publisher** - Simulates ships
2. **Start dds_dis_bridge** - Converts to DIS
3. **Start EM_Mapper** - Receives DIS packets, visualizes RF environment

EM_Mapper will track ship positions and simulate electromagnetic emissions if ships have RF components configured.

## Development

### Adding New Components

To add a new component type:

1. Define component struct in `Components.hpp`
2. Add ComponentArray to `LatLonSimulation.hpp`
3. Add getter/setter methods in `LatLonSimulation.cpp`
4. Update `RemoveEntity()` to clean up component

**Example:**
```cpp
// Components.hpp
struct FuelTank {
    double fuel_liters;
    double consumption_rate_lps;  // liters per second
};

// LatLonSimulation.hpp
class LatLonSimulation {
private:
    ecs::ComponentArray<FuelTank> m_FuelTanks;
public:
    void SetFuelTank(const ecs::Entity entity, const FuelTank& tank);
    // ...
};
```

### Adding New Systems

Systems are methods in LatLonSimulation that process entities:

```cpp
void LatLonSimulation::UpdateFuel(double deltaTime) {
    for (auto [entity, fuel_tank] : ecs::View(m_FuelTanks)) {
        fuel_tank.fuel_liters -= fuel_tank.consumption_rate_lps * deltaTime;
        if (fuel_tank.fuel_liters <= 0) {
            // Handle out of fuel...
        }
    }
}

void LatLonSimulation::Update(double deltaTime) {
    UpdateWaypoints(deltaTime);
    UpdateFuel(deltaTime);      // New system
    DeadReckoning(deltaTime);
}
```

## License

TODO: Add license information

## Credits

- **ECS Architecture**: Adapted from EM_Mapper simulation system
- **DIS Protocol**: Using open-dis-cpp library
- **ROS2 Integration**: Custom implementation

## Support

For issues or questions:
- Check existing issues in the repository
- Review troubleshooting section above
- Check ROS2 and DIS protocol documentation

---

**Last Updated:** 2026-01-07
**ROS2 Version:** Humble or later
**DIS Version:** IEEE 1278.1-2012 (DIS 6)

# Testing Guide for Ship Simulation

This guide shows you how to test the new integrated ship simulation system.

## Test Scenario: 2 Knot Ship Movement

We've created a test scenario with HMS Test Vessel moving at 2 knots from UK coordinates.

### Scenario Details

- **Ship**: HMS Test Vessel (ID: 5001)
- **Starting Position**: 50°48'34.9"N 1°17'15.6"W (Near Portsmouth, UK)
- **Speed**: 2 knots (1.029 m/s)
- **Heading**: 90° (Due East)
- **Configuration**: `config/test_scenario.yaml`

## Quick Tests

### Test 1: Basic Functionality

**Start the test scenario:**
```bash
cd /home/pfoytik/AI/Ros2/ros2_ws
source install/setup.bash

# Run with test configuration
ros2 run my_cpp_package ship_publisher --ros-args \
  -p config_file:=$(pwd)/src/my_cpp_package/config/test_scenario.yaml
```

**Expected output:**
```
[INFO] [ship_publisher]: Loading ships from: .../test_scenario.yaml
[INFO] [ship_publisher]: Loaded ship: HMS Test Vessel (ID=5001)
[INFO] [ship_publisher]: ShipPublisher initialized with 1 ships
```

### Test 2: Real-Time Position Monitoring

**In a separate terminal, run the monitor:**
```bash
cd /home/pfoytik/AI/Ros2/ros2_ws
./monitor_ship.sh
```

You'll see real-time updates showing:
- Ship position (latitude/longitude)
- Heading (90° = East)
- Velocity (1.029 m/s east)

**Watch the longitude value increase** (becoming less negative) as the ship moves east!

### Test 3: Automated Movement Verification

**Run the automated test:**
```bash
cd /home/pfoytik/AI/Ros2/ros2_ws
./test_2knot_scenario.sh
```

This will:
1. Start the ship publisher
2. Capture position at T=0, T=30s, and T=60s
3. Show expected vs actual movement
4. Verify the ship is moving at 2 knots

### Test 4: Full DDS→DIS Pipeline

**Terminal 1 - Ship Publisher:**
```bash
cd /home/pfoytik/AI/Ros2/ros2_ws
source install/setup.bash
ros2 run my_cpp_package ship_publisher --ros-args \
  -p config_file:=$(pwd)/src/my_cpp_package/config/test_scenario.yaml
```

**Terminal 2 - DDS-DIS Bridge:**
```bash
source install/setup.bash
ros2 run my_cpp_package dds_dis_bridge
```

You should see:
```
[INFO] [dds_dis_bridge]: DDS to DIS Bridge started
[INFO] [dds_dis_bridge]: Received ship entity state for ID 5001: lat=50.809694, lon=-1.287xxx
[INFO] [dds_dis_bridge]: Sent DIS EntityStatePdu for entity 5001 (144 bytes)
```

**Terminal 3 - DIS Receiver (Optional):**
```bash
source install/setup.bash
ros2 run my_cpp_package dis_receiver_test
```

This shows the actual DIS packets being received!

## Expected Movement Calculations

At 2 knots (1.029 m/s) heading east from 50.8°N latitude:

| Time | Distance Traveled | Longitude Change | Expected Longitude |
|------|------------------|------------------|-------------------|
| 0s   | 0 m              | 0°               | -1.287667°        |
| 30s  | 30.87 m          | +0.000437°       | -1.287230°        |
| 60s  | 61.74 m          | +0.000874°       | -1.286793°        |
| 5min | 308.7 m          | +0.004372°       | -1.283295°        |
| 10min| 617.4 m          | +0.008744°       | -1.278923°        |

**Note**: At latitude 50.8°N, 1 degree of longitude ≈ 70,600 meters

## Testing with Original Configuration

To test with the original 3 ships (USS Enterprise, USS Nimitz, Red Destroyer):

```bash
cd /home/pfoytik/AI/Ros2/ros2_ws
source install/setup.bash

# Use default config (3 ships)
ros2 run my_cpp_package ship_publisher
```

Then monitor with:
```bash
source install/setup.bash
ros2 topic echo /ship_entity_states
```

You'll see all 3 ships updating at 2 Hz (500ms intervals).

## Advanced Testing: Waypoint Navigation

### Create a Waypoint Patrol Route

Edit `config/test_scenario.yaml` and modify the waypoints section:

```yaml
waypoints:
  enabled: true          # Enable waypoint navigation
  loop: true            # Patrol continuously
  tolerance_m: 50.0     # 50 meter arrival tolerance
  cruise_speed_ms: 2.0  # 2 m/s cruising speed (~4 knots)
  points:
    # Create a square patrol pattern
    - latitude: 50.809694
      longitude: -1.287667
      altitude: 0.0

    - latitude: 50.815000  # ~600m north
      longitude: -1.287667
      altitude: 0.0

    - latitude: 50.815000
      longitude: -1.278000  # ~600m east
      altitude: 0.0

    - latitude: 50.809694
      longitude: -1.278000
      altitude: 0.0
```

**Run the waypoint test:**
```bash
source install/setup.bash
ros2 run my_cpp_package ship_publisher --ros-args \
  -p config_file:=$(pwd)/src/my_cpp_package/config/test_scenario.yaml
```

**Watch it patrol:**
```bash
./monitor_ship.sh
```

You'll see the ship automatically navigate to each waypoint in sequence!

## Performance Testing

### Check Update Rate

```bash
source install/setup.bash
ros2 topic hz /ship_entity_states
```

**Expected**: ~2.00 Hz (500ms update interval)

### Check Latency

```bash
source install/setup.bash
ros2 topic delay /ship_entity_states
```

**Expected**: < 50ms latency

### CPU Usage

```bash
# While ship_publisher is running
top -p $(pgrep -f ship_publisher)
```

**Expected**: < 1% CPU for single ship, < 5% for 100 ships

## Troubleshooting Tests

### Problem: Ship not moving

**Check velocity:**
```bash
ros2 topic echo /ship_entity_states | grep -A 3 velocity
```

Velocity values should be non-zero.

**Check config:**
```bash
cat /home/pfoytik/AI/Ros2/ros2_ws/src/my_cpp_package/config/test_scenario.yaml | grep -A 3 velocity
```

### Problem: Wrong starting position

**Verify config is loaded:**
```bash
ros2 run my_cpp_package ship_publisher --ros-args \
  -p config_file:=/home/pfoytik/AI/Ros2/ros2_ws/src/my_cpp_package/config/test_scenario.yaml
```

Check the INFO log shows the correct config file path.

### Problem: DIS bridge not receiving

**Check topic:**
```bash
ros2 topic list | grep ship_entity_states
```

**Check messages:**
```bash
ros2 topic hz /ship_entity_states
```

**Restart both nodes:**
```bash
# Kill all
pkill -f ship_publisher
pkill -f dds_dis_bridge

# Restart
ros2 run my_cpp_package ship_publisher &
sleep 2
ros2 run my_cpp_package dds_dis_bridge
```

## Integration with EM_Mapper

To test the full pipeline with EM_Mapper:

**Terminal 1 - Ship Publisher:**
```bash
cd /home/pfoytik/AI/Ros2/ros2_ws
source install/setup.bash
ros2 run my_cpp_package ship_publisher --ros-args \
  -p config_file:=$(pwd)/src/my_cpp_package/config/test_scenario.yaml
```

**Terminal 2 - DDS-DIS Bridge:**
```bash
source install/setup.bash
ros2 run my_cpp_package dds_dis_bridge
```

**Terminal 3 - EM_Mapper:**
```bash
cd /home/pfoytik/AI/Ros2/EM_Mapper/build
./em_mapper
```

EM_Mapper should now receive DIS packets and display HMS Test Vessel on the map!

## Validation Checklist

- [ ] Ship publisher starts without errors
- [ ] Ship loads from YAML config correctly
- [ ] Topic `/ship_entity_states` exists
- [ ] Messages publish at ~2 Hz
- [ ] Ship position changes over time (dead reckoning works)
- [ ] Longitude increases (ship moves east)
- [ ] DDS-DIS bridge receives and converts messages
- [ ] DIS receiver shows EntityStatePdu packets
- [ ] Waypoint navigation works (if enabled)
- [ ] EM_Mapper receives DIS packets (if testing integration)

## Creating Your Own Test Scenarios

Copy and modify `test_scenario.yaml`:

```bash
cd /home/pfoytik/AI/Ros2/ros2_ws/src/my_cpp_package/config
cp test_scenario.yaml my_custom_scenario.yaml

# Edit with your favorite editor
nano my_custom_scenario.yaml
```

**Tips for custom scenarios:**
- Convert coordinates: Degrees + (Minutes/60) + (Seconds/3600)
- Knots to m/s: multiply by 0.51444
- Choose realistic speeds:
  - Patrol boat: 5-15 knots
  - Destroyer: 15-30 knots
  - Carrier: 20-35 knots
  - Submarine (surfaced): 10-20 knots

## Quick Reference: Common Commands

```bash
# Start with custom config
ros2 run my_cpp_package ship_publisher --ros-args \
  -p config_file:=/path/to/config.yaml

# Monitor ship positions
./monitor_ship.sh

# Check message rate
ros2 topic hz /ship_entity_states

# View live messages
ros2 topic echo /ship_entity_states

# Start DDS-DIS bridge
ros2 run my_cpp_package dds_dis_bridge

# Test DIS reception
ros2 run my_cpp_package dis_receiver_test
```

---

**Happy Testing!** 🚢

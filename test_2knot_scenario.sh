#!/bin/bash

# Test scenario: HMS Test Vessel moving at 2 knots from UK coordinates
# Location: 50°48'34.9"N 1°17'15.6"W
# Speed: 2 knots (1.029 m/s) heading east (90°)

cd /home/pfoytik/AI/Ros2/ros2_ws
source install/setup.bash

echo "=============================================="
echo "Test Scenario: 2 Knot Ship Movement"
echo "=============================================="
echo "Ship: HMS Test Vessel"
echo "Starting Position: 50°48'34.9\"N 1°17'15.6\"W"
echo "                   (50.809694°N, -1.287667°W)"
echo "Speed: 2 knots (1.029 m/s)"
echo "Heading: 90° (Due East)"
echo "=============================================="
echo ""

# Start ship publisher with custom config
echo "Starting ship publisher with test configuration..."
ros2 run my_cpp_package ship_publisher --ros-args \
  -p config_file:=/home/pfoytik/AI/Ros2/ros2_ws/src/my_cpp_package/config/test_scenario.yaml \
  > /tmp/ship_test.log 2>&1 &

SHIP_PUB_PID=$!
sleep 3

echo ""
echo "========================================="
echo "Position at T=0 seconds (Start)"
echo "========================================="
timeout 2 ros2 topic echo /ship_entity_states --once 2>/dev/null | grep -E "entity_name|latitude|longitude|heading|velocity" | head -10

echo ""
echo "Waiting 30 seconds for ship to move..."
echo "(At 2 knots, ship moves ~15 meters in 30 seconds)"
sleep 30

echo ""
echo "========================================="
echo "Position at T=30 seconds"
echo "========================================="
timeout 2 ros2 topic echo /ship_entity_states --once 2>/dev/null | grep -E "entity_name|latitude|longitude|heading|velocity" | head -10

echo ""
echo "Waiting another 30 seconds..."
sleep 30

echo ""
echo "========================================="
echo "Position at T=60 seconds (1 minute)"
echo "========================================="
timeout 2 ros2 topic echo /ship_entity_states --once 2>/dev/null | grep -E "entity_name|latitude|longitude|heading|velocity" | head -10

echo ""
echo "========================================="
echo "Expected Movement Analysis"
echo "========================================="
echo "Speed: 2 knots = 1.029 m/s"
echo ""
echo "After 30 seconds:"
echo "  Distance: 1.029 m/s × 30s = 30.87 meters east"
echo "  At latitude 50.8°N, 1 degree longitude ≈ 70,600 meters"
echo "  Expected longitude change: +0.000437° (eastward)"
echo ""
echo "After 60 seconds:"
echo "  Distance: 1.029 m/s × 60s = 61.74 meters east"
echo "  Expected longitude change: +0.000874° (eastward)"
echo ""
echo "Initial longitude: -1.287667°"
echo "Expected at T=30s: -1.287230°"
echo "Expected at T=60s: -1.286793°"
echo "========================================="

# Cleanup
kill $SHIP_PUB_PID 2>/dev/null
wait $SHIP_PUB_PID 2>/dev/null

echo ""
echo "Test complete! Check the longitude values above."
echo "The ship should be moving eastward (longitude becoming less negative)."

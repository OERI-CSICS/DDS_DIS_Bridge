#!/bin/bash

# Real-time ship position monitor
# Shows continuous updates of ship position

cd /home/pfoytik/AI/Ros2/ros2_ws
source install/setup.bash

echo "=============================================="
echo "Real-Time Ship Position Monitor"
echo "=============================================="
echo "Press Ctrl+C to stop"
echo ""

# Monitor ship positions in a loop
while true; do
    clear
    echo "=============================================="
    echo "Ship Positions - $(date +%H:%M:%S)"
    echo "=============================================="
    echo ""

    # Get latest message and format it nicely
    timeout 1 ros2 topic echo /ship_entity_states --once 2>/dev/null | \
    awk '
    /entity_id:/ { id=$2 }
    /entity_name:/ { name=$0; gsub(/.*: /, "", name); gsub(/'\''/, "", name) }
    /latitude:/ { lat=$2 }
    /longitude:/ { lon=$2 }
    /heading:/ { hdg=$2 }
    /velocity_x:/ { vel_e=$2 }
    /velocity_y:/ { vel_n=$2 }
    /---/ {
        if (id != "") {
            printf "Ship ID: %s (%s)\n", id, name
            printf "  Position: %.6f°N, %.6f°W\n", lat, -lon
            printf "  Heading:  %.1f°\n", hdg
            printf "  Velocity: %.3f m/s north, %.3f m/s east\n", vel_n, vel_e
            printf "\n"
            id=""
        }
    }
    '

    echo "=============================================="
    echo "Updating every 2 seconds... (Ctrl+C to stop)"

    sleep 2
done

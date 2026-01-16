#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "LatLonTypes.hpp"

// -------------------- Core Ship Components --------------------

// Metadata for mapping ECS entities to ROS2 messages and ship information
struct ShipMetadata {
    uint16_t entity_id;    // ROS2 message entity ID
    std::string name;      // Ship name (e.g., "USS Enterprise")
    uint8_t force_id;      // Force allegiance (1=Blue, 2=Red, etc.)

    ShipMetadata() : entity_id(0), name(""), force_id(0) {}
    ShipMetadata(uint16_t id, const std::string& ship_name, uint8_t force)
        : entity_id(id), name(ship_name), force_id(force) {}
};

// Waypoint controller for autonomous navigation
struct WaypointController {
    std::vector<LatLonPosition> waypoints;  // List of waypoints to navigate
    size_t current_waypoint_index;          // Current target waypoint
    double waypoint_tolerance_meters;       // Distance to consider waypoint "reached"
    double cruise_speed_ms;                 // Speed to travel between waypoints (m/s)
    bool loop;                              // Return to first waypoint when done?
    bool active;                            // Waypoint control enabled?

    WaypointController()
        : current_waypoint_index(0),
          waypoint_tolerance_meters(100.0),
          cruise_speed_ms(10.0),
          loop(false),
          active(false) {}
};

// -------------------- RF Components (for future activation) --------------------

#ifdef ENABLE_RF_COMPONENTS

// Simple 3D vector for relative positions (offsets from entity center)
// Used only for RF component relative locations
struct Vec3 {
    double x, y, z;

    Vec3() : x(0.0), y(0.0), z(0.0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// Antenna pattern types
enum class AntennaPatternType : uint8_t {
    OmniDirectional = 0,  // Isotropic / Omni-directional antenna
    Directional = 1,      // Directional antenna (e.g., Yagi, Parabolic dish)
    Patterned = 2         // Patterned antenna (e.g., phased array)
};

// Represents an EW Emitter beam
struct Beam {
    float frequency;
    float frequency_range;
    float radiated_power;
    float pulse_repetition_frequency;
    float pulse_width;
    float azimuth_center;
    float azimuth_sweep;
    float elevation_center;
    float elevation_sweep;
    float beam_sweep_sync;
};

// Represents an EW Emitter with one or more beams
struct EWEmitter {
    Vec3 relative_location;  // Offset from entity center
    std::vector<Beam> beams;
};

// Represents a radio transmitter
struct Transmitter {
    Vec3 relative_location;  // Offset from entity center
    float transmit_frequency;
    float transmit_bandwidth;
    float transmit_power;
    float transmit_state;
    AntennaPatternType antenna_pattern;
};

// Represents a radio receiver
struct Receiver {
    Vec3 relative_location;  // Offset from entity center
    float receive_frequency;
    float receive_bandwidth;
    float receive_sensitivity;
    AntennaPatternType antenna_pattern;
};

#endif  // ENABLE_RF_COMPONENTS

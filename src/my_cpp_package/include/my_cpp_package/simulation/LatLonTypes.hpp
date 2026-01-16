#pragma once

#include <cmath>

constexpr double PI = M_PI;
constexpr double METERS_PER_DEGREE = 111320.0;  // Approximate meters per degree latitude

// Position in lat/lon space
struct LatLonPosition {
    double latitude;   // decimal degrees
    double longitude;  // decimal degrees
    double altitude;   // meters above sea level

    LatLonPosition() : latitude(0.0), longitude(0.0), altitude(0.0) {}
    LatLonPosition(double lat, double lon, double alt)
        : latitude(lat), longitude(lon), altitude(alt) {}
};

// Velocity in lat/lon space (degrees/second)
struct LatLonVelocity {
    double lat_deg_per_sec;  // degrees/sec latitude change
    double lon_deg_per_sec;  // degrees/sec longitude change
    double alt_m_per_sec;    // meters/sec altitude change

    LatLonVelocity() : lat_deg_per_sec(0.0), lon_deg_per_sec(0.0), alt_m_per_sec(0.0) {}
    LatLonVelocity(double lat_dps, double lon_dps, double alt_mps)
        : lat_deg_per_sec(lat_dps), lon_deg_per_sec(lon_dps), alt_m_per_sec(alt_mps) {}
};

// Euler angles in radians (Roll, Pitch, Yaw)
// For maritime vessels: Yaw corresponds to heading (0 = North, clockwise positive)
class EulerAngles {
   public:
    EulerAngles();
    EulerAngles(double roll, double pitch, double yaw);
    EulerAngles(const EulerAngles& other);
    EulerAngles(EulerAngles&& other) noexcept;
    ~EulerAngles();

    double GetRoll() const { return m_Roll; }
    double GetPitch() const { return m_Pitch; }
    double GetYaw() const { return m_Yaw; }

    void SetRoll(double roll) { m_Roll = roll; }
    void SetPitch(double pitch) { m_Pitch = pitch; }
    void SetYaw(double yaw) { m_Yaw = yaw; }

    EulerAngles& operator=(const EulerAngles& other);
    EulerAngles& operator=(EulerAngles&& other) noexcept;

    bool operator==(const EulerAngles& other) const {
        return m_Roll == other.m_Roll && m_Pitch == other.m_Pitch &&
               m_Yaw == other.m_Yaw;
    }

   private:
    double m_Roll;   // radians
    double m_Pitch;  // radians
    double m_Yaw;    // radians (heading for ships)
};

// Helper functions for lat/lon simulation

// Convert velocity from meters/second to degrees/second
// vel_north_ms: velocity in north direction (m/s)
// vel_east_ms: velocity in east direction (m/s)
// vel_up_ms: velocity in up direction (m/s)
// current_lat: current latitude for longitude correction
LatLonVelocity VelocityMetersToLatLon(double vel_north_ms, double vel_east_ms,
                                      double vel_up_ms, double current_lat);

// Calculate approximate distance in meters between two lat/lon positions
// Uses Euclidean approximation - suitable for distances < 100km
double CalculateDistanceMeters(const LatLonPosition& a, const LatLonPosition& b);

// Calculate velocity vector toward target at specified speed
// Returns velocity in degrees/second pointing from current to target
LatLonVelocity CalculateVelocityToward(const LatLonPosition& current,
                                       const LatLonPosition& target,
                                       double speed_ms);

// Convert degrees to radians
inline double DegreesToRadians(double degrees) {
    return degrees * PI / 180.0;
}

// Convert radians to degrees
inline double RadiansToDegrees(double radians) {
    return radians * 180.0 / PI;
}

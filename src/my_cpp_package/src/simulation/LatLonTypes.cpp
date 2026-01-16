#include "my_cpp_package/simulation/LatLonTypes.hpp"

#include <cmath>

// -------------------- EulerAngles Implementation --------------------

EulerAngles::EulerAngles() : m_Roll(0.0), m_Pitch(0.0), m_Yaw(0.0) {}

EulerAngles::EulerAngles(double roll, double pitch, double yaw)
    : m_Roll(roll), m_Pitch(pitch), m_Yaw(yaw) {}

EulerAngles::EulerAngles(const EulerAngles& other)
    : m_Roll(other.m_Roll), m_Pitch(other.m_Pitch), m_Yaw(other.m_Yaw) {}

EulerAngles::EulerAngles(EulerAngles&& other) noexcept
    : m_Roll(other.m_Roll), m_Pitch(other.m_Pitch), m_Yaw(other.m_Yaw) {}

EulerAngles::~EulerAngles() {}

EulerAngles& EulerAngles::operator=(const EulerAngles& other) {
    if (this != &other) {
        m_Roll = other.m_Roll;
        m_Pitch = other.m_Pitch;
        m_Yaw = other.m_Yaw;
    }
    return *this;
}

EulerAngles& EulerAngles::operator=(EulerAngles&& other) noexcept {
    if (this != &other) {
        m_Roll = other.m_Roll;
        m_Pitch = other.m_Pitch;
        m_Yaw = other.m_Yaw;
    }
    return *this;
}

// -------------------- Helper Functions --------------------

LatLonVelocity VelocityMetersToLatLon(double vel_north_ms, double vel_east_ms,
                                      double vel_up_ms, double current_lat) {
    // Latitude: 1 degree ≈ 111,320 meters everywhere
    double lat_deg_per_sec = vel_north_ms / METERS_PER_DEGREE;

    // Longitude: 1 degree varies with latitude (cosine correction)
    double lon_deg_per_sec = vel_east_ms / (METERS_PER_DEGREE * std::cos(DegreesToRadians(current_lat)));

    return LatLonVelocity(lat_deg_per_sec, lon_deg_per_sec, vel_up_ms);
}

double CalculateDistanceMeters(const LatLonPosition& a, const LatLonPosition& b) {
    // Euclidean approximation in local tangent plane
    // Suitable for distances < 100km

    // Latitude difference in meters
    double lat_diff_m = (b.latitude - a.latitude) * METERS_PER_DEGREE;

    // Longitude difference in meters (with cosine correction at midpoint latitude)
    double mid_lat = (a.latitude + b.latitude) / 2.0;
    double lon_diff_m = (b.longitude - a.longitude) * METERS_PER_DEGREE * std::cos(DegreesToRadians(mid_lat));

    // Altitude difference
    double alt_diff_m = b.altitude - a.altitude;

    // Euclidean distance
    return std::sqrt(lat_diff_m * lat_diff_m + lon_diff_m * lon_diff_m + alt_diff_m * alt_diff_m);
}

LatLonVelocity CalculateVelocityToward(const LatLonPosition& current,
                                       const LatLonPosition& target,
                                       double speed_ms) {
    // Calculate direction vector in meters
    double lat_diff_m = (target.latitude - current.latitude) * METERS_PER_DEGREE;
    double lon_diff_m = (target.longitude - current.longitude) *
                        METERS_PER_DEGREE * std::cos(DegreesToRadians(current.latitude));
    double alt_diff_m = target.altitude - current.altitude;

    // Calculate distance
    double distance = std::sqrt(lat_diff_m * lat_diff_m + lon_diff_m * lon_diff_m + alt_diff_m * alt_diff_m);

    // Normalize and scale by speed
    if (distance < 0.001) {
        // Already at target, return zero velocity
        return LatLonVelocity(0.0, 0.0, 0.0);
    }

    double vel_north_ms = (lat_diff_m / distance) * speed_ms;
    double vel_east_ms = (lon_diff_m / distance) * speed_ms;
    double vel_up_ms = (alt_diff_m / distance) * speed_ms;

    // Convert to degrees/second
    return VelocityMetersToLatLon(vel_north_ms, vel_east_ms, vel_up_ms, current.latitude);
}

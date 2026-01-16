#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "my_cpp_package/msg/ship_entity_state.hpp"
#include "my_cpp_package/simulation/LatLonSimulation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

class ShipPublisher : public rclcpp::Node {
   public:
    ShipPublisher() : Node("ship_publisher") {
        // Create publisher
        publisher_ = this->create_publisher<my_cpp_package::msg::ShipEntityState>(
            "ship_entity_states", 10);

        // Create simulation
        simulation_ = std::make_unique<LatLonSimulation>();

        // Declare and get configuration file parameter
        this->declare_parameter("config_file",
                                "/home/pfoytik/AI/Ros2/ros2_ws/src/my_cpp_package/config/ships.yaml");
        std::string config_file = this->get_parameter("config_file").as_string();

        // Load ships from configuration
        LoadShipsFromConfig(config_file);

        // Initialize last update time
        last_update_time_ = this->now();

        // Create timer (500ms = 2 Hz)
        timer_ = this->create_wall_timer(
            500ms, std::bind(&ShipPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "ShipPublisher initialized with %zu ships",
                    simulation_->GetAllEntities().size());
    }

   private:
    void LoadShipsFromConfig(const std::string& config_file) {
        try {
            RCLCPP_INFO(this->get_logger(), "Loading ships from: %s",
                        config_file.c_str());

            YAML::Node config = YAML::LoadFile(config_file);

            if (!config["ships"]) {
                RCLCPP_ERROR(this->get_logger(),
                             "No 'ships' section in config file");
                return;
            }

            for (const auto& ship_node : config["ships"]) {
                // Create entity
                ecs::Entity entity = simulation_->AddEntity();

                // Parse ship metadata
                uint16_t entity_id = ship_node["entity_id"].as<uint16_t>();
                std::string name = ship_node["name"].as<std::string>();
                uint8_t force_id = ship_node["force_id"].as<uint8_t>();

                ShipMetadata metadata(entity_id, name, force_id);
                simulation_->SetMetadata(entity, metadata);

                // Parse initial position
                LatLonPosition position(
                    ship_node["initial_position"]["latitude"].as<double>(),
                    ship_node["initial_position"]["longitude"].as<double>(),
                    ship_node["initial_position"]["altitude"].as<double>());
                simulation_->SetPosition(entity, position);

                // Parse velocity (convert from m/s to degrees/sec)
                double vel_north = ship_node["velocity"]["north_ms"].as<double>();
                double vel_east = ship_node["velocity"]["east_ms"].as<double>();
                double vel_up = ship_node["velocity"]["up_ms"].as<double>();

                LatLonVelocity velocity =
                    VelocityMetersToLatLon(vel_north, vel_east, vel_up,
                                           position.latitude);
                simulation_->SetVelocity(entity, velocity);

                // Parse orientation (convert degrees to radians)
                double heading = ship_node["orientation"]["heading"].as<double>();
                double pitch = ship_node["orientation"]["pitch"].as<double>();
                double roll = ship_node["orientation"]["roll"].as<double>();

                EulerAngles orientation(DegreesToRadians(roll),
                                        DegreesToRadians(pitch),
                                        DegreesToRadians(heading));
                simulation_->SetOrientation(entity, orientation);

                // Parse waypoint controller (if present)
                if (ship_node["waypoints"]) {
                    WaypointController waypoints;
                    waypoints.active =
                        ship_node["waypoints"]["enabled"].as<bool>(false);
                    waypoints.loop =
                        ship_node["waypoints"]["loop"].as<bool>(false);
                    waypoints.waypoint_tolerance_meters =
                        ship_node["waypoints"]["tolerance_m"].as<double>(100.0);
                    waypoints.cruise_speed_ms =
                        ship_node["waypoints"]["cruise_speed_ms"].as<double>(10.0);

                    // Parse waypoint list
                    if (ship_node["waypoints"]["points"]) {
                        for (const auto& wp :
                             ship_node["waypoints"]["points"]) {
                            LatLonPosition waypoint(
                                wp["latitude"].as<double>(),
                                wp["longitude"].as<double>(),
                                wp["altitude"].as<double>());
                            waypoints.waypoints.push_back(waypoint);
                        }
                    }

                    simulation_->SetWaypointController(entity, waypoints);
                }

                RCLCPP_INFO(this->get_logger(), "Loaded ship: %s (ID=%d)",
                            name.c_str(), entity_id);
            }

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading config: %s",
                         e.what());
        }
    }

    void timer_callback() {
        // Calculate delta time
        auto current_time = this->now();
        double deltaTime = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        // Update simulation
        simulation_->Update(deltaTime);

        // Publish all ship states
        for (const auto& entity : simulation_->GetAllEntities()) {
            PublishShipState(entity);
        }
    }

    void PublishShipState(const ecs::Entity& entity) {
        // Get components from simulation
        auto pos_opt = simulation_->GetPosition(entity);
        auto vel_opt = simulation_->GetVelocity(entity);
        auto orient_opt = simulation_->GetOrientation(entity);
        auto metadata_opt = simulation_->GetMetadata(entity);

        // Check if entity has all required components
        if (!pos_opt || !vel_opt || !orient_opt || !metadata_opt) {
            RCLCPP_WARN(this->get_logger(),
                        "Entity %d missing required components, skipping",
                        entity.ID());
            return;
        }

        const LatLonPosition& position = **pos_opt;
        const LatLonVelocity& velocity = **vel_opt;
        const EulerAngles& orientation = **orient_opt;
        const ShipMetadata& metadata = **metadata_opt;

        // Create ROS2 message
        my_cpp_package::msg::ShipEntityState msg;

        msg.entity_id = metadata.entity_id;
        msg.entity_name = metadata.name;
        msg.latitude = position.latitude;
        msg.longitude = position.longitude;
        msg.altitude = position.altitude;

        // Convert radians to degrees for ROS message
        msg.heading = RadiansToDegrees(orientation.GetYaw());
        msg.pitch = RadiansToDegrees(orientation.GetPitch());
        msg.roll = RadiansToDegrees(orientation.GetRoll());

        // Convert velocity from degrees/sec to m/s for ROS message
        // Note: This is approximate reverse conversion
        msg.velocity_y = velocity.lat_deg_per_sec * 111320.0;  // North/South
        msg.velocity_x = velocity.lon_deg_per_sec *
                         (111320.0 * std::cos(DegreesToRadians(position.latitude)));  // East/West
        msg.velocity_z = velocity.alt_m_per_sec;  // Up/Down

        msg.force_id = metadata.force_id;

        // Set timestamp
        msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();

        // Publish message
        publisher_->publish(msg);

        RCLCPP_DEBUG(this->get_logger(), "Published ship %s: lat=%.6f, lon=%.6f",
                     metadata.name.c_str(), position.latitude,
                     position.longitude);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_cpp_package::msg::ShipEntityState>::SharedPtr
        publisher_;
    std::unique_ptr<LatLonSimulation> simulation_;
    rclcpp::Time last_update_time_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShipPublisher>());
    rclcpp::shutdown();
    return 0;
}

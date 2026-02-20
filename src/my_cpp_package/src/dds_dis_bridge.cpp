#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "my_cpp_package/msg/ship_entity_state.hpp"

// DIS library includes
#include <dis6/EntityStatePdu.h>
#include <dis6/utils/DataStream.h>
#include <dis6/Vector3Double.h>
#include <dis6/Vector3Float.h>
#include <dis6/Orientation.h>
#include <dis6/EntityID.h>
#include <dis6/EntityType.h>
#include <dis6/DeadReckoningParameter.h>

// Simple UDP socket for DIS transmission
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class DDSToDISBridge : public rclcpp::Node
{
public:
  DDSToDISBridge()
  : Node("dds_dis_bridge")
  {
    // Declare parameters with defaults
    this->declare_parameter<std::string>("dis_address", "10.253.140.255");  // eno1 broadcast
    this->declare_parameter<int>("dis_port", 3000);
    this->declare_parameter<double>("position_threshold_meters", 10.0);
    this->declare_parameter<bool>("use_multicast", false);  // Disabled by default for loopback
    
    subscription_ = this->create_subscription<my_cpp_package::msg::ShipEntityState>(
      "ship_entity_states", 10,
      std::bind(&DDSToDISBridge::topic_callback, this, std::placeholders::_1));

    // Setup UDP socket for DIS transmission
    setup_udp_socket();
    
    RCLCPP_INFO(this->get_logger(), "DDS to DIS Bridge started - listening for ship entity states");
  }
  
  ~DDSToDISBridge()
  {
    if (socket_fd_ > 0) {
      close(socket_fd_);
    }
  }

private:
  void setup_udp_socket()
  {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
      return;
    }

    // Enable broadcast
    int broadcast_enable = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to enable SO_BROADCAST");
    }

    // Get parameters
    auto dis_address = this->get_parameter("dis_address").as_string();
    auto dis_port = this->get_parameter("dis_port").as_int();
    auto use_multicast = this->get_parameter("use_multicast").as_bool();
    
    // Set up the DIS address
    memset(&dis_addr_, 0, sizeof(dis_addr_));
    dis_addr_.sin_family = AF_INET;
    dis_addr_.sin_port = htons(dis_port);
    dis_addr_.sin_addr.s_addr = inet_addr(dis_address.c_str());
    
    // Configure multicast if enabled
    if (use_multicast && dis_address != "127.0.0.1") {
      int ttl = 1;  // Local network only
      if (setsockopt(socket_fd_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to set multicast TTL");
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "UDP socket configured for DIS transmission to %s:%ld (multicast: %s)",
                dis_address.c_str(), dis_port, use_multicast ? "enabled" : "disabled");
  }

  void topic_callback(const my_cpp_package::msg::ShipEntityState & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received ship entity state for ID %d: lat=%.6f, lon=%.6f", 
                msg.entity_id, msg.latitude, msg.longitude);
    
    // Check for significant position change to trigger DIS transmission
    if (should_transmit_dis(msg)) {
      send_dis_entity_state(msg);
    }
  }

  bool should_transmit_dis(const my_cpp_package::msg::ShipEntityState & msg) const
  {
    // Store last known positions for each entity
    static std::map<uint16_t, std::pair<double, double>> last_positions;
    
    auto it = last_positions.find(msg.entity_id);
    bool should_send = false;
    
    if (it == last_positions.end()) {
      // First time seeing this entity
      should_send = true;
    } else {
      // Get configurable threshold
      auto threshold = this->get_parameter("position_threshold_meters").as_double();
      
      // Check if position changed significantly
      double lat_diff = msg.latitude - it->second.first;
      double lon_diff = msg.longitude - it->second.second;
      
      // Convert to approximate meters (rough calculation)
      double lat_meters = lat_diff * 111320.0;
      double lon_meters = lon_diff * 111320.0 * cos(msg.latitude * M_PI / 180.0);
      double distance = sqrt(lat_meters * lat_meters + lon_meters * lon_meters);
      
      if (distance > threshold) {
        should_send = true;
      }
    }
    
    if (should_send) {
      last_positions[msg.entity_id] = std::make_pair(msg.latitude, msg.longitude);
    }
    
    return should_send;
  }

  void send_dis_entity_state(const my_cpp_package::msg::ShipEntityState & msg) const
  {
    // Create DIS EntityStatePdu
    DIS::EntityStatePdu entity_pdu;
    
    // Set basic PDU info
    entity_pdu.setProtocolVersion(6);
    entity_pdu.setExerciseID(1);
    entity_pdu.setTimestamp(msg.timestamp);
    
    // Set Entity ID
    DIS::EntityID entity_id;
    entity_id.setSite(0);
    entity_id.setApplication(1);
    entity_id.setEntity(msg.entity_id);
    entity_pdu.setEntityID(entity_id);
    
    // Set Entity Type (Naval platform - ship)
    DIS::EntityType entity_type;
    entity_type.setEntityKind(1);        // Platform
    entity_type.setDomain(3);            // Surface (naval)
    entity_type.setCountry(225);         // United States
    entity_type.setCategory(6);          // Guided missile destroyer
    entity_type.setSubcategory(1);       // Arleigh Burke class
    entity_type.setSpecific(0);
    entity_type.setExtra(0);
    entity_pdu.setEntityType(entity_type);
    
    // Set force ID
    entity_pdu.setForceId(msg.force_id);
    
    // Convert geodetic (lat/lon/alt) to DIS ECEF coordinates (WGS84)
    const double a  = 6378137.0;           // WGS84 semi-major axis (meters)
    const double e2 = 0.00669437999014;    // WGS84 first eccentricity squared
    double lat_rad = msg.latitude  * M_PI / 180.0;
    double lon_rad = msg.longitude * M_PI / 180.0;
    double N = a / sqrt(1.0 - e2 * sin(lat_rad) * sin(lat_rad));
    DIS::Vector3Double position;
    position.setX((N + msg.altitude) * cos(lat_rad) * cos(lon_rad));
    position.setY((N + msg.altitude) * cos(lat_rad) * sin(lon_rad));
    position.setZ((N * (1.0 - e2) + msg.altitude) * sin(lat_rad));
    entity_pdu.setEntityLocation(position);
    
    // Set orientation 
    DIS::Orientation orientation;
    orientation.setPsi(msg.heading * M_PI / 180.0);    // Convert degrees to radians
    orientation.setTheta(msg.pitch * M_PI / 180.0);
    orientation.setPhi(msg.roll * M_PI / 180.0);
    entity_pdu.setEntityOrientation(orientation);
    
    // Convert velocity from local ENU (east/north/up) to ECEF (WGS84)
    double v_east  = msg.velocity_x;
    double v_north = msg.velocity_y;
    double v_up    = msg.velocity_z;
    DIS::Vector3Float velocity;
    velocity.setX(-sin(lon_rad)*v_east - sin(lat_rad)*cos(lon_rad)*v_north + cos(lat_rad)*cos(lon_rad)*v_up);
    velocity.setY( cos(lon_rad)*v_east - sin(lat_rad)*sin(lon_rad)*v_north + cos(lat_rad)*sin(lon_rad)*v_up);
    velocity.setZ( cos(lat_rad)*v_north + sin(lat_rad)*v_up);
    entity_pdu.setEntityLinearVelocity(velocity);
    
    // Set dead reckoning parameters
    DIS::DeadReckoningParameter drp;
    drp.setDeadReckoningAlgorithm(2);  // DRM_FPW (Fixed velocity, world coordinates)
    entity_pdu.setDeadReckoningParameters(drp);
    
    // Set entity appearance (0 = no special appearance)
    entity_pdu.setEntityAppearance(0);
    
    // Set capabilities (0 = none)
    entity_pdu.setCapabilities(0);
    
    // Set the PDU length
    entity_pdu.setLength(entity_pdu.getMarshalledSize());
    
    // Marshall the PDU to a data stream
    DIS::DataStream buffer(DIS::BIG);  // Big endian for network transmission
    entity_pdu.marshal(buffer);
    
    // Send via UDP
    if (socket_fd_ > 0) {
      ssize_t bytes_sent = sendto(socket_fd_, &buffer[0], buffer.size(), 0, 
                                  (struct sockaddr*)&dis_addr_, sizeof(dis_addr_));
      if (bytes_sent < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send DIS packet");
      } else {
        RCLCPP_INFO(this->get_logger(), "Sent DIS EntityStatePdu for entity %d (%zu bytes)", 
                    msg.entity_id, buffer.size());
      }
    }
  }

  rclcpp::Subscription<my_cpp_package::msg::ShipEntityState>::SharedPtr subscription_;
  int socket_fd_;
  struct sockaddr_in dis_addr_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DDSToDISBridge>());
  rclcpp::shutdown();
  return 0;
}
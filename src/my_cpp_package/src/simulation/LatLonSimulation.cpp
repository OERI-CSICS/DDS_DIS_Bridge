#include "my_cpp_package/simulation/LatLonSimulation.hpp"

#include <spdlog/spdlog.h>

LatLonSimulation::LatLonSimulation() {}
LatLonSimulation::~LatLonSimulation() {}

// -------------------- Entity Management --------------------

ecs::Entity LatLonSimulation::AddEntity() {
    // Reuse dead entities if available
    if (!m_DeadEntities.empty()) {
        uint32_t id = m_DeadEntities.back();
        m_DeadEntities.pop_back();
        ecs::Entity entity(m_Generations.at(id), id);
        m_Entities.at(id) = entity;
        spdlog::debug("Reused entity: ID={}, Generation={}", id,
                      m_Generations.at(id));
        return entity;
    } else {
        // Create new entity
        uint32_t id = m_Entities.size();
        m_Generations.push_back(0);
        ecs::Entity entity(0, id);
        m_Entities.push_back(entity);
        spdlog::debug("Created entity: ID={}, Generation=0", id);
        return entity;
    }
}

void LatLonSimulation::RemoveEntity(const ecs::Entity entity) {
    uint32_t id = entity.ID();
    m_Generations.at(id)++;
    m_DeadEntities.push_back(id);

    // Remove all components associated with this entity
    if (m_Positions.Has(entity)) {
        m_Positions.Remove(entity);
    }
    if (m_Velocities.Has(entity)) {
        m_Velocities.Remove(entity);
    }
    if (m_Orientations.Has(entity)) {
        m_Orientations.Remove(entity);
    }
    if (m_Metadatas.Has(entity)) {
        m_Metadatas.Remove(entity);
    }
    if (m_WaypointControllers.Has(entity)) {
        m_WaypointControllers.Remove(entity);
    }

#ifdef ENABLE_RF_COMPONENTS
    if (m_Transmitters.Has(entity)) {
        m_Transmitters.Remove(entity);
    }
    if (m_EWEmitters.Has(entity)) {
        m_EWEmitters.Remove(entity);
    }
#endif

    spdlog::debug("Removed entity: ID={}", id);
}

const std::vector<ecs::Entity>& LatLonSimulation::GetAllEntities() const {
    return m_Entities;
}

// -------------------- Main Update Loop --------------------

void LatLonSimulation::Update(double deltaTime) {
    // First, update waypoint controllers to adjust velocities
    UpdateWaypoints(deltaTime);

    // Then apply dead reckoning to update positions
    DeadReckoning(deltaTime);
}

// -------------------- Simulation Systems --------------------

void LatLonSimulation::UpdateWaypoints(double deltaTime) {
    // Query for entities with position, velocity, and waypoint controller
    for (auto [entity, position, velocity, waypoints] :
         ecs::View(m_Positions, m_Velocities, m_WaypointControllers)) {

        // Skip if waypoint control is not active or no waypoints defined
        if (!waypoints.active || waypoints.waypoints.empty()) {
            continue;
        }

        // Check if we're beyond the waypoint list
        if (waypoints.current_waypoint_index >= waypoints.waypoints.size()) {
            if (waypoints.loop) {
                waypoints.current_waypoint_index = 0;
            } else {
                waypoints.active = false;
                spdlog::info("Entity {} completed waypoint navigation", entity.ID());
                continue;
            }
        }

        // Get current target waypoint
        const LatLonPosition& target =
            waypoints.waypoints[waypoints.current_waypoint_index];

        // Calculate distance to target
        double distance = CalculateDistanceMeters(position, target);

        // Check if we've reached the waypoint
        if (distance < waypoints.waypoint_tolerance_meters) {
            spdlog::debug("Entity {} reached waypoint {} at distance {:.2f}m",
                          entity.ID(), waypoints.current_waypoint_index,
                          distance);
            waypoints.current_waypoint_index++;

            // Check if we've completed all waypoints
            if (waypoints.current_waypoint_index >= waypoints.waypoints.size()) {
                if (waypoints.loop) {
                    waypoints.current_waypoint_index = 0;
                } else {
                    waypoints.active = false;
                    // Set velocity to zero when navigation complete
                    velocity = LatLonVelocity(0.0, 0.0, 0.0);
                    spdlog::info("Entity {} completed waypoint navigation",
                                 entity.ID());
                }
            }
        } else {
            // Update velocity to move toward target
            velocity = CalculateVelocityToward(position, target,
                                               waypoints.cruise_speed_ms);
        }
    }
}

void LatLonSimulation::DeadReckoning(double deltaTime) {
    // Query for entities with both position and velocity
    for (auto [entity, position, velocity] :
         ecs::View(m_Positions, m_Velocities)) {
        // Update position based on velocity
        position.latitude += velocity.lat_deg_per_sec * deltaTime;
        position.longitude += velocity.lon_deg_per_sec * deltaTime;
        position.altitude += velocity.alt_m_per_sec * deltaTime;
    }
}

// -------------------- Component Setters --------------------

// Helper template for adding/updating components
template <typename T>
void AddComponent(ecs::ComponentArray<T>& componentArray,
                  const ecs::Entity entity, const T& component) {
    if (componentArray.Has(entity)) {
        componentArray.Get(entity) = component;
    } else {
        componentArray.Insert(entity, component);
    }
}

void LatLonSimulation::SetPosition(const ecs::Entity entity,
                                   const LatLonPosition& position) {
    AddComponent(m_Positions, entity, position);
}

void LatLonSimulation::SetVelocity(const ecs::Entity entity,
                                   const LatLonVelocity& velocity) {
    AddComponent(m_Velocities, entity, velocity);
}

void LatLonSimulation::SetOrientation(const ecs::Entity entity,
                                      const EulerAngles& orientation) {
    AddComponent(m_Orientations, entity, orientation);
}

void LatLonSimulation::SetMetadata(const ecs::Entity entity,
                                   const ShipMetadata& metadata) {
    AddComponent(m_Metadatas, entity, metadata);
}

void LatLonSimulation::SetWaypointController(
    const ecs::Entity entity, const WaypointController& controller) {
    AddComponent(m_WaypointControllers, entity, controller);
}

// -------------------- Component Removers --------------------

void LatLonSimulation::RemovePosition(const ecs::Entity entity) {
    if (m_Positions.Has(entity)) {
        m_Positions.Remove(entity);
    }
}

void LatLonSimulation::RemoveVelocity(const ecs::Entity entity) {
    if (m_Velocities.Has(entity)) {
        m_Velocities.Remove(entity);
    }
}

void LatLonSimulation::RemoveOrientation(const ecs::Entity entity) {
    if (m_Orientations.Has(entity)) {
        m_Orientations.Remove(entity);
    }
}

void LatLonSimulation::RemoveMetadata(const ecs::Entity entity) {
    if (m_Metadatas.Has(entity)) {
        m_Metadatas.Remove(entity);
    }
}

void LatLonSimulation::RemoveWaypointController(const ecs::Entity entity) {
    if (m_WaypointControllers.Has(entity)) {
        m_WaypointControllers.Remove(entity);
    }
}

// -------------------- Component Getters (const) --------------------

std::optional<const LatLonPosition*> LatLonSimulation::GetPosition(
    const ecs::Entity entity) const {
    if (m_Positions.Has(entity)) {
        return &m_Positions.Get(entity);
    }
    return std::nullopt;
}

std::optional<const LatLonVelocity*> LatLonSimulation::GetVelocity(
    const ecs::Entity entity) const {
    if (m_Velocities.Has(entity)) {
        return &m_Velocities.Get(entity);
    }
    return std::nullopt;
}

std::optional<const EulerAngles*> LatLonSimulation::GetOrientation(
    const ecs::Entity entity) const {
    if (m_Orientations.Has(entity)) {
        return &m_Orientations.Get(entity);
    }
    return std::nullopt;
}

std::optional<const ShipMetadata*> LatLonSimulation::GetMetadata(
    const ecs::Entity entity) const {
    if (m_Metadatas.Has(entity)) {
        return &m_Metadatas.Get(entity);
    }
    return std::nullopt;
}

std::optional<const WaypointController*>
LatLonSimulation::GetWaypointController(const ecs::Entity entity) const {
    if (m_WaypointControllers.Has(entity)) {
        return &m_WaypointControllers.Get(entity);
    }
    return std::nullopt;
}

// -------------------- Component Getters (mutable) --------------------

std::optional<LatLonPosition*> LatLonSimulation::GetPosition(
    const ecs::Entity entity) {
    if (m_Positions.Has(entity)) {
        return &m_Positions.Get(entity);
    }
    return std::nullopt;
}

std::optional<LatLonVelocity*> LatLonSimulation::GetVelocity(
    const ecs::Entity entity) {
    if (m_Velocities.Has(entity)) {
        return &m_Velocities.Get(entity);
    }
    return std::nullopt;
}

std::optional<EulerAngles*> LatLonSimulation::GetOrientation(
    const ecs::Entity entity) {
    if (m_Orientations.Has(entity)) {
        return &m_Orientations.Get(entity);
    }
    return std::nullopt;
}

std::optional<ShipMetadata*> LatLonSimulation::GetMetadata(
    const ecs::Entity entity) {
    if (m_Metadatas.Has(entity)) {
        return &m_Metadatas.Get(entity);
    }
    return std::nullopt;
}

std::optional<WaypointController*> LatLonSimulation::GetWaypointController(
    const ecs::Entity entity) {
    if (m_WaypointControllers.Has(entity)) {
        return &m_WaypointControllers.Get(entity);
    }
    return std::nullopt;
}

// -------------------- Component Array Accessors --------------------

ecs::ComponentArray<LatLonPosition>& LatLonSimulation::GetPositions() {
    return m_Positions;
}

ecs::ComponentArray<LatLonVelocity>& LatLonSimulation::GetVelocities() {
    return m_Velocities;
}

ecs::ComponentArray<EulerAngles>& LatLonSimulation::GetOrientations() {
    return m_Orientations;
}

ecs::ComponentArray<ShipMetadata>& LatLonSimulation::GetMetadatas() {
    return m_Metadatas;
}

ecs::ComponentArray<WaypointController>&
LatLonSimulation::GetWaypointControllers() {
    return m_WaypointControllers;
}

const ecs::ComponentArray<LatLonPosition>& LatLonSimulation::GetPositions()
    const {
    return m_Positions;
}

const ecs::ComponentArray<LatLonVelocity>& LatLonSimulation::GetVelocities()
    const {
    return m_Velocities;
}

const ecs::ComponentArray<EulerAngles>& LatLonSimulation::GetOrientations()
    const {
    return m_Orientations;
}

const ecs::ComponentArray<ShipMetadata>& LatLonSimulation::GetMetadatas()
    const {
    return m_Metadatas;
}

const ecs::ComponentArray<WaypointController>&
LatLonSimulation::GetWaypointControllers() const {
    return m_WaypointControllers;
}

#pragma once

#include <optional>
#include <vector>

#include "Components.hpp"
#include "LatLonTypes.hpp"
#include "ecs/ComponentArray.hpp"
#include "ecs/Entity.hpp"
#include "ecs/View.hpp"

// Main simulation class for lat/lon based ship simulation
// Uses Entity Component System architecture for performance
// Handles dead reckoning and waypoint-based navigation
class LatLonSimulation {
   public:
    LatLonSimulation();
    ~LatLonSimulation();

    // Main update loop - call every frame with deltaTime in seconds
    void Update(double deltaTime);

    // Entity management
    ecs::Entity AddEntity();
    void RemoveEntity(const ecs::Entity entity);
    const std::vector<ecs::Entity>& GetAllEntities() const;

    // Component setters
    void SetPosition(const ecs::Entity entity, const LatLonPosition& position);
    void SetVelocity(const ecs::Entity entity, const LatLonVelocity& velocity);
    void SetOrientation(const ecs::Entity entity, const EulerAngles& orientation);
    void SetMetadata(const ecs::Entity entity, const ShipMetadata& metadata);
    void SetWaypointController(const ecs::Entity entity,
                               const WaypointController& controller);

    // Component removers
    void RemovePosition(const ecs::Entity entity);
    void RemoveVelocity(const ecs::Entity entity);
    void RemoveOrientation(const ecs::Entity entity);
    void RemoveMetadata(const ecs::Entity entity);
    void RemoveWaypointController(const ecs::Entity entity);

    // Component getters (const)
    std::optional<const LatLonPosition*> GetPosition(
        const ecs::Entity entity) const;
    std::optional<const LatLonVelocity*> GetVelocity(
        const ecs::Entity entity) const;
    std::optional<const EulerAngles*> GetOrientation(
        const ecs::Entity entity) const;
    std::optional<const ShipMetadata*> GetMetadata(
        const ecs::Entity entity) const;
    std::optional<const WaypointController*> GetWaypointController(
        const ecs::Entity entity) const;

    // Component getters (mutable)
    std::optional<LatLonPosition*> GetPosition(const ecs::Entity entity);
    std::optional<LatLonVelocity*> GetVelocity(const ecs::Entity entity);
    std::optional<EulerAngles*> GetOrientation(const ecs::Entity entity);
    std::optional<ShipMetadata*> GetMetadata(const ecs::Entity entity);
    std::optional<WaypointController*> GetWaypointController(
        const ecs::Entity entity);

    // Component array accessors (for advanced queries)
    ecs::ComponentArray<LatLonPosition>& GetPositions();
    ecs::ComponentArray<LatLonVelocity>& GetVelocities();
    ecs::ComponentArray<EulerAngles>& GetOrientations();
    ecs::ComponentArray<ShipMetadata>& GetMetadatas();
    ecs::ComponentArray<WaypointController>& GetWaypointControllers();

    const ecs::ComponentArray<LatLonPosition>& GetPositions() const;
    const ecs::ComponentArray<LatLonVelocity>& GetVelocities() const;
    const ecs::ComponentArray<EulerAngles>& GetOrientations() const;
    const ecs::ComponentArray<ShipMetadata>& GetMetadatas() const;
    const ecs::ComponentArray<WaypointController>& GetWaypointControllers()
        const;

   private:
    // Entity management
    std::vector<ecs::Entity> m_Entities;
    std::vector<uint32_t> m_Generations;  // Generation counter per entity ID
    std::vector<uint32_t> m_DeadEntities;  // Recycled entity IDs

    // Component arrays
    ecs::ComponentArray<LatLonPosition> m_Positions;
    ecs::ComponentArray<LatLonVelocity> m_Velocities;
    ecs::ComponentArray<EulerAngles> m_Orientations;
    ecs::ComponentArray<ShipMetadata> m_Metadatas;
    ecs::ComponentArray<WaypointController> m_WaypointControllers;

#ifdef ENABLE_RF_COMPONENTS
    ecs::ComponentArray<Transmitter> m_Transmitters;
    ecs::ComponentArray<EWEmitter> m_EWEmitters;
#endif

    // Simulation systems
    void UpdateWaypoints(double deltaTime);
    void DeadReckoning(double deltaTime);
};

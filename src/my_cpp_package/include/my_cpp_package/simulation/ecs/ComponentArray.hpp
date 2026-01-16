#pragma once
#include <concepts>
#include <vector>

#include "Entity.hpp"
#include "spdlog/spdlog.h"
namespace ecs {

template <typename Component>
class ComponentArray {
   public:
    ComponentArray() : m_EntityToIndex(10000, static_cast<size_t>(-1)) {}

    template <typename C>
        requires std::constructible_from<Component, C&&>
    void Insert(const Entity entity, C&& component) {
        if (entity.ID() >= m_EntityToIndex.size()) [[unlikely]] {
            m_EntityToIndex.resize(entity.ID() + 1, static_cast<size_t>(-1));
            spdlog::warn("Resized EntityToIndex to accommodate entity ID {}", entity.ID());
        }
        m_EntityToIndex[entity.ID()] = m_Components.size();
        m_Components.push_back(std::forward<C>(component));
        m_Entities.push_back(entity);
    }

    void Remove(const Entity entity) {
        size_t index = m_EntityToIndex[entity.ID()];
        size_t lastIndex = m_Components.size() - 1;

        m_Components[index] = m_Components[lastIndex];
        m_Entities[index] = m_Entities[lastIndex];
        m_EntityToIndex[m_Entities[index].ID()] = index;

        m_Components.pop_back();
        m_Entities.pop_back();
    }

    Component& Get(const Entity entity) {
        return m_Components[m_EntityToIndex[entity.ID()]];
    }

    const Component& Get(const Entity entity) const {
        return m_Components[m_EntityToIndex[entity.ID()]];
    }

    bool Has(const Entity entity) const {
        size_t index = m_EntityToIndex[entity.ID()];
        return index < m_Components.size();
    }

    size_t Size() const { return m_Components.size(); }

   public:
    std::vector<Component> m_Components;
    std::vector<Entity> m_Entities;
    std::vector<size_t> m_EntityToIndex;
};
};  // namespace ecs

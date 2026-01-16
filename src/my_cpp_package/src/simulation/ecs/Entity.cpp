#include "my_cpp_package/simulation/ecs/Entity.hpp"

namespace ecs {
Entity::Entity() : m_ID(-1), m_Generation(-1) {};
Entity::Entity(uint64_t id) : m_ID(id), m_Generation(0) {};
Entity::Entity(uint8_t generation, uint32_t id)
    : m_ID(id), m_Generation(generation) {};

uint32_t Entity::ID() const { return m_ID; }
uint32_t Entity::Generation() const { return m_Generation; }
bool Entity::Valid() const { return m_ID != static_cast<uint32_t>(-1); }

bool Entity::operator==(const Entity& other) const {
    return m_ID == other.m_ID && m_Generation == other.m_Generation;
}

bool Entity::operator!=(const Entity& other) const { return !(*this == other); }
std::ostream& operator<<(std::ostream& os, const Entity& entity) {
    os << "Entity(ID: " << entity.m_ID
       << ", Generation: " << entity.m_Generation << ")";
    return os;
}
}  // namespace ecs

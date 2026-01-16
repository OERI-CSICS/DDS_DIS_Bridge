#pragma once

#include <fmt/format.h>

#include <cstdint>
#include <iostream>

namespace ecs {
class Entity {
   public:
    Entity();
    Entity(const Entity& other) = default;
    Entity& operator=(const Entity& other) = default;
    explicit Entity(uint64_t id);
    Entity(uint8_t generation, uint32_t id);

    uint32_t ID() const;
    uint32_t Generation() const;
    bool Valid() const;

    bool operator==(const Entity& other) const;

    bool operator!=(const Entity& other) const;

    friend std::ostream& operator<<(std::ostream& os, const Entity& entity);

   private:
    uint32_t m_ID;
    uint32_t m_Generation;
};

};  // namespace ecs

template <>
struct fmt::formatter<ecs::Entity>: fmt::formatter<std::string> {
    template <typename FormatContext>
    auto format(const ecs::Entity& entity, FormatContext& ctx) const -> decltype(ctx.out()) {
        return format_to(ctx.out(), "Entity(ID: {}, Generation: {})",
                         entity.ID(), entity.Generation());
    }
};

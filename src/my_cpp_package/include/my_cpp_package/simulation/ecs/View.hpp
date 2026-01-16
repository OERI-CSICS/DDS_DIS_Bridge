#include <algorithm>
#include <tuple>
#include <utility>
#include <vector>

#include "ComponentArray.hpp"
#include "Entity.hpp"

namespace ecs {

template <typename... Sets>
class View {
   public:
    std::tuple<Sets&...> sets;

    View(Sets&... s) : sets(s...) {}

    // ---------------- Iterator ----------------
    struct Iterator {
        View& view;
        size_t index;
        std::vector<Entity>& driving_dense;

        Iterator(View& v, size_t i, std::vector<Entity>& dense)
            : view(v), index(i), driving_dense(dense) {
            skip_non_matches();
        }

        void skip_non_matches() {
            while (index < driving_dense.size()) {
                if (matches(driving_dense[index])) return;
                ++index;
            }
        }

        bool matches(const Entity e) const {
            return std::apply([&](auto&... s) { return (s.Has(e) && ...); },
                              view.sets);
        }

        Iterator& operator++() {
            ++index;
            skip_non_matches();
            return *this;
        }

        bool operator!=(const Iterator& other) const {
            return index != other.index;
        }

        auto operator*() {
            Entity e = driving_dense.at(index);  // copy
            return std::apply(
                [&](auto&... s) {
                    return std::tuple<Entity, decltype(s.Get(e))&...>(
                        e, s.Get(e)...);
                },
                view.sets);
        }
    };

    // ---------------- Helpers ----------------
   private:
    // Build a vector of pointers to m_Entities for each set
    template <std::size_t... Is>
    std::vector<std::vector<Entity>*> entity_arrays(
        std::index_sequence<Is...>) {
        return {&std::get<Is>(sets).m_Entities...};
    }

    // Find the smallest dense array
    std::vector<Entity>& smallest_dense() {
        auto arrays = entity_arrays(std::index_sequence_for<Sets...>{});
        auto it = std::min_element(
            arrays.begin(), arrays.end(),
            [](auto a, auto b) { return a->size() < b->size(); });
        return **it;
    }

   public:
    auto begin() { return Iterator(*this, 0, smallest_dense()); }
    auto end() {
        return Iterator(*this, smallest_dense().size(), smallest_dense());
    }
};

}  // namespace ecs

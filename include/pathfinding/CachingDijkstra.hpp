#pragma once

#include <functional>
#include <optional>
#include <pathfinding/DijkstraQueue.hpp>
#include <pathfinding/Distance.hpp>
#include <pathfinding/Path.hpp>
#include <queue>
#include <string_view>
#include <vector>

namespace graph {
class Graph;
}

namespace pathfinding {

class CachingDijkstra
{
public:
    CachingDijkstra(const graph::Graph &graph) noexcept;
    CachingDijkstra() = delete;
    CachingDijkstra(CachingDijkstra &&) = default;
    CachingDijkstra(const CachingDijkstra &) = default;
    auto operator=(const CachingDijkstra &) -> CachingDijkstra & = delete;
    auto operator=(CachingDijkstra &&) -> CachingDijkstra & = delete;

    [[nodiscard]] auto findDistance(graph::Node source,
                                    graph::Node target) const noexcept
        -> graph::Distance;

    auto destroy() noexcept -> void;

private:
    [[nodiscard]] auto getDistanceTo(graph::Node n) const noexcept
        -> graph::Distance;

    auto setDistanceTo(graph::Node n, std::int64_t distance) noexcept
        -> void;

    [[nodiscard]] auto computeDistance(graph::Node source,
                                       graph::Node target) noexcept
        -> graph::Distance;

    auto unSettle(graph::Node n) noexcept -> void;

    auto settle(graph::Node n) noexcept -> void;

    [[nodiscard]] auto isSettled(graph::Node n) noexcept -> bool;

    auto reset() noexcept -> void;

    auto insertCache(graph::Node source, graph::Node target, graph::Distance dist) noexcept
        -> void;

private:
    const graph::Graph &graph_;
    std::vector<graph::Distance> distances_;
    std::vector<bool> settled_;
    std::vector<graph::Node> touched_;
    DijkstraQueue pq_;
    std::optional<graph::Node> last_source_;

    using DistanceCache = std::vector<std::vector<graph::Distance>>;
    DistanceCache distance_cache_;
};

} // namespace pathfinding

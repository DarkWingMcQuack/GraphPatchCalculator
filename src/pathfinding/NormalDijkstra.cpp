#include <fmt/core.h>
#include <functional>
#include <graph/GridGraph.hpp>
#include <numeric>
#include <optional>
#include <pathfinding/NormalDijkstra.hpp>
#include <pathfinding/Distance.hpp>
#include <queue>
#include <string_view>
#include <vector>

using graph::Node;
using graph::GridGraph;
using pathfinding::NormalDijkstra;
using pathfinding::Path;
using graph::Distance;
using graph::UNREACHABLE;

NormalDijkstra::NormalDijkstra(const graph::GridGraph& graph) noexcept
    : graph_(graph),
      distances_(graph.width * graph.height,
                 UNREACHABLE),
      settled_(graph.width * graph.height, false),
      pq_(DijkstraQueueComparer{}) {}


auto NormalDijkstra::findRoutes(const graph::Node& source, const graph::Node& target) noexcept
    -> std::vector<Path>
{
    computeDistances(source, target);
    auto path = extractShortestPaths(source, target);

    return path;
}

auto NormalDijkstra::findDistance(const graph::Node& source, const graph::Node& target) noexcept
    -> Distance
{
    computeDistances(source, target);
    return getDistanceTo(target);
}

auto NormalDijkstra::getIndex(const graph::Node& n) const noexcept
    -> std::optional<std::size_t>
{
    auto row = n.row;
    auto column = n.column;

    if(row >= graph_.get().height || row < 0) {
        return std::nullopt;
    }

    if(column >= graph_.get().width || column < 0) {
        return std::nullopt;
    }

    return n.row * graph_.get().width + n.column;
}

auto NormalDijkstra::getDistanceTo(const graph::Node& n) const noexcept
    -> Distance
{
    auto index_opt = getIndex(n);

    if(index_opt) {
        return distances_[index_opt.value()];
    }

    return UNREACHABLE;
}


auto NormalDijkstra::setDistanceTo(const graph::Node& n, Distance distance) noexcept
    -> void
{
    auto index_opt = getIndex(n);

    if(index_opt) {
        distances_[index_opt.value()] = distance;
    }
}

auto NormalDijkstra::extractShortestPaths(const graph::Node& source, const graph::Node& target) const noexcept
    -> std::vector<Path>
{
    //check if a path exists
    if(UNREACHABLE == getDistanceTo(target)) {
        return {};
    }

    Path initial{std::vector{target}};

    std::priority_queue unfinished(
        [](const auto& lhs, const auto& rhs) {
            return lhs.getLength() > rhs.getLength();
        },
        std::vector<Path>{std::move(initial)});

    std::vector<Path> complete_paths;

    while(!unfinished.empty()) {
        auto path = unfinished.top();
        unfinished.pop();

        if(!complete_paths.empty()
           && path.getLength() + 1 > complete_paths[0].getLength()) {
            return complete_paths;
        }

        const auto& last_inserted = path.getSource();
        auto neigbours = getWalkableNeigboursOf(last_inserted);
        auto smallest_distance = findSmallestDistance(neigbours);

        if(smallest_distance == UNREACHABLE) {
            continue;
        }

        for(const auto& neig : neigbours) {
            auto neig_dist = getDistanceTo(neig);

            if(neig_dist != smallest_distance) {
                continue;
            }


            auto path_copy = path;
            path_copy.pushFront(neig);

            if(neig == source) {
                complete_paths.emplace_back(std::move(path_copy));
            } else {
                unfinished.emplace(std::move(path_copy));
            }
        }
    }

    return complete_paths;
}


auto NormalDijkstra::getWalkableNeigboursOf(const graph::Node& n) const noexcept
    -> std::vector<graph::Node>
{
    return graph_.get().getWalkableNeigbours(n);
}


auto NormalDijkstra::reset() noexcept
    -> void
{
    for(auto n : touched_) {
        unSettle(n);
        setDistanceTo(n, UNREACHABLE);
    }
    touched_.clear();
    pq_ = DijkstraQueue{DijkstraQueueComparer{}};
}

auto NormalDijkstra::unSettle(const graph::Node& n)
    -> void
{
    auto index_opt = getIndex(n);

    if(index_opt) {
        settled_[index_opt.value()] = false;
    }
}

auto NormalDijkstra::settle(const graph::Node& n) noexcept
    -> void
{
    auto index_opt = getIndex(n);

    if(index_opt) {
        settled_[index_opt.value()] = true;
    }
}

auto NormalDijkstra::isSettled(const graph::Node& n)
    -> std::optional<bool>
{
    auto index_opt = getIndex(n);

    if(index_opt) {
        return settled_[index_opt.value()];
    }

    return std::nullopt;
}

auto NormalDijkstra::getNodesWithMinDistanceIn(const graph::GridCell& cell) noexcept
    -> std::vector<graph::Node>
{
    auto min_dist = std::accumulate(std::begin(cell),
                                    std::end(cell),
                                    UNREACHABLE,
                                    [&](auto acc, auto node) {
                                        auto dist = getDistanceTo(node);
                                        return std::min(dist, acc);
                                    });

    std::vector<Node> nodes;
    std::copy_if(std::begin(cell),
                 std::end(cell),
                 std::back_inserter(nodes),
                 [&](const auto& node) {
                     return getDistanceTo(node) == min_dist;
                 });

    return nodes;
}

auto NormalDijkstra::computeDistances(const graph::Node& source, const graph::Node& target) noexcept
    -> void
{
    using graph::UNREACHABLE;

    if(source == last_source_
       && isSettled(target)
       && isSettled(target).value()) {
        return;
    }

    if(source != last_source_) {
        last_source_ = source;
        reset();
        pq_.emplace(source, 0l);
        setDistanceTo(source, 0);
        touched_.emplace_back(source);
    }


    while(!pq_.empty()) {
        auto [current_node, current_dist] = pq_.top();
        pq_.pop();

        if(current_node == target) {
            return;
        }

        settle(current_node);

        auto neigbours = getWalkableNeigboursOf(current_node);

        for(auto&& neig : neigbours) {
            touched_.emplace_back(neig);

            auto neig_dist = getDistanceTo(neig);

            if(UNREACHABLE != current_dist and neig_dist > current_dist + 1) {
                setDistanceTo(neig, current_dist + 1);
                pq_.emplace(neig, current_dist + 1);
            }
        }
    }
}


auto NormalDijkstra::getGraph() const noexcept
    -> const GridGraph&
{
    return graph_.get();
}

auto NormalDijkstra::findSmallestDistance(const std::vector<graph::Node>& nodes) const noexcept
    -> Distance
{
    if(nodes.empty()) {
        return UNREACHABLE;
    }

    std::vector<Distance> distances;
    std::transform(std::cbegin(nodes),
                   std::cend(nodes),
                   std::back_inserter(distances),
                   [&](const auto& n) {
                       return getDistanceTo(n);
                   });

    return *std::min_element(std::cbegin(distances),
                             std::cend(distances));
}

#include "utils/Utils.hpp"
#include <fmt/ostream.h>
#include <functional>
#include <graph/Graph.hpp>
#include <numeric>
#include <optional>
#include <pathfinding/CachingDijkstra.hpp>
#include <pathfinding/Distance.hpp>
#include <progresscpp/ProgressBar.hpp>
#include <queue>
#include <string_view>
#include <utils/Range.hpp>
#include <vector>

using graph::Distance;
using graph::Graph;
using graph::Node;
using graph::UNREACHABLE;
using pathfinding::CachingDijkstra;

CachingDijkstra::CachingDijkstra(const graph::Graph& graph) noexcept
    : graph_(graph),
      distances_(graph.size(), UNREACHABLE),
      settled_(graph.size(), false),
      pq_(DijkstraQueueComparer{}),
      distance_cache_(graph.size(),
                      std::vector(graph.size(), UNREACHABLE))
{
    for(auto from : utils::range(graph_.size())) {
        for(auto to : utils::range(graph.size())) {
            distance_cache_[from][to] = computeDistance(from, to);
        }
    }

    //cleanup everything to save memory
    distances_.clear();
    settled_.clear();
    pq_ = DijkstraQueue{DijkstraQueueComparer{}};
    touched_.clear();
    last_source_ = std::nullopt;
}

auto CachingDijkstra::findDistance(graph::Node source,
                                   graph::Node target) const noexcept
    -> Distance
{
    return distance_cache_[source][target];
}

auto CachingDijkstra::insertCache(graph::Node source,
                                  graph::Node target,
                                  graph::Distance dist) noexcept
    -> void
{
    distance_cache_[target][source] = dist;
}

auto CachingDijkstra::destroy() noexcept
    -> void
{
    utils::cleanAndFree(distances_);
    utils::cleanAndFree(settled_);
    utils::cleanAndFree(touched_);
    utils::cleanAndFree(distance_cache_);
}


auto CachingDijkstra::getDistanceTo(graph::Node n) const noexcept
    -> Distance
{
    return distances_[n];
}

auto CachingDijkstra::setDistanceTo(graph::Node n,
                                    Distance distance) noexcept
    -> void
{
    distances_[n] = distance;
}

auto CachingDijkstra::reset() noexcept
    -> void
{
    for(auto n : touched_) {
        unSettle(n);
        setDistanceTo(n, UNREACHABLE);
    }
    touched_.clear();
    pq_ = DijkstraQueue{DijkstraQueueComparer{}};
}

auto CachingDijkstra::unSettle(graph::Node n) noexcept
    -> void
{
    settled_[n] = false;
}

auto CachingDijkstra::settle(graph::Node n) noexcept
    -> void
{

    settled_[n] = true;
}

auto CachingDijkstra::isSettled(graph::Node n) noexcept
    -> bool
{
    return settled_[n];
}

auto CachingDijkstra::computeDistance(graph::Node source,
                                      graph::Node target) noexcept
    -> Distance
{
    using graph::UNREACHABLE;

    if(source == last_source_ && isSettled(target)) {
        return getDistanceTo(target);
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

        settle(current_node);

        if(current_node == target) {
            return current_dist;
        }

        // pop after the return, otherwise we loose a value
        // when reusing the pq
        pq_.pop();

        auto neigbours = graph_.getForwardNeigboursOf(current_node);

        for(auto [neig, dist] : neigbours) {
            auto neig_dist = getDistanceTo(neig);
            auto new_dist = current_dist + dist;

            if(UNREACHABLE != current_dist and neig_dist > new_dist) {
                touched_.emplace_back(neig);
                setDistanceTo(neig, new_dist);
                pq_.emplace(neig, new_dist);
            }
        }
    }

    return getDistanceTo(target);
}

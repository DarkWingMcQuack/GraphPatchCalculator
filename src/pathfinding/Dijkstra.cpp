#include <functional>
#include <graph/Graph.hpp>
#include <numeric>
#include <optional>
#include <pathfinding/Dijkstra.hpp>
#include <pathfinding/Distance.hpp>
#include <queue>
#include <string_view>
#include <vector>

using graph::Node;
using graph::Graph;
using pathfinding::Dijkstra;
using pathfinding::Path;
using graph::Distance;
using graph::UNREACHABLE;

Dijkstra::Dijkstra(const graph::Graph& graph) noexcept
    : graph_(graph),
      distances_(graph.size(), UNREACHABLE),
      settled_(graph.size(), false),
      pq_(DijkstraQueueComparer{}),
      before_(graph.size(), graph::NOT_REACHABLE),
      rank_(graph.size(), UNREACHABLE) {}

auto Dijkstra::findRoute(graph::Node source, graph::Node target) noexcept
    -> std::optional<Path>
{
    [[maybe_unused]] auto _ = computeDistance(source, target);
    return extractShortestPath(source, target);
}

auto Dijkstra::findDistance(graph::Node source, graph::Node target) noexcept
    -> Distance
{
    return computeDistance(source, target);
}

auto Dijkstra::getDistanceTo(graph::Node n) const noexcept
    -> Distance
{
    return distances_[n];
}


auto Dijkstra::setDistanceTo(graph::Node n, Distance distance) noexcept
    -> void
{
    distances_[n] = distance;
}

auto Dijkstra::extractShortestPath(graph::Node source, graph::Node target) const noexcept
    -> std::optional<Path>
{
    //check if a path exists
    if(UNREACHABLE == getDistanceTo(target)) {
        return std::nullopt;
    }

    Path path{std::vector{target}};

    while(path.getSource() != source) {
        const auto& last_inserted = path.getSource();
        path.pushFront(before_[last_inserted]);
    }

    return path;
}


auto Dijkstra::reset() noexcept
    -> void
{
    for(auto n : touched_) {
        unSettle(n);
        setDistanceTo(n, UNREACHABLE);
        setBefore(n, graph::NOT_REACHABLE);
        rank_[n] = UNREACHABLE;
    }

    touched_.clear();
    pq_ = DijkstraQueue{DijkstraQueueComparer{}};
    current_rank_ = 0;
}

auto Dijkstra::unSettle(graph::Node n)
    -> void
{
    settled_[n] = false;
}

auto Dijkstra::settle(graph::Node n) noexcept
    -> void
{
    settled_[n] = true;
}

auto Dijkstra::isSettled(graph::Node n)
    -> bool
{
    return settled_[n];
}

auto Dijkstra::computeDistance(graph::Node source, graph::Node target) noexcept
    -> Distance
{
    using graph::UNREACHABLE;

    if(source == last_source_
       && isSettled(target)) {
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

        //pop after the return, otherwise we loose a value
        //when reusing the pq
        pq_.pop();

        auto neigbours = graph_.getForwardNeigboursOf(current_node);

        for(auto [neig, distance] : neigbours) {

            auto neig_dist = getDistanceTo(neig);
            auto new_dist = current_dist + distance;

            if(UNREACHABLE != current_dist and neig_dist > new_dist) {
                touched_.emplace_back(neig);
                setDistanceTo(neig, new_dist);
                pq_.emplace(neig, new_dist);
                setBefore(neig, current_node);
            }
        }
    }

    return getDistanceTo(target);
}

auto Dijkstra::calculateDijkstraRank(graph::Node source, graph::Node target) noexcept
    -> std::size_t
{
    using graph::UNREACHABLE;

    if(source == last_source_ and rank_[target] != UNREACHABLE) {
        return rank_[target];
    }

    if(source != last_source_) {
        reset();
        last_source_ = source;
        pq_.emplace(source, 0l);
        setDistanceTo(source, 0);
        touched_.emplace_back(source);
    }

    while(!pq_.empty()) {
        auto [current_node, current_dist] = pq_.top();

        if(!isSettled(current_node)) {
            rank_[current_node] = current_rank_++;
        }


        if(current_node == target) {
            return rank_[current_node];
        }

        //pop after the return, otherwise we loose a value
        //when reusing the pq
        pq_.pop();

        if(isSettled(current_node)) {
            continue;
        }

        settle(current_node);

        auto neigbours = graph_.getForwardNeigboursOf(current_node);

        for(auto [neig, distance] : neigbours) {

            auto neig_dist = getDistanceTo(neig);
            auto new_dist = current_dist + distance;

            if(UNREACHABLE != current_dist and neig_dist > new_dist) {
                touched_.emplace_back(neig);
                setDistanceTo(neig, new_dist);
                pq_.emplace(neig, new_dist);
                setBefore(neig, current_node);
            }
        }
    }

    return UNREACHABLE;
}

auto Dijkstra::setBefore(graph::Node n, graph::Node before) noexcept
    -> void
{
    before_[n] = before;
}

#pragma once

#include <graph/Graph.hpp>
#include <pathfinding/Distance.hpp>
#include <queue>
#include <tuple>
#include <vector>

namespace pathfinding {

struct DijkstraQueueComparer
{
    auto operator()(const std::pair<graph::Node, graph::Distance>& lhs,
                    const std::pair<graph::Node, graph::Distance>& rhs) const noexcept
        -> bool
    {
        return lhs.second > rhs.second;
    }
};

using DijkstraQueue = std::priority_queue<std::pair<graph::Node, graph::Distance>,
                                          std::vector<std::pair<graph::Node, graph::Distance>>,
                                          DijkstraQueueComparer>;


struct AStarQueueComparer
{
    auto operator()(const std::tuple<graph::Node, graph::Distance, graph::Distance>& lhs,
                    const std::tuple<graph::Node, graph::Distance, graph::Distance>& rhs) const noexcept
        -> bool
    {
        // return lhs.second > rhs.second;
        return std::get<1>(lhs) + std::get<2>(lhs)
            > std::get<1>(rhs) + std::get<2>(rhs);
    }
};

using AStarQueue = std::priority_queue<std::tuple<graph::Node, graph::Distance, graph::Distance>,
                                       std::vector<std::tuple<graph::Node, graph::Distance, graph::Distance>>,
                                       AStarQueueComparer>;

} // namespace pathfinding

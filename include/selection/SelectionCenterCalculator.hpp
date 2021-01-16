
#pragma once

#include "pathfinding/Path.hpp"
#include <cmath>
#include <fmt/core.h>
#include <graph/Graph.hpp>
#include <pathfinding/DijkstraQueue.hpp>
#include <pathfinding/Distance.hpp>
#include <queue>
#include <selection/NodeSelection.hpp>
#include <vector>

namespace selection {

template<class PathFinder>
class SelectionCenterCalculator
{
public:
    SelectionCenterCalculator(const graph::Graph& graph)
        : graph_(graph),
          path_finder_(graph_),
          settled_(graph_.size(), false) {}

    auto calculateCenter(graph::Node from, graph::Node to) noexcept
        -> std::optional<graph::Node>
    {
        auto path_opt = getPath(from, to);
        if(!path_opt) {
            return std::nullopt;
        }

        auto path = std::move(path_opt.value());
        auto initial_center_opt = findCenter(path);
        if(!initial_center_opt) {
            return std::nullopt;
        }

        auto initial_center = std::move(initial_center_opt.value());

        auto optimized = optimzeCenter(from,
                                       to,
                                       initial_center);

        reset();

        return optimized;
    }

private:
    auto getPath(graph::Node from, graph::Node to) noexcept
        -> std::optional<pathfinding::Path>
    {
        return path_finder_.findRoute(from, to);
    }

    auto findCenter(const pathfinding::Path& path) const noexcept
        -> std::optional<graph::Node>
    {
        return path.getMiddleNode();
    }

    auto optimzeCenter(graph::Node /*source*/,
                       graph::Node /*target*/,
                       graph::Node initial_center) noexcept
        -> graph::Node
    {
        return initial_center;
    }

    auto reset() noexcept
        -> void
    {
        for(auto i : touched_) {
            settled_[i] = false;
        }

        touched_.clear();
        pq_ = pathfinding::DijkstraQueue{pathfinding::DijkstraQueueComparer{}};
    }

private:
    const graph::Graph& graph_;
    PathFinder path_finder_;
    std::vector<bool> settled_;
    std::vector<std::size_t> touched_;
    pathfinding::DijkstraQueue pq_;
};

} // namespace selection

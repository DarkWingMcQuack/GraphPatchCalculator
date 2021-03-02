#pragma once

#include <pathfinding/Path.hpp>
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
class MiddleChoosingCenterCalculator
{
public:
    MiddleChoosingCenterCalculator(const graph::Graph& graph)
        : graph_(graph),
          path_finder_(graph_){}

    auto calculateCenter(graph::Node from, graph::Node to) noexcept
        -> std::optional<graph::Node>
    {
        auto path_opt = getPath(from, to);
        if(!path_opt) {
            return std::nullopt;
        }

        auto path = std::move(path_opt.value());
        return path.getMiddleNode();
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

private:
    const graph::Graph& graph_;
    PathFinder path_finder_;
};

} // namespace selection

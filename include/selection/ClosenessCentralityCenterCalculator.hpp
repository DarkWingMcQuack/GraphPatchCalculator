#pragma once

#include <cmath>
#include <fmt/core.h>
#include <graph/Graph.hpp>
#include <pathfinding/DijkstraQueue.hpp>
#include <pathfinding/Distance.hpp>
#include <pathfinding/Path.hpp>
#include <queue>
#include <selection/NodeSelection.hpp>
#include <utils/Range.hpp>
#include <vector>

namespace selection {

template<class PathFinder,
         class DistanceOracle>

class ClosenessCentralityCenterCalculator
{
public:
    ClosenessCentralityCenterCalculator(const graph::Graph& graph,
                                        const DistanceOracle& distance_oracle)
        : graph_(graph),
          path_finder_(graph_),
          closeness_centrality_(graph_.size(), 0.0)
    {
        auto graph_size = graph_.size();
        auto node_range = utils::range(graph_size);
        for(auto n : node_range) {
            auto farness = std::transform_reduce(
                std::begin(node_range),
                std::end(node_range),
                0.0,
                [](auto current, auto next) {
                    if(next == graph::UNREACHABLE) {
                        return current;
                    }
                    return current + next;
                },
                [&](auto from) {
                    return distance_oracle.findDistance(from, n);
                });

            auto closeness = static_cast<double>(graph_size)
                / static_cast<double>(farness);

            closeness_centrality_[n] = closeness;
        }
    }

    auto calculateCenter(graph::Node from, graph::Node to) noexcept
        -> std::optional<graph::Node>
    {
        auto path_opt = getPath(from, to);
        if(!path_opt) {
            return std::nullopt;
        }

        auto path = std::move(path_opt.value());
        return findCenter(path);
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

        if(path.empty()) {
            return std::nullopt;
        }

        const auto& nodes = path.getNodes();

        return std::max_element(
            std::begin(nodes),
            std::end(nodes),
            [&](auto lhs, auto rhs) {
                return closeness_centrality_[lhs]
                    < closeness_centrality_[rhs];
            });
    }

private:
    const graph::Graph& graph_;
    PathFinder path_finder_;
    std::vector<double> closeness_centrality_;
};

} // namespace selection

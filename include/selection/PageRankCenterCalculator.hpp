#pragma once

#include <cmath>
#include <execution>
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

template<class PathFinder>
class PageRankCenterCalculator
{
public:
    PageRankCenterCalculator(const graph::Graph& graph, std::size_t iterations)
        : graph_(graph),
          path_finder_(graph_),
          pr_(graph_.size(), 0.0)
    {
        for(auto i = 0ul; i < iterations; i++) {
            pr_ = pageRankIteration();
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
    auto pageRankIteration() noexcept
        -> std::vector<double>
    {
        auto dumping_factor = 0.85;

        std::vector page_rank(graph_.size(), 0.0);

        auto nodes = utils::range(graph_.size());

        std::transform(std::execution::par_unseq,
                       std::begin(nodes),
                       std::end(nodes),
                       std::begin(page_rank),
                       [&](auto current_node) {
                           return pageRankUpdate(current_node, dumping_factor);
                       });

        return page_rank;
    }

    auto pageRankUpdate(graph::Node n, double dumping_factor) noexcept
        -> double
    {
        auto out_neigs = graph_.getForwardNeigboursOf(n);
        auto sum = std::transform_reduce(
            std::execution::par_unseq,
            std::begin(out_neigs),
            std::end(out_neigs),
            0.0,
            std::plus<>(),
            [&](auto pair) {
                auto [neig, _] = pair;
                return pr_[neig] / graph_.getBackwardNeigboursOf(neig).size();
            });

        return (1 - dumping_factor) + dumping_factor * sum;
    }

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

        return *std::max_element(
            std::begin(nodes),
            std::end(nodes),
            [&](auto lhs, auto rhs) {
                return pr_[lhs] < pr_[rhs];
            });
    }

private:
    const graph::Graph& graph_;
    PathFinder path_finder_;
    std::vector<double> pr_;
};

} // namespace selection

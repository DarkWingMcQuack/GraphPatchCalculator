#pragma once

#include <fmt/core.h>
#include <pathfinding/Distance.hpp>
#include <queue>
#include <random>
#include <selection/NodeSelection.hpp>
#include <selection/SelectionLookup.hpp>
#include <unordered_set>
#include <vector>

namespace selection {

class SelectionOptimizer
{
public:
    SelectionOptimizer(std::size_t number_of_nodes,
                       std::vector<NodeSelection> selections,
                       std::size_t max_number_of_selections = std::numeric_limits<std::size_t>::max());

    auto optimize() noexcept
        -> void;

    auto getLookup() && noexcept
        -> SelectionLookup;

private:
    auto optimize(std::size_t idx) noexcept
        -> void;

    auto optimizeLeft(std::size_t idx) noexcept
        -> void;

    auto optimizeRight(std::size_t idx) noexcept
        -> void;

    auto getLeftOptimalGreedySelection(std::size_t node_idx,
                                       const std::unordered_set<graph::Node>& nodes) const noexcept
        -> std::size_t;

    auto getRightOptimalGreedySelection(std::size_t node_idx,
                                        const std::unordered_set<graph::Node>& nodes) const noexcept
        -> std::size_t;

private:
    std::size_t number_of_nodes_;

    std::vector<NodeSelection> selections_;

    std::vector<CenterSet> source_selections_;
    std::vector<CenterSet> target_selections_;

    std::unordered_set<std::size_t> keep_list_left_;
    std::unordered_set<std::size_t> keep_list_right_;

    std::size_t max_number_of_selections_;
};

} // namespace selection

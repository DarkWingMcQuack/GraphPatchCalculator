#pragma once

#include <fmt/core.h>
#include <fmt/format.h>
#include <graph/Graph.hpp>
#include <map>
#include <pathfinding/Distance.hpp>
#include <queue>
#include <random>
#include <selection/NodeSelection.hpp>
#include <unordered_set>
#include <utils/Utils.hpp>
#include <vector>

namespace selection {

using CenterSet = std::vector<std::pair<std::size_t, graph::Distance>>;

class SelectionLookup
{
public:
    SelectionLookup(std::size_t number_of_nodes,
                    std::vector<NodeSelection> selections);

    SelectionLookup(
        std::size_t number_of_nodes,
        std::vector<graph::Node> centers,
        std::vector<CenterSet> source_selections,
        std::vector<CenterSet> target_selections);


    [[nodiscard]] auto getSelectionAnswering(const graph::Node& source,
                                             const graph::Node& target) const noexcept
        -> graph::Distance;

    [[nodiscard]] auto getSizeDistributionSource() const noexcept
        -> std::map<std::size_t, std::size_t>;

    [[nodiscard]] auto getSizeDistributionTarget() const noexcept
        -> std::map<std::size_t, std::size_t>;

    [[nodiscard]] auto getSizeDistributionTotal() const noexcept
        -> std::map<std::size_t, std::size_t>;

    [[nodiscard]] auto averageSelectionsPerNode() const noexcept
        -> double;

    auto toFile(std::string_view path) const noexcept
        -> void;

private:
private:
    std::size_t number_of_nodes_;
    std::vector<graph::Node> centers_;

    std::vector<CenterSet> source_selections_;
    std::vector<CenterSet> target_selections_;

    friend class SelectionBucketCreator;
    friend class SelectionLookupOptimizer;
};

} // namespace selection

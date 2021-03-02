#pragma once

#include <execution>
#include <graph/Graph.hpp>
#include <nlohmann/json.hpp>
#include <string_view>
#include <vector>

namespace selection {

using Patch = std::vector<std::pair<graph::Node, graph::Distance>>;

class NodeSelection
{
public:
    NodeSelection(Patch source_patch,
                  Patch target_patch,
                  graph::Node center,
                  bool is_inverse_valid);

    [[nodiscard]] auto getSourcePatch() const noexcept
        -> const Patch&;

    [[nodiscard]] auto getTargetPatch() const noexcept
        -> const Patch&;

    [[nodiscard]] auto getSourcePatch() noexcept
        -> Patch&;

    [[nodiscard]] auto getTargetPatch() noexcept
        -> Patch&;

    [[nodiscard]] auto canAnswer(graph::Node from, graph::Node to) const noexcept
        -> bool;

    [[nodiscard]] auto getCenter() const noexcept
        -> graph::Node;

    [[nodiscard]] auto isSubSetOf(const NodeSelection& other) const noexcept
        -> bool;

    auto clear() noexcept
        -> void;

    [[nodiscard]] auto isEmpty() const noexcept
        -> bool;

    [[nodiscard]] auto weight() const noexcept
        -> std::size_t;

    [[nodiscard]] auto averageDistance() const noexcept
        -> graph::Distance;


    auto deleteFromSource(const std::vector<graph::Node>& nodes) noexcept
        -> void;
    auto deleteFromTarget(const std::vector<graph::Node>& nodes) noexcept
        -> void;

    auto toFile(std::string_view path) const noexcept
        -> void;

    auto toLatLngFiles(std::string_view path, const graph::Graph& graph) const noexcept
        -> void;

    auto toJson(const graph::Graph& graph) const noexcept
        -> nlohmann::json;

    auto toFileAsJson(std::string_view path, const graph::Graph& graph) const noexcept
        -> void;

    auto isInverseValid() const noexcept
        -> bool;

private:
    Patch source_patch_;
    Patch target_patch_;
    graph::Node center_;
    bool is_inverse_valid_;
};



template<class DistanceOracle>
[[nodiscard]] auto couldMerge(const NodeSelection& first,
                              const NodeSelection& second,
                              const DistanceOracle& oracle) noexcept
    -> bool
{
    auto center = first.getCenter();
    const auto& first_sources = first.getSourcePatch();
    const auto& first_targets = first.getTargetPatch();

    const auto& second_sources = second.getSourcePatch();
    const auto& second_targets = second.getTargetPatch();

    auto first_condition = std::all_of(
        std::begin(first_sources),
        std::end(first_sources),
        [&](auto pair) {
            auto source = pair.first;
            auto source_center = pair.second;
            return std::all_of(
                std::begin(second_targets),
                std::end(second_targets),
                [&](auto inner_pair) {
                    auto [target, _] = inner_pair;

                    auto true_dist = oracle.findDistance(source, target);
                    auto center_target = oracle.findDistance(center, target);

                    return source_center + center_target == true_dist;
                });
        });

    if(!first_condition) {
        return false;
    }

    auto second_condition = std::all_of(
        std::begin(first_targets),
        std::end(first_targets),
        [&](auto pair) {
            auto target = pair.first;
            auto center_target = pair.second;
            return std::all_of(
                std::begin(second_sources),
                std::end(second_sources),
                [&](auto inner_pair) {
                    auto [source, _] = inner_pair;

                    auto true_dist = oracle.findDistance(source, target);
                    auto source_center = oracle.findDistance(source, center);

                    return source_center + center_target == true_dist;
                });
        });

    return second_condition;
}

template<class DistanceOracle>
[[nodiscard]] auto merge(NodeSelection first,
                         NodeSelection second,
                         const DistanceOracle& oracle) noexcept
    -> NodeSelection
{
    auto center = first.getCenter();
    auto source_patch = std::move(first.getSourcePatch());
    auto target_patch = std::move(first.getTargetPatch());

    std::transform(std::begin(second.getSourcePatch()),
                   std::end(second.getSourcePatch()),
                   std::back_inserter(source_patch),
                   [&](auto pair) {
                       auto [source, _] = pair;
                       auto dist = oracle.findDistance(source, center);
                       return std::pair{source, dist};
                   });

    std::transform(std::begin(second.getTargetPatch()),
                   std::end(second.getTargetPatch()),
                   std::back_inserter(target_patch),
                   [&](auto pair) {
                       auto [target, _] = pair;
                       auto dist = oracle.findDistance(center, target);
                       return std::pair{target, dist};
                   });

    return NodeSelection(std::move(source_patch),
                         std::move(target_patch),
                         center,
                         first.isInverseValid());
}

} // namespace selection

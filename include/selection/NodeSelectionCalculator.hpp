#pragma once

#include <graph/Graph.hpp>
#include <pathfinding/Distance.hpp>
#include <queue>
#include <selection/NodeSelection.hpp>
#include <selection/SelectionCenterCalculator.hpp>
#include <vector>

namespace selection {

template<class PathFinder,
         class CachedPathFinder>
class NodeSelectionCalculator
{
public:
    NodeSelectionCalculator(const graph::Graph& graph,
                            const std::vector<std::vector<bool>>& coverage)
        : cached_path_finder_(graph),
          center_calculator_(graph),
          graph_(graph),
          source_settled_(graph_.size(), false),
          target_settled_(graph_.size(), false),
          coverage_(coverage) {}

    [[nodiscard]] auto distanceOf(graph::Node source,
                                  graph::Node target) const noexcept
        -> graph::Distance
    {
        return cached_path_finder_.findDistance(source, target);
    }


    [[nodiscard]] auto calculateFullSelection(graph::Node source_start,
                                              graph::Node target_start) noexcept
        -> std::optional<NodeSelection>
    {
        auto center_opt = calculateCenter(source_start, target_start);
        if(!center_opt) {
            return std::nullopt;
        }
        auto center = center_opt.value();

        auto source_to_center = cached_path_finder_.findDistance(source_start, center);
        auto target_to_center = cached_path_finder_.findDistance(center, target_start);

        source_patch_.emplace_back(source_start, source_to_center);
        target_patch_.emplace_back(target_start, target_to_center);

        auto last_node = graph_.size();

        graph::Node current_src_candidate = 0;
        graph::Node current_trg_candidate = 0;

        while(current_src_candidate < last_node or current_trg_candidate < last_node) {

            if(current_src_candidate < last_node) {
                auto source_dist_opt = checkSourceAffiliation(current_src_candidate,
                                                              center,
                                                              target_patch_);
                if(source_dist_opt) {
                    auto source_dist = source_dist_opt.value();


                    if(!areAllTargetSettledFor(current_src_candidate)) {
                        source_patch_.emplace_back(current_src_candidate, source_dist);
                    }
                }
                current_src_candidate++;
            }

            if(current_trg_candidate < last_node) {
                auto target_dist_opt = checkTargetAffiliation(current_trg_candidate,
                                                              center,
                                                              source_patch_);
                if(target_dist_opt) {
                    auto target_dist = target_dist_opt.value();

                    if(!areAllTargetSettledFor(current_trg_candidate)) {
                        target_patch_.emplace_back(current_trg_candidate, target_dist);
                    }
                }
                current_trg_candidate++;
            }
        }

        //create the selection which was found
        NodeSelection selection{std::move(source_patch_),
                                std::move(target_patch_),
                                center};

        //cleanup and reset the state of the calculator
        cleanup();

        return selection;
    }

    [[nodiscard]] auto checkSourceAffiliation(graph::Node source,
                                              graph::Node center,
                                              const Patch& targets) noexcept
        -> std::optional<graph::Distance>
    {
        auto center_dist = cached_path_finder_.findDistance(source, center);

        if(center_dist == graph::UNREACHABLE) {
            return std::nullopt;
        }

        auto valid = std::all_of(
            std::begin(targets),
            std::end(targets),
            [&](auto pair) {
                auto [target, center_target_dist] = pair;
                auto dist = cached_path_finder_.findDistance(source, target);

                return center_dist + center_target_dist == dist;
            });

        if(!valid) {
            return std::nullopt;
        }

        return center_dist;
    }

    [[nodiscard]] auto checkTargetAffiliation(graph::Node target,
                                              graph::Node center,
                                              const Patch& sources) noexcept
        -> std::optional<graph::Distance>
    {
        auto center_dist = cached_path_finder_.findDistance(center, target);

        if(center_dist == graph::UNREACHABLE) {
            return std::nullopt;
        }

        auto valid = std::all_of(
            std::begin(sources),
            std::end(sources),
            [&](auto pair) {
                auto [source, center_target_dist] = pair;

                auto dist = cached_path_finder_.findDistance(source, target);
                return center_dist + center_target_dist == dist;
            });

        if(!valid) {
            return std::nullopt;
        }

        return center_dist;
    }

private:
    auto cleanup() noexcept
        -> void
    {
        for(auto n : touched_) {
            unsettle(n);
        }
        touched_.clear();
        source_patch_.clear();
        target_patch_.clear();
    }

    auto unsettle(graph::Node node) noexcept
        -> void
    {
        source_settled_[node] = false;
        target_settled_[node] = false;
    }

    auto settleLeft(graph::Node node) noexcept
        -> void
    {
        source_settled_[node] = true;
    }

    auto settleRight(graph::Node node) noexcept
        -> void
    {
        target_settled_[node] = true;
    }

    [[nodiscard]] auto isLeftSettled(graph::Node node) const noexcept
        -> bool
    {
        return source_settled_[node];
    }

    [[nodiscard]] auto isRightSettled(graph::Node node) const noexcept
        -> bool
    {
        return target_settled_[node];
    }

    [[nodiscard]] auto calculateCenter(graph::Node source,
                                       graph::Node target) noexcept
        -> std::optional<graph::Node>
    {
        return center_calculator_.calculateCenter(source, target);
    }

    [[nodiscard]] auto areAllSourceSettledFor(graph::Node target) const noexcept
        -> bool
    {
        return std::all_of(
            std::begin(source_patch_),
            std::end(source_patch_),
            [&](auto pair) {
                auto [source, _] = pair;
                return coverage_[source].empty()
                    or coverage_[source][target];
            });
    }

    [[nodiscard]] auto areAllTargetSettledFor(graph::Node source) const noexcept
        -> bool
    {
        const auto& source_vec = coverage_[source];
        if(source_vec.empty()) {
            return true;
        }

        return std::all_of(
            std::begin(target_patch_),
            std::end(target_patch_),
            [&](auto pair) {
                auto [target, _] = pair;
                return source_vec[target];
            });
    }

private:
    CachedPathFinder cached_path_finder_;
    SelectionCenterCalculator<PathFinder> center_calculator_;
    const graph::Graph& graph_;

    std::vector<graph::Node> touched_;

    std::vector<bool> source_settled_;
    std::vector<bool> target_settled_;

    std::vector<std::pair<graph::Node, graph::Distance>> source_patch_;
    std::vector<std::pair<graph::Node, graph::Distance>> target_patch_;
    const std::vector<std::vector<bool>>& coverage_;
};

} // namespace selection

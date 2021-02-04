#pragma once

#include <graph/Graph.hpp>
#include <pathfinding/Distance.hpp>
#include <queue>
#include <selection/NodeSelection.hpp>
#include <vector>

namespace selection {

template<class CenterCalculator,
         class CachedPathFinder>
class NodeSelectionCalculator
{
public:
    NodeSelectionCalculator(const CachedPathFinder& cached_path_finder,
                            CenterCalculator center_calculator,
                            const graph::Graph& graph,
                            const std::vector<std::vector<bool>>& coverage)
        : cached_path_finder_(cached_path_finder),
          center_calculator_(center_calculator),
          graph_(graph),
          coverage_(coverage) {}

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
        auto center_to_target = cached_path_finder_.findDistance(center, target_start);

        source_patch_.emplace_back(source_start, source_to_center);
        target_patch_.emplace_back(target_start, center_to_target);

        auto last_node = graph_.size();

        graph::Node current_src_candidate = 0;
        graph::Node current_trg_candidate = 0;

        while(current_src_candidate < last_node or current_trg_candidate < last_node) {

            while(current_src_candidate < last_node
                  and !processSourceCandidate(current_src_candidate,
                                              center,
                                              source_start)) {
                current_src_candidate++;
            }

            while(current_trg_candidate < last_node
                  and !processTargetCandidate(current_trg_candidate,
                                              center,
                                              target_start)) {
                current_trg_candidate++;
            }

            current_trg_candidate++;
            current_src_candidate++;
        }

        //create the selection which was found
        NodeSelection selection{std::move(source_patch_),
                                std::move(target_patch_),
                                center};

        //cleanup and reset the state of the calculator
        cleanup();

        return selection;
    }

private:
    [[nodiscard]] auto processSourceCandidate(graph::Node node, graph::Node center, graph::Node start) noexcept
        -> bool
    {
        if(node == center or node == start) {
            return false;
        }

        if(countNewPathsForSource(node) == 0) {
            return false;
        }

        auto source_dist_opt = checkSourceAffiliation(node,
                                                      center,
                                                      target_patch_);
        if(source_dist_opt) {
            auto source_dist = source_dist_opt.value();
            source_patch_.emplace_back(node, source_dist);

            return true;
        }

        return false;
    }

    [[nodiscard]] auto processTargetCandidate(graph::Node node, graph::Node center, graph::Node start) noexcept
        -> bool
    {
        if(node == center or node == start) {
            return false;
        }

        if(countNewPathsForTarget(node) == 0) {
            return false;
        }


        auto target_dist_opt = checkTargetAffiliation(node,
                                                      center,
                                                      source_patch_);
        if(target_dist_opt) {
            auto target_dist = target_dist_opt.value();
            target_patch_.emplace_back(node, target_dist);

            return true;
        }

        return false;
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
        source_patch_.clear();
        target_patch_.clear();
    }

    [[nodiscard]] auto calculateCenter(graph::Node source,
                                       graph::Node target) noexcept
        -> std::optional<graph::Node>
    {
        return center_calculator_.calculateCenter(source, target);
    }

    [[nodiscard]] auto countNewPathsForTarget(graph::Node target) const noexcept
        -> std::size_t
    {
        return std::count_if(
            std::begin(source_patch_),
            std::end(source_patch_),
            [&](auto pair) {
                auto [source, _] = pair;
                return !coverage_[source].empty()
                    and !coverage_[source][target];
            });
    }

    [[nodiscard]] auto countNewPathsForSource(graph::Node source) const noexcept
        -> std::size_t
    {
        const auto& source_vec = coverage_[source];
        if(source_vec.empty()) {
            return 0;
        }

        return std::count_if(
            std::begin(target_patch_),
            std::end(target_patch_),
            [&](auto pair) {
                auto [target, _] = pair;
                return !source_vec[target];
            });
    }

private:
    const CachedPathFinder& cached_path_finder_;
    CenterCalculator center_calculator_;

    const graph::Graph& graph_;

    std::vector<std::pair<graph::Node, graph::Distance>> source_patch_;
    std::vector<std::pair<graph::Node, graph::Distance>> target_patch_;
    const std::vector<std::vector<bool>>& coverage_;
};

} // namespace selection

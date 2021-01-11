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
    NodeSelectionCalculator(const graph::Graph& graph)
        : center_calculator_(graph),
          cached_path_finder_(graph),
          graph_(graph),
          source_settled_(graph_.size(), false),
          target_settled_(graph_.size(), false) {}


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

        std::deque<graph::Node> source_candidates;
        source_candidates.emplace_back(source_start);
        source_patch_.emplace_back(source_start, source_to_center);

        auto target_to_center = cached_path_finder_.findDistance(target_start, center);

        std::deque<graph::Node> target_candidates;
        target_candidates.push_back(target_start);
        target_patch_.emplace_back(target_start, target_to_center);


        while(!source_candidates.empty() or !target_candidates.empty()) {

            if(!source_candidates.empty()) {
                auto current = source_candidates.front();
                source_candidates.pop_front();

                if(auto source_dist_opt = checkSourceAffiliation(current, center, target_patch_)) {
                    auto source_dist = source_dist_opt.value();
                    source_patch_.emplace_back(current, source_dist);

                    auto neigbours = graph_.getNeigboursOf(current);
                    for(auto [neig, dist] : neigbours) {
                        if(!isLeftSettled(neig)) {
                            settleLeft(neig);
                            touched_.push_back(neig);
                            source_candidates.push_back(neig);
                        }
                    }
                }
            }

            if(!target_candidates.empty()) {
                auto current = target_candidates.front();
                target_candidates.pop_front();

                if(auto target_dist_opt = checkTargetAffiliation(current, center, source_patch_)) {
                    auto target_dist = target_dist_opt.value();

                    target_patch_.emplace_back(current, target_dist);

                    auto neigbours = graph_.getNeigboursOf(current);
                    for(auto [neig, dist] : neigbours) {
                        if(!isRightSettled(neig)) {
                            settleRight(neig);
                            touched_.push_back(neig);
                            target_candidates.push_back(neig);
                        }
                    }
                }
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

        auto valid = std::all_of(
            std::begin(targets),
            std::end(targets),
            [&](auto pair) {
                auto [target, center_target_dist] = pair;

                auto dist = cached_path_finder_.findDistance(source, target);

                if(center_target_dist == graph::UNREACHABLE
                   or center_dist == graph::UNREACHABLE) {
                    if(dist != graph::UNREACHABLE) {
                        return false;
                    }
                }

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

        auto valid = std::all_of(
            std::begin(sources),
            std::end(sources),
            [&](auto pair) {
                auto [source, center_target_dist] = pair;

                auto dist = cached_path_finder_.findDistance(source, target);

                if(center_target_dist == graph::UNREACHABLE
                   or center_dist == graph::UNREACHABLE) {
                    if(dist != graph::UNREACHABLE) {
                        return false;
                    }
                }

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

private:
    SelectionCenterCalculator<PathFinder> center_calculator_;
    CachedPathFinder cached_path_finder_;
    const graph::Graph& graph_;

    std::vector<graph::Node> touched_;

    std::vector<bool> source_settled_;
    std::vector<bool> target_settled_;

    std::vector<std::pair<graph::Node, graph::Distance>> source_patch_;
    std::vector<std::pair<graph::Node, graph::Distance>> target_patch_;
};

} // namespace selection

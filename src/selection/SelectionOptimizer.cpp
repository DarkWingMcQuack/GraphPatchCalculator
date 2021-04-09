#include <execution>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <graph/Graph.hpp>
#include <pathfinding/Distance.hpp>
#include <progresscpp/ProgressBar.hpp>
#include <selection/NodeSelection.hpp>
#include <selection/SelectionOptimizer.hpp>
#include <utils/Range.hpp>
#include <vector>

using selection::SelectionLookup;
using selection::SelectionOptimizer;


SelectionOptimizer::SelectionOptimizer(std::size_t number_of_nodes,
                                       std::vector<NodeSelection> selections,
                                       const pathfinding::CachingDijkstra& oracle,
                                       graph::Distance min_dist,
                                       std::size_t max_number_of_selections)
    : number_of_nodes_(number_of_nodes),
      selections_(std::move(selections)),
      source_selections_(number_of_nodes_),
      target_selections_(number_of_nodes_),
      oracle_(oracle),
      min_dist_(min_dist),
      max_number_of_selections_(max_number_of_selections)
{
    for(const auto& [i, selection] : utils::enumerate(selections_)) {
        for(auto [node, dist] : selection.getSourcePatch()) {
            source_selections_[node].emplace_back(i, dist);
        }

        for(auto [node, dist] : selection.getTargetPatch()) {
            target_selections_[node].emplace_back(i, dist);
        }
    }
}

auto SelectionOptimizer::optimize() noexcept
    -> void
{
    fmt::print("optimizing patch lookup...\n");
    progresscpp::ProgressBar bar{number_of_nodes_, 80ul};

    for(auto n : utils::range(number_of_nodes_)) {
        optimize(n);

        bar++;
        bar.displayIfChangedAtLeast(0.001);
    }

    bar.done();
}

auto SelectionOptimizer::getLookup() && noexcept
    -> SelectionLookup
{
    std::vector<graph::Node> centers;
    std::transform(std::begin(selections_),
                   std::end(selections_),
                   std::back_inserter(centers),
                   [](const auto& selection) {
                       return selection.getCenter();
                   });

    return SelectionLookup{number_of_nodes_,
                           std::move(centers),
                           std::move(source_selections_),
                           std::move(target_selections_)};
}


auto SelectionOptimizer::optimize(std::size_t idx) noexcept
    -> void
{
    optimizeLeft(idx);
    optimizeRight(idx);
}

auto SelectionOptimizer::getLeftOptimalGreedySelection(graph::Node node,
                                                       const std::unordered_set<graph::Node>& nodes) const noexcept
    -> std::size_t
{
    const auto& node_selects = source_selections_[node];

    return std::transform_reduce(
               std::execution::unseq,
               std::begin(node_selects),
               std::end(node_selects),
               std::pair{node_selects[0].first, 0l},
               [](auto current, auto next) {
                   auto [best_index, best_score] = current;
                   auto [new_index, new_score] = next;

                   if(new_score > best_score) {
                       return next;
                   }

                   return current;
               },
               [&](auto pair) {
                   auto [idx, _] = pair;
                   const auto& right_nodes = selections_[idx].getTargetPatch();

                   auto score = std::count_if(std::begin(right_nodes),
                                              std::end(right_nodes),
                                              [&](auto n) {
                                                  return nodes.count(n.first) == 0
                                                      and node != n.first
                                                      and oracle_.findDistance(node, n.first) > min_dist_;
                                              });

                   return std::pair{idx, score};
               })
        .first;
}

auto SelectionOptimizer::getRightOptimalGreedySelection(graph::Node node,
                                                        const std::unordered_set<graph::Node>& nodes) const noexcept
    -> std::size_t
{
    const auto& node_selects = target_selections_[node];

    return std::transform_reduce(
               std::execution::unseq,
               std::begin(node_selects),
               std::end(node_selects),
               std::pair{node_selects[0].first, 0l},
               [](auto current, auto next) {
                   auto [best_index, best_score] = current;
                   auto [new_index, new_score] = next;

                   if(new_score > best_score) {
                       return next;
                   }

                   return current;
               },
               [&](auto pair) {
                   auto [idx, _] = pair;
                   const auto& left_nodes = selections_[idx].getSourcePatch();

                   auto score = std::count_if(std::begin(left_nodes),
                                              std::end(left_nodes),
                                              [&](auto n) {
                                                  return nodes.count(n.first) == 0
                                                      and node != n.first
                                                      and oracle_.findDistance(n.first, node) > min_dist_;
                                              });

                   return std::pair{idx, score};
               })
        .first;
}

namespace {

auto isContainedIn(std::unordered_set<graph::Node>& first,
                   std::unordered_set<graph::Node>& second) noexcept
    -> bool
{
    return std::all_of(std::begin(first),
                       std::end(first),
                       [&](auto node) {
                           return second.count(node) > 0;
                       });
}

} // namespace

auto SelectionOptimizer::optimizeLeft(graph::Node node) noexcept
    -> void
{
    const auto& left_secs = source_selections_[node];
    std::unordered_set<graph::Node> all_nodes;

    for(auto [idx, _] : left_secs) {
        const auto& target_nodes = selections_[idx].getTargetPatch();

        if(selections_[idx].getCenter() == node) {
            continue;
        }

        for(auto [target, _] : target_nodes) {
            if(oracle_.findDistance(node, target) > min_dist_) {
                all_nodes.insert(target);
            }
        }
    }

    std::unordered_set<std::size_t> new_selection_set;
    std::unordered_set<graph::Node> covered_nodes;

    auto counter = 0ul;
    for(auto [idx, _] : left_secs) {
        if(keep_list_left_.count(idx) == 0) {
            continue;
        }

		if(counter++ >= max_number_of_selections_){
		  break;
		}

        const auto& target_nodes = selections_[idx].getTargetPatch();

        for(auto [target, _] : target_nodes) {
            covered_nodes.insert(target);
        }

        if(selections_[idx].getCenter() != node) {
            new_selection_set.emplace(idx);
        }
    }

    while(counter < max_number_of_selections_
          and !isContainedIn(all_nodes, covered_nodes)) {
        auto next_selection_idx = getLeftOptimalGreedySelection(node, covered_nodes);
        const auto& target_nodes = selections_[next_selection_idx].getTargetPatch();

        for(auto [target, _] : target_nodes) {
            covered_nodes.insert(target);
        }

        if(selections_[next_selection_idx].getCenter() != node) {
            keep_list_left_.emplace(next_selection_idx);
            counter++;
        }
        new_selection_set.emplace(next_selection_idx);
    }

    source_selections_[node].erase(
        std::remove_if(std::begin(source_selections_[node]),
                       std::end(source_selections_[node]),
                       [&](auto pair) {
                           return new_selection_set.count(pair.first) == 0;
                       }),
        std::end(source_selections_[node]));
}

auto SelectionOptimizer::optimizeRight(graph::Node node) noexcept
    -> void
{
    const auto& right_secs = target_selections_[node];
    std::unordered_set<graph::Node> all_nodes;

    for(auto [idx, _] : right_secs) {
        const auto& source_nodes = selections_[idx].getSourcePatch();
        for(auto [source, _] : source_nodes) {
            if(oracle_.findDistance(source, node) > min_dist_) {
                all_nodes.insert(source);
            }
        }
    }

    all_nodes.erase(node);

    std::unordered_set<graph::Node> covered_nodes;
    std::unordered_set<std::size_t> new_selection_set;
    auto counter = 0ul;
    for(auto [idx, _] : right_secs) {
        if(keep_list_right_.count(idx) == 0) {
            continue;
        }

		if(counter++ >= max_number_of_selections_){
		  break;
		}

        const auto& source_nodes = selections_[idx].getSourcePatch();

        for(auto [source, _] : source_nodes) {
            covered_nodes.insert(source);
        }

        if(selections_[idx].getCenter() != node) {
            new_selection_set.emplace(idx);
        }
    }

    while(counter < max_number_of_selections_
          and !isContainedIn(all_nodes, covered_nodes)) {
        auto next_selection_idx = getRightOptimalGreedySelection(node, covered_nodes);
        const auto& source_nodes = selections_[next_selection_idx].getSourcePatch();

        for(auto [source, _] : source_nodes) {
            covered_nodes.insert(source);
        }

        if(selections_[next_selection_idx].getCenter() != node) {
            new_selection_set.emplace(next_selection_idx);
            counter++;
        }
        keep_list_right_.emplace(next_selection_idx);
    }


    target_selections_[node].erase(
        std::remove_if(std::begin(target_selections_[node]),
                       std::end(target_selections_[node]),
                       [&](auto pair) {
                           return new_selection_set.count(pair.first) == 0;
                       }),
        std::end(target_selections_[node]));
}

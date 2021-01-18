#pragma once

#include <execution>
#include <graph/Graph.hpp>
#include <pathfinding/Distance.hpp>
#include <progresscpp/ProgressBar.hpp>
#include <queue>
#include <random>
#include <selection/NodeSelection.hpp>
#include <selection/NodeSelectionCalculator.hpp>
#include <utils/Range.hpp>
#include <vector>

namespace selection {

template<class PathFinder,
         class CachedPathFinder>
class FullNodeSelectionCalculator
{
public:
    FullNodeSelectionCalculator(const graph::Graph& graph,
                                std::size_t prune_distance)
        : graph_(graph),
          all_to_all_(graph.size()),
          node_selector_(graph)
    {
        for(auto first : utils::range(graph.size())) {
            all_to_all_[first] = std::vector(graph.size(), true);

            for(auto second : utils::range(graph.size())) {
                if(node_selector_.distanceOf(first, second) > prune_distance) {
                    all_to_all_[first][second] = false;
                }
            }
        }
    }

    [[nodiscard]] auto calculateFullNodeSelection() noexcept
        -> std::vector<NodeSelection>
    {
        std::vector<NodeSelection> calculated_selections;

        progresscpp::ProgressBar bar{graph_.size(), 80ul};
        auto done_counter = countDoneNodes();

        while(!done()) {

            auto [first, second] = getRandomRemainingPair();

            auto selection_opt = node_selector_.calculateFullSelection(first, second);
            if(!selection_opt) {

                all_to_all_[first][second] = true;
                ifDoneClear(first);

                continue;
            }

            auto selection = std::move(selection_opt.value());

            fmt::print("before size: {}\n", selection.weight());
            optimizeSelection(selection);
            fmt::print("size: {}\n", selection.weight());
            fmt::print("----------------------------------------\n");
            eraseNodeSelection(selection);

            calculated_selections.emplace_back(std::move(selection));


            //update progress bar
            auto new_done = countDoneNodes();
            auto diff = new_done - done_counter;
            bar += diff;
            done_counter = new_done;

            bar.displayIfChangedAtLeast(0.001);
        }

        bar.done();

        return calculated_selections;
    }


    [[nodiscard]] auto getRandomRemainingPair() const noexcept
        -> std::pair<graph::Node, graph::Node>
    {
        auto undone = std::count_if(std::begin(all_to_all_),
                                    std::end(all_to_all_),
                                    [](const auto& b) {
                                        return !b.empty();
                                    });

        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<std::size_t> outer_dis(0, undone - 1);

        auto outer_random = outer_dis(gen);

        graph::Node source = 0;
        std::size_t counter = 0;
        for(std::size_t i = 0; i < all_to_all_.size(); i++) {
            if(!all_to_all_[i].empty()) {
                if(counter++ == outer_random) {
                    source = i;
                    break;
                }
            }
        }

        auto inner_undone = std::count_if(std::begin(all_to_all_[source]),
                                          std::end(all_to_all_[source]),
                                          [](auto b) {
                                              return !b;
                                          });

        std::uniform_int_distribution<std::size_t> inner_dis(0, inner_undone - 1);

        auto inner_random = inner_dis(gen);

        std::size_t inner_counter = 0;
        graph::Node target = 0;
        for(std::size_t i = all_to_all_[source].size() - 1; i >= 0; i--) {
            if(!all_to_all_[source][i]) {
                if(inner_counter++ == inner_random) {
                    target = i;
                    break;
                }
            }
        }

        return std::pair{source, target};
    }

    auto eraseNodeSelection(const NodeSelection& selection) noexcept
        -> void
    {
        const auto& sources = selection.getSourcePatch();
        const auto& targets = selection.getTargetPatch();

        for(auto [first, _] : sources) {
            for(auto [second, _] : targets) {
                if(!all_to_all_[first].empty()) {
                    all_to_all_[first][second] = true;
                }
            }
        }

        for(auto [n, _] : sources) {
            ifDoneClear(n);
        }
    }

    auto optimizeSelection(NodeSelection& selection) noexcept
        -> void
    {
        auto changed = true;
        auto& sources = selection.getSourcePatch();
        auto& targets = selection.getTargetPatch();

        while(changed) {
            auto weight_before = selection.weight();

            sources.erase(
                std::remove_if(std::begin(sources),
                               std::end(sources),
                               [&](auto pair) {
                                   auto [source, _] = pair;
                                   return areAllTargetSettledFor(source, targets);
                               }),
                std::end(sources));

            targets.erase(
                std::remove_if(std::begin(targets),
                               std::end(targets),
                               [&](auto pair) {
                                   auto [target, _] = pair;
                                   return areAllSourceSettledFor(sources, target);
                               }),
                std::end(targets));

            auto weight_after = selection.weight();

            changed = weight_before != weight_after;
        }
    }

    [[nodiscard]] auto areAllSourceSettledFor(const Patch& sources, graph::Node target) const noexcept
        -> bool
    {
        return std::all_of(
            std::begin(sources),
            std::end(sources),
            [&](auto pair) {
                auto [source, _] = pair;
                return all_to_all_[source].empty()
                    or all_to_all_[source][target];
            });
    }

    [[nodiscard]] auto areAllTargetSettledFor(graph::Node source, const Patch& targets) const noexcept
        -> bool
    {
        const auto& source_vec = all_to_all_[source];
        if(source_vec.empty()) {
            return true;
        }

        return std::all_of(
            std::begin(targets),
            std::end(targets),
            [&](auto pair) {
                auto [target, _] = pair;
                return source_vec[target];
            });
    }

    [[nodiscard]] auto countDoneNodes() const noexcept
        -> std::size_t
    {
        return std::count_if(std::begin(all_to_all_),
                             std::end(all_to_all_),
                             [](const auto& b) {
                                 return b.empty();
                             });
    }

    auto ifDoneClear(graph::Node n) noexcept
        -> void
    {
        auto done = std::all_of(std::begin(all_to_all_[n]),
                                std::end(all_to_all_[n]),
                                [](auto x) { return x; });
        if(done) {
            all_to_all_[n].clear();
        }
    }

    [[nodiscard]] auto done() noexcept
        -> bool
    {
        return std::all_of(
            std::begin(all_to_all_),
            std::end(all_to_all_),
            [](const auto& x) { return x.empty(); });
    }

private:
    const graph::Graph& graph_;
    std::vector<std::vector<bool>> all_to_all_;
    NodeSelectionCalculator<PathFinder, CachedPathFinder> node_selector_;
};

} // namespace selection

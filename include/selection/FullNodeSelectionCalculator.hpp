#pragma once

#include <execution>
#include <fmt/core.h>
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

template<class CenterCalculator,
         class DistanceOracle>
class FullNodeSelectionCalculator
{
public:
    FullNodeSelectionCalculator(const graph::Graph& graph,
                                const DistanceOracle& distance_oracle,
                                CenterCalculator center_calculator,
                                graph::Distance prune_distance)
        : graph_(graph),
          distance_oracle_(distance_oracle),
          all_to_all_(graph.size()),
          node_selector_(distance_oracle,
                         std::move(center_calculator),
                         graph,
                         all_to_all_)
    {
        for(auto first : utils::range(graph.size())) {
            all_to_all_[first] = std::vector(graph.size(), true);

            bool empty = true;
            for(auto second : utils::range(graph.size())) {
                auto distance = distance_oracle_.findDistance(first, second);
                if(distance > prune_distance and distance != graph::UNREACHABLE) {
                    all_to_all_[first][second] = false;
                    empty = false;
                }
            }

            if(empty) {
                all_to_all_[first].clear();
            }
        }
    }

    [[nodiscard]] auto calculateFullNodeSelection() noexcept
        -> std::vector<NodeSelection>
    {
        std::vector<NodeSelection> calculated_selections;

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

            if(selection.weight() == 0) {
                continue;
            }

            eraseNodeSelection(selection);
            calculated_selections.emplace_back(std::move(selection));

            //update progress bar
            auto new_done = countDoneNodes();
            auto diff = new_done - done_counter;
            done_counter = new_done;
        }

        return calculated_selections;
    }

private:
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

    [[nodiscard]] auto getMaxDistanceRemainingPair() const noexcept
        -> std::pair<graph::Node, graph::Node>
    {
        auto nodes = utils::range(graph_.size());
        graph::Node source = 0;
        graph::Node target = 0;
        auto max_dist = 0;

        for(auto from : nodes) {
            if(all_to_all_[from].empty()) {
                continue;
            }

            for(auto to : nodes) {
                if(all_to_all_[from][to]) {
                    continue;
                }

                auto dist = distance_oracle_.findDistance(from, to);
                if(dist != graph::UNREACHABLE and dist > max_dist) {
                    max_dist = dist;
                    source = from;
                    target = to;
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
    const DistanceOracle& distance_oracle_;
    std::vector<std::vector<bool>> all_to_all_;
    NodeSelectionCalculator<CenterCalculator, DistanceOracle> node_selector_;
};

} // namespace selection

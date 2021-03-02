#include <filesystem>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <graph/Graph.hpp>
#include <pathfinding/CachingDijkstra.hpp>
#include <pathfinding/Dijkstra.hpp>
#include <pathfinding/HubLabels.hpp>
#include <selection/ClosenessCentralityCenterCalculator.hpp>
#include <selection/FullNodeSelectionCalculator.hpp>
#include <selection/MiddleChoosingCenterCalculator.hpp>
#include <selection/PageRankCenterCalculator.hpp>
#include <selection/SelectionLookup.hpp>
#include <selection/SelectionOptimizer.hpp>
#include <utils/ProgramOptions.hpp>
#include <utils/Timer.hpp>
#include <utils/Utils.hpp>

using pathfinding::CachingDijkstra;
using pathfinding::Dijkstra;
using selection::NodeSelection;
using selection::FullNodeSelectionCalculator;
namespace fs = std::filesystem;


template<class DistanceOracle>
auto queryAll(const graph::Graph &graph,
              const DistanceOracle &oracle,
              const selection::SelectionLookup &lookup)
{
    auto number_of_nodes = graph.size();
    auto found = 0;
    auto not_fount = 0;
    for(auto from : utils::range(number_of_nodes)) {
        for(auto to : utils::range(number_of_nodes)) {
            if(from == to) {
                continue;
            }
            auto selection_result = lookup.getSelectionAnswering(from, to);
            auto oracle_result = oracle.findDistance(from, to);

            if(selection_result and selection_result.value().second == oracle_result) {
                found++;
            } else if(selection_result and selection_result.value().second != oracle_result) {
                fmt::print("{} - {} oracle: {}, us: {}\n",
                           from,
                           to,
                           oracle_result,
                           selection_result.value().second);
            } else if(!selection_result and oracle_result == graph::UNREACHABLE) {
                found++;
            } else {
                not_fount++;
            }
        }
    }

    fmt::print("found: {}\n", found);
    fmt::print("not found: {}\n", not_fount);
}

template<class DistanceOracle>
auto mergeSelections(std::vector<NodeSelection> &&selections,
                     const DistanceOracle &oracle) noexcept
    -> std::vector<NodeSelection>
{
    progresscpp::ProgressBar bar{selections.size(), 80ul};

    for(auto i = selections.size() - 1; i >= 0 and i < selections.size(); i--) {
        for(auto j = 0; j < selections.size(); j++) {

            if(j == i or selections[i].isEmpty() or selections[j].isEmpty()) {
                continue;
            }
            const auto &big = selections[j];
            const auto &small = selections[i];

            if(selection::couldMerge(big, small, oracle)) {
                selections[i] = selection::merge(std::move(selections[j]),
                                                 std::move(selections[i]),
                                                 oracle);

                selections[j].clear();
                // break;
            }
        }

        bar++;
        bar.displayIfChangedAtLeast(0.01);
    }

    bar.done();

    selections.erase(
        std::remove_if(std::begin(selections),
                       std::end(selections),
                       [&](const auto &s) {
                           return s.weight() == 0;
                       }),
        std::end(selections));

    return std::move(selections);
}

auto writeToFiles(const graph::Graph &graph,
                  std::string_view result_folder,
                  const std::vector<NodeSelection> &selections) noexcept
    -> void
{
    auto selection_folder = fmt::format("{}/selections", result_folder);
    fs::create_directories(selection_folder);

    for(std::size_t i{0}; i < selections.size(); i++) {
        const auto path = fmt::format("{}/selection-{}.json", selection_folder, i);
        selections[i].toFileAsJson(path, graph);
    }
}


template<class DistanceOracle>
auto runSelection(const graph::Graph &graph,
                  const DistanceOracle &distance_oracle,
                  std::string_view result_folder,
                  std::size_t prune_distance,
                  std::size_t max_selections)
{
    using CenterCalculator = selection::MiddleChoosingCenterCalculator<Dijkstra>;
    using SelectionCalculator = FullNodeSelectionCalculator<CenterCalculator, DistanceOracle>;

    CenterCalculator center_calculator{graph};

    SelectionCalculator selection_calculator{graph,
                                             distance_oracle,
                                             std::move(center_calculator),
                                             prune_distance};

    auto selections = selection_calculator.calculateFullNodeSelection();

    std::sort(std::rbegin(selections),
              std::rend(selections),
              [](const auto &lhs, const auto &rhs) {
                  return lhs.weight() < rhs.weight();
              });
    fmt::print("selections: {}\n", selections.size());

    // selections = mergeSelections(std::move(selections),
    //                              distance_oracle);

    fmt::print("selections: {}\n", selections.size());

    writeToFiles(graph, result_folder, selections);

    selection::SelectionOptimizer optimizer{graph.size(),
                                            std::move(selections),
                                            max_selections};
    optimizer.optimize();

    auto lookup = std::move(optimizer).getLookup();

    for(auto [size, amount] : lookup.getSizeDistributionTotal()) {
        fmt::print("{} : {},\n", size, amount);
    }

    queryAll(graph, distance_oracle, lookup);
}

auto main(int argc, char *argv[]) -> int
{
    const auto options = utils::parseArguments(argc, argv);
    const auto graph_file = options.getGraphFile();
    const auto graph = graph::parseFMIFile(graph_file).value();
    const auto prune_distance = options.getPruneDistance();
    const auto max_selections = options.getMaxNumberOfSelectionsPerNode();
    const auto graph_filename = utils::unquote(fs::path(graph_file).filename());
    const auto result_folder = fmt::format("./results/{}/", graph_filename);

    fs::create_directories(result_folder);

    if(options.hasFmiGraphFile()) {
        auto hl = generateHubLabelsFromFmiFile(options.getFmiGraphFile().data());
        runSelection(graph,
                     hl,
                     result_folder,
                     prune_distance,
                     max_selections);
    } else {
        CachingDijkstra distance_oracle{graph};
        runSelection(graph,
                     distance_oracle,
                     result_folder,
                     prune_distance,
                     max_selections);
    }
}

#include <filesystem>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <graph/Graph.hpp>
#include <pathfinding/CachingDijkstra.hpp>
#include <pathfinding/Dijkstra.hpp>
#include <selection/FullNodeSelectionCalculator.hpp>
#include <selection/SelectionLookup.hpp>
#include <selection/SelectionOptimizer.hpp>
#include <utils/ProgramOptions.hpp>
#include <utils/Timer.hpp>
#include <utils/Utils.hpp>

using pathfinding::CachingDijkstra;
using pathfinding::Dijkstra;
using selection::FullNodeSelectionCalculator;
namespace fs = std::filesystem;

using SelectionCalculator = FullNodeSelectionCalculator<Dijkstra, CachingDijkstra>;

auto runSelection(const graph::Graph &graph,
                  std::string_view result_folder,
                  std::size_t prune_distance)
{
    utils::Timer t;
    SelectionCalculator selection_calculator{graph, prune_distance};
    auto selections = selection_calculator.calculateFullNodeSelection();

    fmt::print("runtime: {}\n", t.elapsed());
    fmt::print("selections calculated: {}\n", selections.size());

    std::sort(std::rbegin(selections), std::rend(selections),
              [](const auto &lhs, const auto &rhs) {
                  return lhs.weight() < rhs.weight();
              });

    auto selection_folder = fmt::format("{}/selections", result_folder);
    fs::create_directories(selection_folder);

    for(std::size_t i{0}; i < selections.size(); i++) {
        const auto path = fmt::format("{}/selection-{}", selection_folder, i);
        selections[i].toFile(path);
        selections[i].toLatLngFiles(path, graph);
    }

    selection::SelectionOptimizer optimizer{graph.size(),
                                            std::move(selections)};
    optimizer.optimize();

    auto lookup = std::move(optimizer).getLookup();

    std::size_t optimized_total = 0;
    for(auto [size, amount] : lookup.getSizeDistributionTotal()) {
        fmt::print("{} : {},\n", size, amount);
        optimized_total += size * amount;
    }

    fmt::print("optimized: {}\n", optimized_total);
}

auto main(int argc, char *argv[]) -> int
{
    const auto options = utils::parseArguments(argc, argv);
    const auto graph_file = options.getGraphFile();
    const auto graph = graph::parseFMIFile(graph_file).value();
    const auto prune_distance = options.getPruneDistance();
    const auto graph_filename = utils::unquote(fs::path(graph_file).filename());
    const auto result_folder = fmt::format("./results/{}/", graph_filename);

    fs::create_directories(result_folder);

    runSelection(graph, result_folder, prune_distance);
}

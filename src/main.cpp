#include <filesystem>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <fstream>
#include <graph/Graph.hpp>
#include <pathfinding/CachingDijkstra.hpp>
#include <pathfinding/Dijkstra.hpp>
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


auto createRankQueries(const graph::Graph &graph) noexcept
    -> std::vector<std::vector<std::pair<graph::Node, graph::Node>>>
{
    Dijkstra dijkstra{graph};
    auto number_of_nodes = graph.size();

    std::vector<std::vector<std::pair<graph::Node, graph::Node>>> queries;
    queries.resize(number_of_nodes);

    for(graph::Node from = 0; from < number_of_nodes; from++) {
        for(graph::Node to = 0; to < number_of_nodes; to++) {
            if(from == to) {
                continue;
            }

            auto rank = dijkstra.calculateDijkstraRank(from, to);
            if(rank < number_of_nodes) {
                queries[rank].emplace_back(from, to);
            }
        }
    }

    std::random_device rd;
    std::mt19937 g(rd());
    for(auto &v : queries) {
        std::shuffle(std::begin(v),
                     std::end(v),
                     g);
    }

    return queries;
}

template<class DistanceOracle>
auto queryAll(const graph::Graph &graph,
              DistanceOracle &oracle,
              const selection::SelectionLookup &lookup) noexcept
    -> std::tuple<
        std::map<std::size_t, std::pair<double, std::size_t>>,
        std::map<std::size_t, std::pair<double, std::size_t>>,
        std::map<std::size_t, std::pair<std::size_t, std::size_t>>>
{
    utils::Timer timer;
    auto number_of_nodes = graph.size();
    auto all_found = 0ul;
    auto all_not_found = 0ul;

    timer.reset();
    for(graph::Node from = 0; from < number_of_nodes; from++) {
        for(graph::Node to = 0; to < number_of_nodes; to++) {
            auto oracle_result = oracle.findDistance(from, to);

            if(from == to) {
                continue;
            }

            if(oracle_result != graph::UNREACHABLE) {
                all_found++;
            } else {
                all_not_found++;
            }
        }
    }
    auto elapsed = timer.elapsed();

    oracle.destroy();
	oracle.~DistanceOracle();

    auto all_queries = createRankQueries(graph);

    std::vector<std::vector<std::pair<graph::Node, graph::Node>>> found_queries(graph.size());
    std::vector<std::vector<std::pair<graph::Node, graph::Node>>> not_found_queries(graph.size());
    std::map<std::size_t, std::pair<std::size_t, std::size_t>> per_dijkstra_rank_found;
    for(int rank = 0; rank < number_of_nodes; rank++) {
        for(auto [from, to] : all_queries[rank]) {
            if(lookup.getSelectionAnswering(from, to) != graph::UNREACHABLE) {
                found_queries[rank].emplace_back(from, to);
                per_dijkstra_rank_found[rank].second++;
            } else {
                not_found_queries[rank].emplace_back(from, to);
            }
        }

		per_dijkstra_rank_found[rank].first = all_queries[rank].size();

        utils::cleanAndFree(all_queries[rank]);
    }
    utils::cleanAndFree(all_queries);


    std::map<std::size_t, std::pair<double, std::size_t>>
        per_dijkstra_rank_found_runtime;
    std::map<std::size_t, std::pair<double, std::size_t>> per_dijkstra_rank_not_found_runtime;

    auto counter = 0;
    auto found = 0ul;
    auto not_found = 0ul;
    auto found_query_time = 0.0;
    auto not_found_query_time = 0.0;
    for(int rank = 0; rank < number_of_nodes; rank++) {

        timer.reset();
        for(auto [from, to] : found_queries[rank]) {
            counter += lookup.getSelectionAnswering(from, to);
        }
        auto time = timer.elapsed();

        per_dijkstra_rank_found_runtime[rank].first = time;
        per_dijkstra_rank_found_runtime[rank].second = found_queries[rank].size();
        found += found_queries[rank].size();
        found_query_time += time;

        timer.reset();
        for(auto [from, to] : not_found_queries[rank]) {
            counter += lookup.getSelectionAnswering(from, to);
        }
        time = timer.elapsed();

        per_dijkstra_rank_not_found_runtime[rank].first = time;
        per_dijkstra_rank_not_found_runtime[rank].second = not_found_queries[rank].size();
        not_found_query_time += time;
        not_found += not_found_queries[rank].size();


        utils::cleanAndFree(found_queries[rank]);
        utils::cleanAndFree(not_found_queries[rank]);
    }




    fmt::print("{} \t {} \t {} \t {} \t {}\n",
               found_query_time / found,
               not_found_query_time / not_found,
               static_cast<double>(found) / static_cast<double>(not_found + found),
               static_cast<double>(all_found) / static_cast<double>(all_not_found + all_found),
               elapsed / (all_found + all_not_found));

    return std::tuple{per_dijkstra_rank_found_runtime,
                      per_dijkstra_rank_not_found_runtime,
                      per_dijkstra_rank_found};
}


auto writeDijkstraRankToFile(
    const std::map<std::size_t, std::pair<double, std::size_t>> &per_dijkstra_rank_found_runtime,
    const std::map<std::size_t, std::pair<double, std::size_t>> &per_dijkstra_rank_not_found_runtime,
    const std::map<std::size_t, std::pair<std::size_t, std::size_t>> &per_dijkstra_rank_found,
    const std::string &filename) noexcept
    -> void
{
    std::ofstream found{filename + "_found"};
    for(auto [rank, pair] : per_dijkstra_rank_found_runtime) {
        if(pair.second > 0) {
            found << rank << "\t:\t" << (pair.first / pair.second) << "\n";
        }
    }

    std::ofstream not_found{filename + "_not_found"};
    for(auto [rank, pair] : per_dijkstra_rank_not_found_runtime) {
        if(pair.second > 0) {
            not_found << rank << "\t:\t" << (pair.first / pair.second) << "\n";
        }
    }

    std::ofstream found_existing{filename + "_found_vs_existing"};
    for(auto [rank, pair] : per_dijkstra_rank_found) {
        if(pair.first > 0) {
            found_existing << rank
                           << "\t:\t"
                           << (static_cast<double>(pair.second) / static_cast<double>(pair.first))
                           << "\n";
        }
    }
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
                  DistanceOracle &distance_oracle,
                  const std::string &result_folder,
                  graph::Distance prune_distance,
                  std::size_t max_selections)
{
    using CenterCalculator = selection::MiddleChoosingCenterCalculator<Dijkstra>;
    using SelectionCalculator = FullNodeSelectionCalculator<CenterCalculator, DistanceOracle>;

    CenterCalculator center_calculator{graph};

    SelectionCalculator selection_calculator{graph,
                                             distance_oracle,
                                             std::move(center_calculator),
                                             prune_distance};

    utils::Timer t;

    t.reset();
    auto selections = selection_calculator.calculateFullNodeSelection();
    auto time = t.elapsed();
    fmt::print("{} \t ", time);

    std::sort(std::rbegin(selections),
              std::rend(selections),
              [](const auto &lhs, const auto &rhs) {
                  return lhs.weight() < rhs.weight();
              });

    // writeToFiles(graph, result_folder, selections);

    t.reset();
    selection::SelectionOptimizer optimizer{graph.size(),
                                            std::move(selections),
                                            distance_oracle,
                                            prune_distance,
                                            max_selections};
    optimizer.optimize();

    auto lookup = std::move(optimizer).getLookup();

    time = t.elapsed();
    fmt::print("{} \t {} \t ", time, lookup.averageSelectionsPerNode());

    auto [found, not_found, found_existing] = queryAll(graph, distance_oracle, lookup);

    writeDijkstraRankToFile(found,
                            not_found,
                            found_existing,
                            result_folder
                                + "dijkstra_rank_"
                                + std::to_string(max_selections));
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

    CachingDijkstra distance_oracle{graph};
    runSelection(graph,
                 distance_oracle,
                 result_folder,
                 prune_distance,
                 max_selections);
}

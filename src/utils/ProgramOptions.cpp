#include <CLI/CLI.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <utils/ProgramOptions.hpp>
#include <pathfinding/Distance.hpp>

using utils::ProgramOptions;


ProgramOptions::ProgramOptions(graph::Distance prune_distance,
                               std::string graph_file,
                               std::size_t maximum_number_of_selections_per_node,
                               std::optional<std::string> result_folder)
    : prune_distance_(prune_distance),
      graph_file_(std::move(graph_file)),
      maximum_number_of_selections_per_node_(maximum_number_of_selections_per_node),
      separation_folder_(std::move(result_folder)) {}

auto ProgramOptions::getGraphFile() const noexcept
    -> std::string_view
{
    return graph_file_;
}

auto ProgramOptions::hasResultFolder() const noexcept
    -> bool
{
    return !!separation_folder_;
}


auto ProgramOptions::getResultFolder() const noexcept
    -> std::string_view
{
    return separation_folder_.value();
}

auto ProgramOptions::getPruneDistance() const noexcept
    -> graph::Distance
{
    return prune_distance_;
}

auto ProgramOptions::getMaxNumberOfSelectionsPerNode() const noexcept
    -> std::size_t
{
    return maximum_number_of_selections_per_node_;
}


auto utils::parseArguments(int argc, char* argv[])
    -> ProgramOptions
{
    CLI::App app{"Grid-Graph Path Finder"};

    std::string graph_file;
    std::string result_folder;
    graph::Distance prune_distance = 0;
    std::size_t maximum_selections = std::numeric_limits<std::size_t>::max();

    app.add_option("-g,--graph",
                   graph_file,
                   "file containing the graph structure in the fmi format")
        ->check(CLI::ExistingFile)
        ->required();

    app.add_option("-o,--output",
                   result_folder,
                   "output folder")
        ->check(CLI::ExistingDirectory);

    app.add_option("-p,--prune",
                   prune_distance,
                   "minimum distance between two nodes to be not pruned")
        ->check(CLI::PositiveNumber);

    app.add_option("-m,--max-selections",
                   maximum_selections,
                   "maximum number of selections per node")
        ->check(CLI::PositiveNumber);

    try {
        app.parse(argc, argv);
    } catch(const CLI::ParseError& e) {
        std::exit(app.exit(e));
    }

    return ProgramOptions{prune_distance,
                          std::move(graph_file),
                          maximum_selections,
                          result_folder.empty()
                              ? std::optional<std::string>()
                              : std::optional{result_folder}};
}

#include <CLI/CLI.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <utils/ProgramOptions.hpp>

using utils::ProgramOptions;


ProgramOptions::ProgramOptions(std::size_t prune_distance,
                               std::string graph_file,
                               std::optional<std::string> result_folder)
    : prune_distance_(prune_distance),
      graph_file_(std::move(graph_file)),
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
    -> std::size_t
{
    return prune_distance_;
}


auto utils::parseArguments(int argc, char* argv[])
    -> ProgramOptions
{
    CLI::App app{"Grid-Graph Path Finder"};

    std::string graph_file;
    std::string result_folder;
    std::size_t prune_distance = 1;

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

    try {
        app.parse(argc, argv);
    } catch(const CLI::ParseError& e) {
        std::exit(app.exit(e));
    }

    return ProgramOptions{prune_distance,
                          std::move(graph_file),
                          result_folder.empty()
                              ? std::optional<std::string>()
                              : std::optional{result_folder}};
}

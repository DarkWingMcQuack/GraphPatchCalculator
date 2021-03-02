#pragma once

#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace utils {

class ProgramOptions
{
public:
    ProgramOptions(std::size_t prune_distance,
                   std::string graph_file,
                   std::size_t maximum_number_of_selections_per_node,
                   std::optional<std::string> fmi_graph_file,
                   std::optional<std::string> result_folder = std::nullopt);

    auto getGraphFile() const noexcept
        -> std::string_view;

    auto hasResultFolder() const noexcept
        -> bool;

    auto getResultFolder() const noexcept
        -> std::string_view;

    auto getPruneDistance() const noexcept
        -> std::size_t;

    auto getMaxNumberOfSelectionsPerNode() const noexcept
        -> std::size_t;

    auto hasFmiGraphFile() const noexcept
        -> bool;

    auto getFmiGraphFile() const noexcept
        -> std::string_view;

private:
    std::size_t prune_distance_;
    std::string graph_file_;
    std::size_t maximum_number_of_selections_per_node_;
    std::optional<std::string> fmi_graph_file_;
    std::optional<std::string> separation_folder_;
};

auto parseArguments(int argc, char* argv[])
    -> ProgramOptions;

} // namespace utils

#pragma once

#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace utils {

class ProgramOptions
{
public:
    ProgramOptions(std::string graph_file,
                   std::optional<std::string> result_folder = std::nullopt);

    auto getGraphFile() const noexcept
        -> std::string_view;

    auto hasResultFolder() const noexcept
        -> bool;

    auto getResultFolder() const noexcept
        -> std::string_view;

private:
    std::string graph_file_;
    std::optional<std::string> separation_folder_;
};

auto parseArguments(int argc, char* argv[])
    -> ProgramOptions;

} // namespace utils

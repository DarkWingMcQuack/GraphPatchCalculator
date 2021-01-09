#include <execution>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <functional>
#include <graph/Graph.hpp>
#include <numeric>
#include <optional>
#include <pathfinding/Path.hpp>
#include <string_view>
#include <vector>


using graph::Node;
using pathfinding::Path;


Path::Path(std::vector<graph::Node> path) noexcept
    : path_(std::move(path)) {}


auto Path::pushBack(graph::Node node) noexcept -> void
{
    path_.emplace_back(node);
}
auto Path::pushFront(graph::Node node) noexcept -> void
{
    path_.emplace(std::begin(path_),
                  node);
}

auto Path::getLength() const noexcept -> std::size_t
{
    //-1 because when we have source-target we need - 1 to calculate the number
    // of edges between them
    return path_.size() - 1;
}

auto Path::getSource() const noexcept -> const graph::Node&
{
    return path_.front();
}

auto Path::getSource() noexcept -> graph::Node&
{
    return path_.front();
}

auto Path::getTarget() const noexcept -> const graph::Node&
{
    return path_.back();
}

auto Path::getTarget() noexcept -> graph::Node&
{
    return path_.back();
}

auto Path::getNodes() const noexcept
    -> const std::vector<graph::Node>&
{
    return path_;
}
auto Path::getNodes() noexcept
    -> std::vector<graph::Node>&
{
    return path_;
}

auto Path::contains(const graph::Node& node) const noexcept
    -> bool
{
    return std::find(std::cbegin(path_),
                     std::cend(path_),
                     node)
        != std::cend(path_);
}


auto Path::getMiddleNode() const noexcept
    -> std::optional<graph::Node>
{
    if(path_.empty()) {
        return std::nullopt;
    }

    auto middle_index = static_cast<std::size_t>(std::floor(path_.size() / 2.));

    return path_[middle_index];
}


auto pathfinding::operator<<(std::ostream& os, const Path& p) noexcept
    -> std::ostream&
{
    return os << fmt::format("{}", p.path_);
}

#pragma once

#include <nonstd/span.hpp>
#include <pathfinding/Distance.hpp>
#include <vector>

namespace graph {

using Node = std::uint_fast32_t;

static constexpr inline auto NOT_REACHABLE = std::numeric_limits<Node>::max();

class Graph
{
public:
    Graph(const std::vector<std::vector<std::pair<Node, Distance>>>& adj_list) noexcept;

    auto getNeigboursOf(Node node) const noexcept
        -> nonstd::span<const std::pair<Node, Distance>>;

    auto size() const noexcept
        -> std::size_t;

private:
    std::vector<std::pair<Node, Distance>> neigbours_;
    std::vector<size_t> offset_;
};

auto parseFMIFile(std::string_view path) noexcept
    -> std::optional<Graph>;

} // namespace graph

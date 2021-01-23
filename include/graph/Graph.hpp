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
    Graph(const std::vector<std::vector<std::pair<Node, Distance>>>& adj_list,
          std::vector<double> lats,
          std::vector<double> lngs) noexcept;

    auto getForwardNeigboursOf(Node node) const noexcept
        -> nonstd::span<const std::pair<Node, Distance>>;

    auto getBackwardNeigboursOf(Node node) const noexcept
        -> nonstd::span<const std::pair<Node, Distance>>;

    auto size() const noexcept
        -> std::size_t;

    auto forwardEdgeExists(Node from, Node to) const noexcept
        -> bool;

    auto backwardEdgeExists(Node from, Node to) const noexcept
        -> bool;

    auto getLatLng(Node n) const noexcept
        -> std::pair<double, double>;

private:
    std::vector<std::pair<Node, Distance>> forward_neigbours_;
    std::vector<size_t> forward_offset_;

    std::vector<std::pair<Node, Distance>> backward_neigbours_;
    std::vector<size_t> backward_offset_;

    std::vector<double> lats_;
    std::vector<double> lngs_;
};

auto parseFMIFile(std::string_view path) noexcept
    -> std::optional<Graph>;

} // namespace graph

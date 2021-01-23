#include "utils/Range.hpp"
#include <fmt/core.h>
#include <fstream>
#include <graph/Graph.hpp>
#include <nonstd/span.hpp>
#include <pathfinding/Distance.hpp>
#include <utils/Utils.hpp>

using graph::Graph;

namespace {

auto adjListToOffsetArray(std::vector<std::vector<std::pair<graph::Node, graph::Distance>>> adj_list)
    -> std::pair<std::vector<std::pair<graph::Node, graph::Distance>>,
                 std::vector<size_t>>
{
    std::vector<std::pair<graph::Node, graph::Distance>> neigbours;
    std::vector<std::size_t> offset(adj_list.size() + 1);

    for(auto i = 0; i < adj_list.size(); i++) {
        auto neigs = std::move(adj_list[i]);

        std::sort(std::begin(neigs),
                  std::end(neigs),
                  [](auto lhs, auto rhs) {
                      return lhs.first < rhs.first;
                  });

        neigbours.insert(std::end(neigbours),
                         std::begin(neigs),
                         std::end(neigs));

        offset[i + 1] = neigbours.size();
    }

    neigbours.emplace_back(std::numeric_limits<graph::Node>::max(),
                           graph::UNREACHABLE);

    return std::pair{std::move(neigbours),
                     std::move(offset)};
}

auto reverseAdjList(const std::vector<std::vector<std::pair<graph::Node, graph::Distance>>>& adj_list)
    -> std::vector<std::vector<std::pair<graph::Node, graph::Distance>>>
{
    std::vector<std::vector<std::pair<graph::Node, graph::Distance>>> reverse_list(adj_list.size());

    for(const auto& [source, neigs] : utils::enumerate(adj_list)) {
        for(auto [target, dist] : neigs) {
            reverse_list[target].emplace_back(source, dist);
        }
    }

    return reverse_list;
}

} // namespace

Graph::Graph(const std::vector<std::vector<std::pair<Node, Distance>>>& adj_list,
             std::vector<double> lats,
             std::vector<double> lngs) noexcept
    : lats_(std::move(lats)),
      lngs_(std::move(lngs))
{
    auto backward_adj_list = reverseAdjList(adj_list);

    auto [forward_neigs, forward_offset] = adjListToOffsetArray(adj_list);
    forward_neigbours_ = std::move(forward_neigs);
    forward_offset_ = std::move(forward_offset);


    auto [backward_neigs, backward_offset] = adjListToOffsetArray(backward_adj_list);
    backward_neigbours_ = std::move(backward_neigs);
    backward_offset_ = std::move(backward_offset);
}

auto Graph::getForwardNeigboursOf(Node node) const noexcept
    -> nonstd::span<const std::pair<Node, Distance>>
{
    const auto start_offset = forward_offset_[node];
    const auto end_offset = forward_offset_[node + 1];
    const auto* start = &forward_neigbours_[start_offset];
    const auto* end = &forward_neigbours_[end_offset];

    return nonstd::span{start, end};
}

auto Graph::getBackwardNeigboursOf(Node node) const noexcept
    -> nonstd::span<const std::pair<Node, Distance>>
{
    const auto start_offset = backward_offset_[node];
    const auto end_offset = backward_offset_[node + 1];
    const auto* start = &backward_neigbours_[start_offset];
    const auto* end = &backward_neigbours_[end_offset];

    return nonstd::span{start, end};
}

auto Graph::forwardEdgeExists(Node from, Node to) const noexcept
    -> bool
{
    auto neigs = getForwardNeigboursOf(from);
    auto search_edge = std::pair{to, 0};

    return std::binary_search(std::begin(neigs),
                              std::end(neigs),
                              search_edge,
                              [](auto lhs, auto rhs) {
                                  return lhs.first < rhs.first;
                              });
}

auto Graph::backwardEdgeExists(Node from, Node to) const noexcept
    -> bool
{
    auto neigs = getForwardNeigboursOf(from);
    auto search_edge = std::pair{to, 0};

    return std::binary_search(std::begin(neigs),
                              std::end(neigs),
                              search_edge,
                              [](auto lhs, auto rhs) {
                                  return lhs.first < rhs.first;
                              });
}

auto Graph::size() const noexcept
    -> std::size_t
{
    return forward_offset_.size() - 1;
}

auto Graph::getLatLng(Node n) const noexcept
    -> std::pair<double, double>
{
    return std::pair{lats_[n],
                     lngs_[n]};
}


auto graph::parseFMIFile(std::string_view path) noexcept
    -> std::optional<Graph>
{
    // Open the File
    std::ifstream in{path.data()};

    // Check if object is valid
    if(!in) {
        fmt::print("unable to open file {}\n", path);
        return std::nullopt;
    }

    std::string str;
    //skip the comments
    while(std::getline(in, str)) {
        if(str[0] == '#') {
            continue;
        } else {
            break;
        }
    }

    std::uint_fast32_t number_of_nodes;
    std::uint_fast32_t number_of_edges;

    in >> number_of_nodes >> number_of_edges;

    Node node;
    Node id2;
    double latitude;
    double longitude;
    int elevation;

    std::vector<double> lats;
    std::vector<double> lngs;

    for(size_t i{0}; i < number_of_nodes; i++) {
        in >> node >> id2 >> latitude >> longitude >> elevation;
        lats.emplace_back(latitude);
        lngs.emplace_back(longitude);
    }

    Node from;
    Node to;
    Distance cost;
    int speed;
    int type;

    std::vector edges(number_of_nodes,
                      std::vector<std::pair<Node, Distance>>{});

    for(size_t i{0}; i < number_of_edges; i++) {
        in >> from >> to >> cost >> speed >> type;
        edges[from].emplace_back(to, cost);
    }

    return Graph{edges,
                 std::move(lats),
                 std::move(lngs)};
}

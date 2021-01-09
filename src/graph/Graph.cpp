#include <fstream>
#include <graph/Graph.hpp>
#include <nonstd/span.hpp>
#include <pathfinding/Distance.hpp>
#include <utils/Utils.hpp>

using graph::Graph;

Graph::Graph(const std::vector<std::vector<std::pair<Node, Distance>>>& adj_list) noexcept
    : offset_(adj_list.size() + 1, 0)
{
    for(auto i = 0; i < adj_list.size(); i++) {
        const auto& neigs = adj_list[i];
        neigbours_.insert(std::end(neigbours_),
                          std::begin(neigs),
                          std::end(neigs));

        offset_[i + 1] = neigbours_.size();
    }

    neigbours_.emplace_back(std::numeric_limits<Node>::max(), UNREACHABLE);
}

auto Graph::getNeigboursOf(Node node) const noexcept
    -> nonstd::span<const std::pair<Node, Distance>>
{
    const auto start_offset = offset_[node];
    const auto end_offset = offset_[node + 1];
    const auto* start = &neigbours_[start_offset];
    const auto* end = &neigbours_[end_offset];

    return nonstd::span{start, end};
}

auto Graph::size() const noexcept
    -> std::size_t
{
    return offset_.size() - 1;
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

    fmt::print("number of nodes {}\nnumber of edges {}\n",
               number_of_nodes,
               number_of_edges);

    Node node;
    Node id2;
    double latitude;
    double longitude;
    int elevation;

    for(size_t i{0}; i < number_of_nodes; i++) {
        in >> node >> id2 >> latitude >> longitude >> elevation;
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
        edges[from].emplace_back(cost, to);
    }

    return Graph{edges};
}

#include <execution>
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fstream>
#include <graph/Graph.hpp>
#include <map>
#include <pathfinding/Distance.hpp>
#include <queue>
#include <random>
#include <selection/NodeSelection.hpp>
#include <selection/SelectionLookup.hpp>
#include <unordered_set>
#include <utils/Range.hpp>
#include <utils/Utils.hpp>
#include <vector>

using selection::SelectionLookup;
using selection::NodeSelection;

SelectionLookup::SelectionLookup(std::size_t number_of_nodes,
                                 std::vector<graph::Node> centers,
                                 std::vector<CenterSet> source_selections,
                                 std::vector<CenterSet> target_selections)
    : number_of_nodes_(number_of_nodes),
      centers_(std::move(centers)),
      source_selections_(std::move(source_selections)),
      target_selections_(std::move(target_selections)) {}


auto SelectionLookup::getSizeDistributionSource() const noexcept
    -> std::map<std::size_t, std::size_t>
{
    std::map<std::size_t, std::size_t> ret_map;
    for(auto n : utils::range(number_of_nodes_)) {
        const auto& s = source_selections_[n];

        auto iter = ret_map.find(s.size());

        if(iter == std::end(ret_map)) {
            ret_map[s.size()] = 1;
        } else {
            iter->second++;
        }
    }

    return ret_map;
}


auto SelectionLookup::getSelectionAnswering(const graph::Node& source,
                                            const graph::Node& target) const noexcept
    -> graph::Distance
{
    const auto& first = source_selections_[source];
    const auto& second = target_selections_[target];

    const auto first_size = first.size();
    const auto second_size = second.size();
    auto first_idx = 0;
    auto second_idx = 0;

    while(first_idx < first_size and second_idx < second_size) {
        const auto [f_i, f_d] = first[first_idx];
        const auto [s_i, s_d] = second[second_idx];

        if(f_i == s_i) {
            return f_d + s_d;
        }

        if(f_i < s_i) {
            first_idx++;
        } else {
            second_idx++;
        }
    }

    return graph::UNREACHABLE;
}

auto SelectionLookup::getSizeDistributionTarget() const noexcept
    -> std::map<std::size_t, std::size_t>
{
    std::map<std::size_t, std::size_t> ret_map;
    for(auto n : utils::range(number_of_nodes_)) {
        const auto& s = target_selections_[n];

        auto iter = ret_map.find(s.size());


        if(iter == std::end(ret_map)) {
            ret_map[s.size()] = 1;
        } else {
            iter->second++;
        }
    }

    return ret_map;
}

auto SelectionLookup::getSizeDistributionTotal() const noexcept
    -> std::map<std::size_t, std::size_t>
{
    std::map<std::size_t, std::size_t> ret_map;
    for(auto n : utils::range(number_of_nodes_)) {
        const auto& s = source_selections_[n];
        const auto& t = target_selections_[n];

        auto key = s.size() + t.size();

        auto iter = ret_map.find(key);

        if(iter == std::end(ret_map)) {
            ret_map[key] = 1;
        } else {
            iter->second++;
        }
    }

    return ret_map;
}

auto SelectionLookup::averageSelectionsPerNode() const noexcept
    -> double
{
    auto selections_total = std::accumulate(std::begin(source_selections_),
                                            std::end(source_selections_),
                                            0,
                                            [](auto current, const auto& s) {
                                                return current + s.size();
                                            })
        + std::accumulate(std::begin(target_selections_),
                          std::end(target_selections_),
                          0,
                          [](auto current, const auto& s) {
                              return current + s.size();
                          });

    return static_cast<double>(selections_total)
        / static_cast<double>(number_of_nodes_);
}


template<>
struct fmt::formatter<std::pair<std::size_t, graph::Distance>> : fmt::formatter<std::string_view>
{
    // parse is inherited from formatter<string_view>.
    template<typename FormatContext>
    auto format(std::pair<std::size_t, graph::Distance> c, FormatContext& ctx)
    {
        auto name = fmt::format("({},{})", c.first, c.second);
        return formatter<string_view>::format(name, ctx);
    }
};

auto SelectionLookup::toFile(std::string_view path) const noexcept
    -> void
{
    std::ofstream file{path.data()};

    for(auto node : utils::range(number_of_nodes_)) {
        file << node << fmt::format("{}", fmt::join(source_selections_[node], ",")) << "\n";
        file << node << fmt::format("{}", fmt::join(target_selections_[node], ",")) << "\n";
    }
}

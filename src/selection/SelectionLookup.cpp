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


auto SelectionLookup::getSelectionAnswering(const graph::Node& source,
                                            const graph::Node& target) const noexcept
    -> std::optional<std::pair<graph::Node, graph::Distance>>
{
    const auto& first_source_selections = source_selections_[source];
    const auto& second_target_selections = target_selections_[target];

    return getCommonCenter(source,
                           target,
                           first_source_selections,
                           second_target_selections);
}

auto SelectionLookup::getCommonCenter(graph::Node source,
                                      graph::Node target,
                                      const CenterSet& first,
                                      const CenterSet& second) const noexcept
    -> std::optional<std::pair<graph::Node, graph::Distance>>
{
    auto iter1 = std::cbegin(first);
    auto iter2 = std::cbegin(second);
    auto iter1_end = std::cend(first);
    auto iter2_end = std::cend(second);

    while(iter1 != iter1_end and iter2 != iter2_end) {
        auto center1 = centers_[iter1->first];

        if(center1 == target) {
            auto distance = iter1->second;
            return std::pair{center1, distance};
        }

        auto center2 = centers_[iter2->first];
        if(center2 == source) {
            auto distance = iter2->second;
            return std::pair{center2, distance};
        }

        if(iter1->first < iter2->first) {
            ++iter1;
        } else if(iter2->first < iter1->first) {
            ++iter2;
        } else {
            auto common_center = centers_[iter2->first];
            auto distance = iter1->second + iter2->second;
            return std::pair{common_center, distance};
        }
    }

    if(iter1 != iter1_end) {
        while(iter1 != iter1_end) {
            auto center1 = centers_[iter1->first];
            if(center1 == target) {
                auto distance = iter1->second;
                return std::pair{center1, distance};
            }

            iter1++;
        }
    }

    if(iter2 != iter2_end) {
        while(iter2 != iter2_end) {
            auto center2 = centers_[iter2->first];
            if(center2 == source) {
                auto distance = iter2->second;
                return std::pair{center2, distance};
            }

            iter2++;
        }
    }

    return std::nullopt;
}

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

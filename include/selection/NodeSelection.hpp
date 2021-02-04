#pragma once

#include <graph/Graph.hpp>
#include <nlohmann/json.hpp>
#include <string_view>
#include <vector>

namespace selection {

using Patch = std::vector<std::pair<graph::Node, graph::Distance>>;

class NodeSelection
{
public:
    NodeSelection(Patch source_patch,
                  Patch target_patch,
                  graph::Node center);

    [[nodiscard]] auto getSourcePatch() const noexcept
        -> const Patch&;

    [[nodiscard]] auto getTargetPatch() const noexcept
        -> const Patch&;

    [[nodiscard]] auto getSourcePatch() noexcept
        -> Patch&;

    [[nodiscard]] auto getTargetPatch() noexcept
        -> Patch&;

    [[nodiscard]] auto canAnswer(graph::Node from, graph::Node to) const noexcept
        -> bool;

    [[nodiscard]] auto getCenter() const noexcept
        -> graph::Node;

    [[nodiscard]] auto isSubSetOf(const NodeSelection& other) const noexcept
        -> bool;

    [[nodiscard]] auto weight() const noexcept
        -> std::size_t;

    [[nodiscard]] auto averageDistance() const noexcept
        -> graph::Distance;

    auto deleteFromSource(const std::vector<graph::Node>& nodes) noexcept
        -> void;
    auto deleteFromTarget(const std::vector<graph::Node>& nodes) noexcept
        -> void;

    auto toFile(std::string_view path) const noexcept
        -> void;

    auto toLatLngFiles(std::string_view path, const graph::Graph& graph) const noexcept
        -> void;

    auto toJson(const graph::Graph& graph) const noexcept
        -> nlohmann::json;

    auto toFileAsJson(std::string_view path, const graph::Graph& graph) const noexcept
        -> void;

private:
    Patch source_patch_;
    Patch target_patch_;
    graph::Node center_;
};

} // namespace selection

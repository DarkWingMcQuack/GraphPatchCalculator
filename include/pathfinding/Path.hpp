#pragma once

#include <functional>
#include <graph/Graph.hpp>
#include <optional>
#include <string_view>
#include <vector>

namespace pathfinding {

class Path
{
public:
    Path(std::vector<graph::Node> path = {}) noexcept;
    Path(Path&&) = default;
    Path(const Path&) = default;

    auto operator=(Path&&) -> Path& = default;
    auto operator=(const Path&) -> Path& = default;

    auto pushBack(graph::Node node) noexcept -> void;
    auto pushFront(graph::Node node) noexcept -> void;

    auto size() const noexcept -> std::size_t;
    auto getLength() const noexcept -> std::size_t;
    auto empty() const noexcept -> bool;

    auto getSource() const noexcept -> const graph::Node&;
    auto getSource() noexcept -> graph::Node&;

    auto getTarget() const noexcept -> const graph::Node&;
    auto getTarget() noexcept -> graph::Node&;

    auto getNodes() const noexcept
        -> const std::vector<graph::Node>&;
    auto getNodes() noexcept
        -> std::vector<graph::Node>&;

    auto contains(const graph::Node&) const noexcept
        -> bool;

    auto getMiddleNode() const noexcept
        -> std::optional<graph::Node>;

private:
    friend auto operator<<(std::ostream& os, const Path& p) noexcept
        -> std::ostream&;

    std::vector<graph::Node> path_;
};

auto operator<<(std::ostream& os, const Path& p) noexcept
    -> std::ostream&;
} // namespace pathfinding

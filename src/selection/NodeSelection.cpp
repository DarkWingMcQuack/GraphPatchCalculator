#include <algorithm>
#include <fstream>
#include <numeric>
#include <selection/NodeSelection.hpp>

using selection::NodeSelection;
using selection::Patch;
using graph::Node;

NodeSelection::NodeSelection(Patch source_patch,
                             Patch target_patch,
                             graph::Node center)
    : source_patch_(std::move(source_patch)),
      target_patch_(std::move(target_patch)),
      center_(center) {}


auto NodeSelection::weight() const noexcept
    -> std::size_t
{
    return source_patch_.size() * target_patch_.size();
}

auto NodeSelection::averageDistance() const noexcept
    -> graph::Distance
{
    auto source_center = std::transform_reduce(std::begin(source_patch_),
                                               std::end(source_patch_),
                                               0,
                                               std::plus<>{},
                                               [](auto pair) {
                                                   return pair.second;
                                               })
        / source_patch_.size();

    auto center_target = std::transform_reduce(std::begin(target_patch_),
                                               std::end(target_patch_),
                                               0,
                                               std::plus<>{},
                                               [](auto pair) {
                                                   return pair.second;
                                               })
        / target_patch_.size();

    return source_center + center_target;
}

auto NodeSelection::getSourcePatch() const noexcept
    -> const Patch&
{
    return source_patch_;
}

auto NodeSelection::getSourcePatch() noexcept
    -> Patch&
{
    return source_patch_;
}

auto NodeSelection::getTargetPatch() noexcept
    -> Patch&
{
    return target_patch_;
}

auto NodeSelection::getTargetPatch() const noexcept
    -> const Patch&
{
    return target_patch_;
}

auto NodeSelection::getCenter() const noexcept
    -> graph::Node
{
    return center_;
}

auto NodeSelection::deleteFromSource(const std::vector<graph::Node>& nodes) noexcept
    -> void
{
    for(auto n : nodes) {
        auto n_search = std::pair{n, 0};
        auto iter = std::lower_bound(std::begin(source_patch_),
                                     std::end(source_patch_),
                                     n_search,
                                     [](auto lhs, auto rhs) {
                                         return lhs.first < rhs.first;
                                     });
        if(iter != std::end(source_patch_) and n == iter->first) {
            source_patch_.erase(iter);
        }
    }
}

auto NodeSelection::deleteFromTarget(const std::vector<graph::Node>& nodes) noexcept
    -> void
{
    for(auto n : nodes) {
        auto n_search = std::pair{n, 0};
        auto iter = std::lower_bound(std::begin(target_patch_),
                                     std::end(target_patch_),
                                     n_search,
                                     [](auto lhs, auto rhs) {
                                         return lhs.first < rhs.first;
                                     });
        if(iter != std::end(source_patch_) and n == iter->first) {
            source_patch_.erase(iter);
        }
    }
}

namespace {

template<class T, class U, class V>
auto firstLessCompare(const std::pair<T, U>& lhs, const std::pair<T, V> rhs) noexcept
    -> bool
{
    return lhs.first < rhs.first;
}

} // namespace

auto NodeSelection::isSubSetOf(const NodeSelection& other) const noexcept
    -> bool
{
    return (std::includes(std::begin(other.source_patch_),
                          std::end(other.source_patch_),
                          std::begin(source_patch_),
                          std::end(source_patch_),
                          [](auto lhs, auto rhs) {
                              return lhs.first < rhs.first;
                          })
            and std::includes(std::begin(other.target_patch_),
                              std::end(other.target_patch_),
                              std::begin(target_patch_),
                              std::end(target_patch_),
                              [](auto lhs, auto rhs) {
                                  return lhs.first < rhs.first;
                              }))
        or (std::includes(std::begin(other.target_patch_),
                          std::end(other.target_patch_),
                          std::begin(source_patch_),
                          std::end(source_patch_),
                          [](auto lhs, auto rhs) {
                              return lhs.first < rhs.first;
                          })
            and std::includes(std::begin(other.source_patch_),
                              std::end(other.source_patch_),
                              std::begin(target_patch_),
                              std::end(target_patch_),
                              [](auto lhs, auto rhs) {
                                  return lhs.first < rhs.first;
                              }));
}

auto NodeSelection::canAnswer(Node from, Node to) const noexcept
    -> bool
{
    auto from_search = std::pair{from, 0};
    auto to_search = std::pair{to, 0};

    return std::binary_search(std::begin(source_patch_),
                              std::end(source_patch_),
                              from_search,
                              [](auto lhs, auto rhs) {
                                  return lhs.first < rhs.first;
                              })
        and std::binary_search(std::begin(target_patch_),
                               std::end(target_patch_),
                               to_search,
                               [](auto lhs, auto rhs) {
                                   return lhs.first < rhs.first;
                               });
}

auto NodeSelection::toFile(std::string_view path) const noexcept
    -> void
{
    std::ofstream file{path.data()};
    for(auto [node, dist] : source_patch_) {
        file << "0: (" << node << ", " << dist << ")\n";
    }
    for(auto [node, dist] : target_patch_) {
        file << "0: (" << node << ", " << dist << ")\n";
    }
    file << "center: " << center_ << "\n";
}


auto NodeSelection::toLatLngFiles(std::string_view path, const graph::Graph& graph) const noexcept
    -> void
{
    std::string source_file = path.data();
    source_file += "-source";

    std::string target_file = path.data();
    target_file += "-target";

    std::string center_file = path.data();
    center_file += "-center";

    std::ofstream src_file{source_file};
    src_file << "# lng,\tlat\n";
    for(auto [node, dist] : source_patch_) {
        auto [lat, lng] = graph.getLatLng(node);
        src_file << lng << ",\t" << lat << "\n";
    }

    std::ofstream trg_file{target_file};
    trg_file << "# lng,\tlat\n";
    for(auto [node, dist] : target_patch_) {
        auto [lat, lng] = graph.getLatLng(node);
        src_file << lng << ",\t" << lat << "\n";
    }

    std::ofstream c_file{center_file};
    c_file << "# lng,\tlat\n";
    auto [lat, lng] = graph.getLatLng(center_);
    c_file << lng << ",\t" << lat << "\n";
}

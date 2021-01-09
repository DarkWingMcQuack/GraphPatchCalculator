#pragma once

#include <limits>
#include <cstdint>

namespace graph {

using Distance = std::int64_t;
constexpr inline auto UNREACHABLE = std::numeric_limits<Distance>::max();

} // namespace graph

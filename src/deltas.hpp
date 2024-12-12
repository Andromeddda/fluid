#ifndef HEADER_GUARD_DELTAS_HPP
#define HEADER_GUARD_DELTAS_HPP

#include <utility>
#include <array>

namespace fluid
{
    constexpr std::array<std::pair<int, int>, 4> deltas{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
} // namespace fluid

#endif // HEADER_GUARD_DELTAS_HPP
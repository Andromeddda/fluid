#ifndef HEADER_GUARD_RANDOM_HPP
#define HEADER_GUARD_RANDOM_HPP

#include "fixed.hpp"

#include <utility>
#include <random>

namespace fluid
{
    std::mt19937 rnd(1337);

    template<typename T>
    T random01() 
    {
        if constexpr (std::is_same_v<T, float> or std::is_same_v<T, double>) 
            return T(rnd()) / T(std::mt19937::max());
        else
            return T::from_raw((rnd() & ((1 << T::k) - 1)));
    }
} // namespace fluid

#endif // HEADER_GUARD_RANDOM_HPP
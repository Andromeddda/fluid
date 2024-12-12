#ifndef HEADER_GUARD_VECTOR_FIELD_HPP
#define HEADER_GUARD_VECTOR_FIELD_HPP

#include <cstdio>
#include <string>
#include <cstring>
#include <ostream>
#include <utility>
#include <array>
#include <bits/stdc++.h>

#include "deltas.hpp"

namespace fluid
{
    template <typename Type, size_t N, size_t M>
    struct VectorField 
    {
        std::array<Type, deltas.size()> v[N][M];

        Type& add(int x, int y, int dx, int dy, Type dv);
        Type& get(int x, int y, int dx, int dy);
    };

    template <typename Type, size_t N, size_t M>
    Type& VectorField<Type, N, M>::add(int x, int y, int dx, int dy, Type dv) 
    {
        return get(x, y, dx, dy) += dv;
    }

    template <typename Type, size_t N, size_t M>
    Type& VectorField<Type, N, M>::get(int x, int y, int dx, int dy) 
    {
        size_t i = std::ranges::find(deltas, std::pair(dx, dy)) - deltas.begin();
        assert(i < deltas.size());
        return v[x][y][i];
    }

} // namespace fluid

#endif // HEADER_GUARD_VECTOR_FIELD_HPP
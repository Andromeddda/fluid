#ifndef HEADER_GUARD_VECTOR_FIELD_HPP
#define HEADER_GUARD_VECTOR_FIELD_HPP

#include <cstdio>
#include <string>
#include <cstring>
#include <ostream>
#include <utility>
#include <array>

#include "deltas.hpp"
#include "matrix_processing.hpp"

namespace fluid
{
    template <typename Type, size_t... SizeArgs>
    struct VectorField 
    {
        VectorField(size_t n, size_t m) : v(n, m) {}

        using vector_t = typename std::array<Type, deltas.size()>;

        MatrixType<vector_t, SizeArgs...>::type v;

        Type& add(int x, int y, int dx, int dy, Type dv);
        Type& get(int x, int y, int dx, int dy);
    };

    template <typename Type, size_t... SizeArgs>
    Type& VectorField<Type, SizeArgs...>::add(int x, int y, int dx, int dy, Type dv) 
    {
        return get(x, y, dx, dy) += dv;
    }

    template <typename Type, size_t... SizeArgs>
    Type& VectorField<Type, SizeArgs...>::get(int x, int y, int dx, int dy) 
    {
        size_t i = std::ranges::find(deltas, std::pair(dx, dy)) - deltas.begin();
        assert(i < deltas.size());
        return v[x][y][i];
    }

} // namespace fluid

#endif // HEADER_GUARD_VECTOR_FIELD_HPP
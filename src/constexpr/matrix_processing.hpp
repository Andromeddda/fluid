#ifndef HEADER_GUARD_MATRIX_PROCESSING_HPP
#define HEADER_GUARD_MATRIX_PROCESSING_HPP

#include "matrix.hpp"

namespace fluid
{
    template <typename T, size_t ...SizeArgs>
    struct MatrixType;

    template <typename T>
    struct MatrixType<T>        { using type = DynamicMatrix<T>; };

    template <typename T, size_t N, size_t M>
    struct MatrixType<T, N, M>  { using type = StaticMatrix<T, N, M>; };

    // check if two size_t variables match the template arguments
    template <size_t ...SizeArgs>
    struct SizesMatch;

    template <>
    struct SizesMatch<>
    {
        size_t n, m;
        SizesMatch(size_t n, size_t m) : n(n), m(m) {}

        operator bool() const { return true; }
    };

    template <size_t N, size_t M>
    struct SizesMatch<N, M>
    {
        size_t n, m;
        SizesMatch(size_t n, size_t m) : n(n), m(m) {}

        operator bool() const { return (n == N) && (m == M); }
    };

    // Get sizes

    template <size_t ...SizeArgs>
    struct GetSizes;

    template <>
    struct GetSizes<>
    {
        static const size_t n = 1, m = 1;
    };

    template <size_t N, size_t M>
    struct GetSizes<N, M>
    {
        static const size_t n = N, m = M;
    };




} // namespace fluid


#endif // HEADER_GUARD_MATRIX_PROCESSING_HPP
#ifndef HEADER_GUARD_FIXED_FIXED_HPP
#define HEADER_GUARD_FIXED_FIXED_HPP

#include <bits/stdc++.h>
#include <ostream>
#include <cstdint>
#include <type_traits>

namespace fluid
{
    template <size_t N> 
        requires (N <= 64)
    class int_t
    {
    private:
        using more_then_16 = std::conditional<N <= 32, int32_t, int64_t>::type;
        using more_then_8 = std::conditional<N <= 16, int16_t, more_then_16>::type;
    public:
        using type = std::conditional<N <= 8, int8_t, more_then_8>::type;
    };

    template <size_t N> 
        requires (N <= 64)
    class fast_int_t
    {
    private:
        using more_then_16 = std::conditional<N <= 32, int_fast32_t, int_fast64_t>::type;
        using more_then_8 = std::conditional<N <= 16, int_fast16_t, more_then_16>::type;
    public:
        using type = std::conditional<N <= 8, int_fast8_t, more_then_8>::type;
    };


    template<size_t N, size_t K, bool Fast = false>
    struct Fixed
    {
        // choose between int_t and fast_int_t
        using type = 
            typename std::conditional<Fast, typename fast_int_t<N>::type, typename int_t<N>::type >::type;

        type v;

        static const size_t n = N;
        static const size_t k = K;

        constexpr Fixed() : v(0) {}
        constexpr Fixed(int v);
        constexpr Fixed(float f);
        constexpr Fixed(double f);

        template <size_t N2, size_t K2>
        constexpr Fixed(const Fixed<N2, K2, Fast>& other);

        template <size_t N2, size_t K2>
        Fixed& operator= (const Fixed<N2, K2, Fast>& other);

        static constexpr Fixed from_raw(int x);

        auto operator<=>(const Fixed&) const = default;
        bool operator==(const Fixed&) const = default;
    };

    // Construct from fundamental type

    template<size_t N, size_t K, bool Fast>
    constexpr Fixed<N, K, Fast>::Fixed(int v) : v(v << K) {}

    template<size_t N, size_t K, bool Fast>
    constexpr Fixed<N, K, Fast>::Fixed(float f) : v(f*(1 << K)) {}

    template<size_t N, size_t K, bool Fast>
    constexpr Fixed<N, K, Fast>::Fixed(double f) : v(f*(1 << K)) {}

    // Convertion from Fixed to another Fixed

    template <size_t N1, size_t K1, bool Fast>
    template <size_t N2, size_t K2>
    constexpr Fixed<N1, K1, Fast>::Fixed(const Fixed<N2, K2, Fast>& other)
    {
        if constexpr (K1 >= K2)
            v = other.v << (K1 - K2);
        else    
            v = other.v >> (K2 - K1);
    }

    // static construct from raw int 

    template<size_t N, size_t K, bool Fast>
    constexpr Fixed<N, K, Fast> Fixed<N, K, Fast>::from_raw(int x)
    {
        Fixed<N, K, Fast> ret;
        ret.v = x;
        return ret;
    }

    // operations

    template <size_t N, size_t K, bool Fast, typename T>
    Fixed<N, K, Fast> operator+(Fixed<N, K, Fast> a, T b) 
    {
        return Fixed<N, K, Fast>::from_raw(a.v + Fixed<N, K, Fast>(b).v);
    }

    template <size_t N, size_t K, bool Fast, typename T>
    Fixed<N, K, Fast> operator-(Fixed<N, K, Fast> a, T b) 
    {
        return Fixed<N, K, Fast>::from_raw(a.v - Fixed<N, K, Fast>(b).v);
    }

    template <size_t N, size_t K, bool Fast, typename T>
    Fixed<N, K, Fast> operator*(Fixed<N, K, Fast> a, T b) 
    {
        return Fixed<N, K, Fast>::from_raw(((int64_t) a.v * Fixed<N, K, Fast>(b).v) >> 16);
    }

    template <size_t N, size_t K, bool Fast, typename T>
    Fixed<N, K, Fast> operator/ (Fixed<N, K, Fast> a, T b) 
    {
        return Fixed<N, K, Fast>::from_raw(((int64_t) a.v << 16) / Fixed<N, K, Fast>(b).v);
    }

    template <size_t N, size_t K, bool Fast, typename T>
    bool operator== (Fixed<N, K, Fast> a, T b) 
    {
        return a == Fixed<N, K, Fast>(b);
    }


    template <size_t N, size_t K, bool Fast, typename T>
    Fixed<N, K, Fast>& operator+=(Fixed<N, K, Fast> &a, T b) 
    {
        return a = a + Fixed<N, K, Fast>(b);
    }

    template <size_t N, size_t K, bool Fast, typename T>
    Fixed<N, K, Fast>& operator-=(Fixed<N, K, Fast> &a, T b) 
    {
        return a = a - Fixed<N, K, Fast>(b);
    }

    template <size_t N, size_t K, bool Fast, typename T>
    Fixed<N, K, Fast>& operator*=(Fixed<N, K, Fast> &a, T b) 
    {
        return a = a * Fixed<N, K, Fast>(b);
    }

    template <size_t N, size_t K, bool Fast, typename T>
    Fixed<N, K, Fast>& operator/=(Fixed<N, K, Fast> &a, T b) 
    {
        return a = a / Fixed<N, K, Fast>(b);
    }

    template <size_t N, size_t K, bool Fast>
    Fixed<N, K, Fast> operator-(Fixed<N, K, Fast> x) 
    {
        return Fixed<N, K, Fast>::from_raw(-x.v);
    }

    template <size_t N, size_t K, bool Fast>
    Fixed<N, K, Fast> abs(Fixed<N, K, Fast> x) 
    {
        if (x.v < 0) {
            x.v = -x.v;
        }
        return x;
    }

    template <size_t N, size_t K, bool Fast>
    std::ostream &operator<<(std::ostream &out, Fixed<N, K, Fast> x) 
    {
        return out << x.v / (double) (1 << 16);
    }
} // namespace fluid

#endif // HEADER_GUARD_FIXED_FIXED_HPP
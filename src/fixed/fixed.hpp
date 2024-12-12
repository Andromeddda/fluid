#ifndef HEADER_GUARD_FIXED_FIXED_HPP
#define HEADER_GUARD_FIXED_FIXED_HPP

#include <bits/stdc++.h>
#include <ostream>

namespace fluid
{
   struct Fixed 
   {
        int32_t v;

        constexpr Fixed(int v): v(v << 16) {}
        constexpr Fixed(float f): v(f * (1 << 16)) {}
        constexpr Fixed(double f): v(f * (1 << 16)) {}
        constexpr Fixed(): v(0) {}

        static constexpr Fixed from_raw(int32_t x)
        {
            Fixed ret;
            ret.v = x;
            return ret;
        }

        auto operator<=>(const Fixed&) const = default;
        bool operator==(const Fixed&) const = default;
    };

    Fixed operator-(Fixed x);

    Fixed operator+(Fixed a, Fixed b);
    Fixed operator-(Fixed a, Fixed b);
    Fixed operator*(Fixed a, Fixed b);
    Fixed operator/(Fixed a, Fixed b);

    Fixed& operator+=(Fixed &a, Fixed b);
    Fixed& operator-=(Fixed &a, Fixed b);
    Fixed& operator*=(Fixed &a, Fixed b);
    Fixed& operator/=(Fixed &a, Fixed b);

    Fixed abs(Fixed x);

    std::ostream &operator<<(std::ostream &out, Fixed x);
} // namespace fluid

#endif // HEADER_GUARD_FIXED_FIXED_HPP
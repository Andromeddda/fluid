#include "fixed/fixed.hpp"

namespace fluid
{
    Fixed operator+(Fixed a, Fixed b) 
    {
        return Fixed::from_raw(a.v + b.v);
    }

    Fixed operator-(Fixed a, Fixed b) 
    {
        return Fixed::from_raw(a.v - b.v);
    }

    Fixed operator*(Fixed a, Fixed b) 
    {
        return Fixed::from_raw(((int64_t) a.v * b.v) >> 16);
    }

    Fixed operator/(Fixed a, Fixed b) 
    {
        return Fixed::from_raw(((int64_t) a.v << 16) / b.v);
    }

    Fixed& operator+=(Fixed &a, Fixed b) 
    {
        return a = a + b;
    }

    Fixed& operator-=(Fixed &a, Fixed b) 
    {
        return a = a - b;
    }

    Fixed& operator*=(Fixed &a, Fixed b) 
    {
        return a = a * b;
    }

    Fixed& operator/=(Fixed &a, Fixed b) 
    {
        return a = a / b;
    }

    Fixed operator-(Fixed x) 
    {
        return Fixed::from_raw(-x.v);
    }

    Fixed abs(Fixed x) 
    {
        if (x.v < 0) {
            x.v = -x.v;
        }
        return x;
    }

    std::ostream &operator<<(std::ostream &out, Fixed x) 
    {
        return out << x.v / (double) (1 << 16);
    }

} // namespace fluid

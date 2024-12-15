#ifndef HEADER_GUARD_TYPE_PROCESSING_HPP
#define HEADER_GUARD_TYPE_PROCESSING_HPP

#include <utility>
#include <array>
#include "fixed.hpp"

namespace fluid
{
    enum TypeFlag
    {   
        Fixed_F, 
        FastFixed_F, 
        Float_F, 
        Double_F
    };

    struct TypeDescriptor
    {
        TypeFlag flag;
        size_t N, M;

        bool operator==(const TypeDescriptor& other) const = default;
    };

    #define S(N, M)             std::pair<size_t, size_t>{N, M}
    #define FIXED(N, M)         fluid::TypeDescriptor{fluid::TypeFlag::Fixed_F,       N, M}
    #define FAST_FIXED(N, M)    fluid::TypeDescriptor{fluid::TypeFlag::FastFixed_F,   N, M}
    #define FLOAT               fluid::TypeDescriptor{fluid::TypeFlag::Float_F,       0, 0}
    #define DOUBLE              fluid::TypeDescriptor{fluid::TypeFlag::Double_F,      0, 0}

    #ifndef SIZES
        #error "sizes are not defined"    
    #endif

    #ifndef TYPES
        #error "types are not defined"    
    #endif

    constexpr std::array sizes = {SIZES, std::pair<size_t, size_t>{0, 0}};
    constexpr std::array types = {TYPES};

    #undef S
    #undef FIXED
    #undef FAST_FIXED
    #undef FLOAT
    #undef DOUBLE

    template <TypeFlag Flag, size_t N, size_t M>
    struct GetType;

    template <size_t N, size_t M>
    struct GetType<TypeFlag::Fixed_F, N, M>     { using type = Fixed<N, M, false>; };

    template <size_t N, size_t M>
    struct GetType<TypeFlag::FastFixed_F, N, M> { using type = Fixed<N, M, true>; };

    template <size_t N, size_t M>
    struct GetType<TypeFlag::Float_F, N, M>     { using type = float; };

    template <size_t N, size_t M>
    struct GetType<TypeFlag::Double_F, N, M>    { using type = double; };

    constexpr const size_t 
        template_combination_number = types.size() * types.size() * types.size() * sizes.size();

} // namespace fluid




#endif // HEADER_GUARD_TYPE_PROCESSING_HPP
#ifndef HEADER_GUARD_PARTICLE_PARAMS_HPP
#define HEADER_GUARD_PARTICLE_PARAMS_HPP

#include <array>

namespace fluid
{
    template <typename P, typename V, typename VF, size_t... SizeArgs>
    class Simulation;

    template<typename P, typename V>
    struct ParticleParams 
    {
        char type;
        P cur_p;
        std::array<V, deltas.size()> v;

        template <typename VF, size_t... SizeArgs>
        void swap_with(Simulation<P, V, VF, SizeArgs...>& simulation, int x, int y);
    };


    template <typename P, typename V>
    template <typename VF, size_t... SizeArgs>
    void ParticleParams<P, V>::swap_with(Simulation<P, V, VF, SizeArgs...>& simulation, int x, int y) 
    {
        std::swap(simulation.field_[x][y], type);
        std::swap(simulation.p_[x][y], cur_p);
        std::swap(simulation.velocity_.v[x][y], v);
    }

} // namespace fluid

#endif // HEADER_GUARD_PARTICLE_PARAMS_HPP
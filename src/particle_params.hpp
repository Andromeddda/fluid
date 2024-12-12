#ifndef HEADER_GUARD_PARTICLE_PARAMS_HPP
#define HEADER_GUARD_PARTICLE_PARAMS_HPP


namespace fluid
{
    template <typename P, typename V, typename VF, size_t N, size_t M>
    class Simulation;

    template<typename P, typename V>
    struct ParticleParams 
    {
        char type;
        P cur_p;
        std::array<V, deltas.size()> v;

        template <typename VF, size_t N, size_t M>
        void swap_with(Simulation<P, V, VF, N, M>& simulation, int x, int y);
    };


    template <typename P, typename V>
    template <typename VF, size_t N, size_t M>
    void ParticleParams<P, V>::swap_with(Simulation<P, V, VF, N, M>& simulation, int x, int y) 
    {
        std::swap(simulation.field_[x][y], type);
        std::swap(simulation.p_[x][y], cur_p);
        std::swap(simulation.velocity_.v[x][y], v);
    }

} // namespace fluid

#endif // HEADER_GUARD_PARTICLE_PARAMS_HPP
#ifndef HEADER_GUARD_SIMULATION_SIMULATION_HPP
#define HEADER_GUARD_SIMULATION_SIMULATION_HPP

namespace fluid
{
    template <typename P_Type, typename V_Type, typename VF_Type, size_t N, size_t M>
    class Simulation
    {
        char field_[N][M + 1];
    public:
        Simulation(char **field) : field_(field) {}

        void run();
    };

} // namespace fluid

#endif // HEADER_GUARD_SIMULATION_SIMULATION_HPP
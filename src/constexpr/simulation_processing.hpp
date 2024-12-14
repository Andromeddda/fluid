#ifndef HEADER_GUARD_SIMULATION_PROCESSING_HPP
#define HEADER_GUARD_SIMULATION_PROCESSING_HPP

#include <utility>
#include <memory>
#include <array>
#include <functional>
#include "type_processing.hpp"
#include "simulation.hpp"

namespace fluid
{
    struct SimulationDescriptor
    {
        TypeDescriptor p;
        TypeDescriptor v;
        TypeDescriptor vf;
        std::pair<size_t, size_t> size;
    };

    template <SimulationDescriptor SD>
    struct GetSimulationType
    {
        using P  =  typename GetType<SD.p.flag,  SD.p.N,  SD.p.M>::type;
        using V  =  typename GetType<SD.v.flag,  SD.v.N,  SD.v.M>::type;
        using VF =  typename GetType<SD.vf.flag, SD.vf.N, SD.vf.M>::type;

        using type = Simulation<P, V, VF, SD.size.first, SD.size.second>;
    };

    typedef typename std::array<SimulationDescriptor, template_combination_number> SimulationDescriptorArray;

    //
    // Produce array of descriptors of all possible simulations in compile-time
    //
    constexpr SimulationDescriptorArray make_descriptors_constexpr()
    {
        SimulationDescriptorArray result;

        size_t i = 0LU;
        for (auto P_descriptor : types)
            for (auto V_descriptor : types)
                for (auto VF_descriptor : types)
                    for (auto Shape : sizes)
                        result[i++] = SimulationDescriptor(P_descriptor, V_descriptor, VF_descriptor, Shape);
        return result;
    }

    constexpr SimulationDescriptorArray descriptors = make_descriptors_constexpr();

    //
    // Produce array of all possible simulations in compile-time
    //

    typedef typename std::shared_ptr<AbstractSimulation>                    SimulationPtr;
    typedef typename std::function<SimulationPtr()>                         SimulationProcucer;
    typedef typename std::array<SimulationPtr, template_combination_number> ProducerArray;

    ProducerArray producers;

    template <size_t index>
    struct IterateDescriptors
    {
        static IterateDescriptors<index - 1> previous;

        IterateDescriptors() { 
            producers[index - 1] = SimulationProcucer(producer);
        }

        static SimulationPtr producer()  {
            return GetSimulationType<descriptors[index - 1]>::type(); 
        }
    };

} // namespace fluid


#endif // HEADER_GUARD_SIMULATION_PROCESSING_HPP
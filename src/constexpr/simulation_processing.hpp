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
        size_t N;
        size_t M;
    };

    // if N = 0 or M == 0 use Simulation with dynamic allocation
    // otherwise use simulation with static allocation
    template <typename P, typename V, typename VF, size_t N, size_t M>
    struct GetSimulationTypeHelper
    {
        using type = typename std::conditional<N != 0 && M != 0, Simulation<P, V, VF, N, M>, Simulation<P, V, VF>>::type;
    };

    template <SimulationDescriptor SD>
    struct GetSimulationType
    {
        using P  =  typename GetType<SD.p.flag,  SD.p.N,  SD.p.M>::type;
        using V  =  typename GetType<SD.v.flag,  SD.v.N,  SD.v.M>::type;
        using VF =  typename GetType<SD.vf.flag, SD.vf.N, SD.vf.M>::type;

        using type = typename GetSimulationTypeHelper<P, V, VF, SD.N, SD.M>::type;
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
                    for (auto& [N, M] : sizes)
                        result[i++] = SimulationDescriptor(P_descriptor, V_descriptor, VF_descriptor, N, M);
        return result;
    }

    constexpr SimulationDescriptorArray descriptors = make_descriptors_constexpr();

    //
    // Produce array of functions (producers) that return all possible simulations
    //

    typedef typename std::shared_ptr<AbstractSimulation>                         SimulationPtr;
    typedef typename std::function<SimulationPtr()>                              SimulationProducer;
    typedef typename std::array<SimulationProducer, template_combination_number> ProducerArray;

    ProducerArray producers;

    template <size_t index>
    struct IterateDescriptors
    {
        static IterateDescriptors<index - 1> previous;

        IterateDescriptors() 
        { 
            producers[index - 1] = SimulationProducer(IterateDescriptors::producer);
        }

        static SimulationPtr producer()  
        {
            return SimulationPtr(new GetSimulationType<descriptors[index - 1]>::type());
        }
    };

    template <>
    struct IterateDescriptors<0>
    {
        IterateDescriptors() {};
    };

    IterateDescriptors<template_combination_number> constexrp_iterator;

} // namespace fluid


#endif // HEADER_GUARD_SIMULATION_PROCESSING_HPP
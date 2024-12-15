#ifndef HEADER_GUARD_READ_SIMULATION_HPP
#define HEADER_GUARD_READ_SIMULATION_HPP

#include <utility>
#include <array>
#include <ostream>
#include <istream>
#include <iostream>
#include <string>
#include <exception>


#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>


#include "simulation.hpp"
#include "simulation_processing.hpp"


namespace fluid
{
    class Reader
    {
    public:
        Reader(const std::string& filename);
        Reader(const char *filename);

        SimulationPtr get_simulation();
    private:
        std::string filename_;
    };


    Reader::Reader(const std::string& filename) 
        : filename_(filename)
        {}

    Reader::Reader(const char *filename) 
        : filename_(filename)
        {}


    // SimulationPtr Reader::get_simulation(TypeDescriptor p, TypeDescriptor v, TypeDescriptor vf)
    // {
    //     // open file
    //     std::istream is(filename_);

    //     // read field shape
    //     size_t N, M;
    //     is >> N >> M;

    //     SimulationDescriptor sd{p, v, vf, std::make_pair(N, M)};
    // }
} // namespace fluid

#endif // HEADER_GUARD_READ_SIMULATION_HPP
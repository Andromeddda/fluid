#ifndef HEADER_GUARD_READ_SIMULATION_HPP
#define HEADER_GUARD_READ_SIMULATION_HPP

#include <utility>
#include <array>
#include <fstream>
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

        SimulationPtr get_simulation(TypeDescriptor p, TypeDescriptor v, TypeDescriptor vf);
    private:
        std::string filename_;
    };


    Reader::Reader(const std::string& filename) 
        : filename_(filename)
        {}

    Reader::Reader(const char *filename) 
        : filename_(filename)
        {}


    SimulationPtr Reader::get_simulation(TypeDescriptor p, TypeDescriptor v, TypeDescriptor vf)
    {
        SimulationPtr result;

        // open file
        std::ifstream is(filename_);

        // read field shape
        size_t N, M;
        is >> N >> M;

        // construct simulation
        SimulationDescriptor sd{p, v, vf, N, M};
        auto producer = get_producer(sd);
        result = producer(N, M);

        // read ASCII art of field
        std::string line;
        std::getline(is, line);
        for (auto i = 0LU; i < N; i++) 
        {
            std::getline(is, line);

            for (auto j = 0LU; j < M; j++)
                result->set_field(i, j, line[j]);
        }

        return result;
    }
} // namespace fluid

#endif // HEADER_GUARD_READ_SIMULATION_HPP
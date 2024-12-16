#ifndef HEADER_GUARD_PARSE_OPTIONS_HPP
#define HEADER_GUARD_PARSE_OPTIONS_HPP

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
    struct Options
    {
        TypeDescriptor p;
        TypeDescriptor v;
        TypeDescriptor vf;
        std::string filename;
        std::string save_filename;
        size_t n_threads;
    };

    static const char j_option[]  = "-j";
    static const char p_option[]  = "--p-type=";
    static const char v_option[]  = "--v-type=";
    static const char vf_option[] = "--vf-type=";
    static const char save_to[]   = "--save-to=";

    TypeDescriptor parse_type(char* type_str);

    Options parse_options(int argc, char* argv[])
    {
        Options result;
        result.n_threads = 1;

        if (argc < 6)
        {
            std::cout << "Incorrect amount of arguments provided.\n";
            std::cout << "Usage:\n";
            std::cout << "./build/fluid --p-type=... --v-type=... --vf-type=... <filename> --save-to=... [-jN]\n";
            exit(-1);
        }

        for (int i = 1; i < argc; i++)
        {
            if (std::strncmp(argv[i], p_option, 9) == 0)
            {
                result.p = parse_type(argv[i] + 9);
                continue;
            }

            if (std::strncmp(argv[i], v_option, 9) == 0)
            {
                result.v = parse_type(argv[i] + 9);
                continue;
            }

            if (std::strncmp(argv[i], vf_option, 10) == 0)
            {
                result.vf = parse_type(argv[i] + 10);
                continue;
            }

            if (std::strncmp(argv[i], save_to, 10) == 0)
            {
                result.save_filename = std::string(argv[i] + 10);
                continue;
            }

            if (std::strncmp(argv[i], j_option, 2) == 0)
            {
                result.n_threads = std::atoll(argv[i] + 2);
                continue;
            }

            result.filename = std::string(argv[i]);
        }
        return result;
    }

    static const char fixed_[]      = "FIXED";
    static const char fast_fixed_[] = "FAST_FIXED";
    static const char float_[]      = "FLOAT";
    static const char double_[]     = "DOUBLE";

    TypeDescriptor parse_type(char* type_str)
    {
        size_t N = 0, M = 0;

        if (std::strncmp(type_str, fixed_, 5) == 0)
        {
            int res = sscanf(type_str + 5, "(%lu,%lu)", &N, &M);
            if (res != 2) 
            {
                std::cout << "Incorrect option for FIXED type\n";
                exit(-1);
            }

            return TypeDescriptor{TypeFlag::Fixed_F, N, M};
        }

        if (std::strncmp(type_str, fast_fixed_, 10) == 0)
        {
            int res = sscanf(type_str + 10, "(%lu,%lu)", &N, &M);   
            if (res != 2) 
            {
                std::cout << "Incorrect option for FAST_FIXED type\n";
                exit(-1);
            }

            return TypeDescriptor{TypeFlag::FastFixed_F, N, M};
        }

        if (std::strncmp(type_str, float_, 5) == 0)
            return TypeDescriptor{TypeFlag::Float_F, 0, 0};


        if (std::strncmp(type_str, double_, 6) == 0)
            return TypeDescriptor{TypeFlag::Double_F, 0, 0};

        std::cout << "Unknown type provided\n";
        exit(-1);
}
} // namespace fluid

#endif // HEADER_GUARD_PARSE_OPTIONS_HPP
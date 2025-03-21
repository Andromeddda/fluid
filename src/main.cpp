#include <iostream>

#include "fixed.hpp"
#include "vector_field.hpp"
#include "simulation.hpp"
#include "simulation_processing.hpp"
#include "read_simulation.hpp"
#include "parse_options.hpp"

using namespace fluid;
using namespace std;

int main(int argc, char* argv[]) 
{
    Options opts = parse_options(argc, argv);

    Reader r(opts.filename);

    SimulationPtr p = r.get_simulation(opts.p, opts.v, opts.vf, opts.n_threads);

    p->run(cout, opts.save_filename);
}
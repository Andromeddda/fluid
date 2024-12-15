#include <bits/stdc++.h>

#include <iostream>

#include "fixed.hpp"
#include "vector_field.hpp"
#include "simulation.hpp"
#include "simulation_processing.hpp"
#include "read_simulation.hpp"

using namespace fluid;
using namespace std;


int main() 
{
    
    TypeDescriptor td{TypeFlag::Fixed_F, 32, 16};

    Reader r("./data/1.in");

    SimulationPtr p = r.get_simulation(td, td, td);

    p->run(cout);   
    
}
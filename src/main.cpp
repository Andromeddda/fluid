#include <bits/stdc++.h>

#include "fixed.hpp"
#include "vector_field.hpp"
#include "simulation.hpp"
#include "simulation_processing.hpp"

using namespace fluid;
using namespace std;

constexpr size_t N = 14, M = 5;

char field[N][M + 1] = {
    "#####",
    "#.  #",
    "#.# #",
    "#.# #",
    "#.# #",
    "#.# #",    
    "#.# #",
    "#.# #",
    "#...#",
    "#####",
    "#   #",
    "#   #",
    "#   #",
    "#####",
};



// static constexpr Fixed inf = Fixed::from_raw(std::numeric_limits<int32_t>::max());
// static constexpr Fixed eps = Fixed::from_raw(deltas.size());

int main() {
    // Fixed<32, 16> g = 0.1;

    // Simulation<Fixed<32, 16>, Fixed<32, 16>, Fixed<64, 20>, N, M> simulation(field);

    // simulation.set_g(g);
    // simulation.set_rho(' ', 0.01);
    // simulation.set_rho('.', 1000);

    // simulation.run(std::cout);
}
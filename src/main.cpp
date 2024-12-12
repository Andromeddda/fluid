#include <bits/stdc++.h>

#include "fixed/fixed.hpp"
#include "vector_field.hpp"
#include "simulation.hpp"

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

static constexpr Fixed inf = Fixed::from_raw(std::numeric_limits<int32_t>::max());
static constexpr Fixed eps = Fixed::from_raw(deltas.size());

int main() {
    Fixed g = 0.1;

    Simulation<Fixed, Fixed, Fixed, N, M> simulation(field);

    simulation.set_g(g);
    simulation.set_rho(' ', 0.01);
    simulation.set_rho('.', 1000);

    simulation.run(std::cout);
}
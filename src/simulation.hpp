#ifndef HEADER_GUARD_SIMULATION_SIMULATION_HPP
#define HEADER_GUARD_SIMULATION_SIMULATION_HPP

#include <cstdio>
#include <string>
#include <cstring>
#include <ostream>
#include <assert.h>
#include <bits/stdc++.h>

#include "vector_field.hpp"
#include "particle_params.hpp"

namespace fluid
{
    constexpr size_t T = 1'000'000;

    template <typename P, typename V, typename VF, size_t N, size_t M>
    class Simulation
    {
        char field_[N][M + 1];

    public:
        Simulation(char field[][M + 1]);

        void set_g(P g);
        void set_rho(char c, P rho);

        void run(std::ostream& os);

    private:
        friend class ParticleParams<P, V>;

        P   g_;
        P   rho_[256];

        P   p_[N][M]; 
        P   old_p_[N][M];

        VectorField<V, N, M>   velocity_;
        VectorField<VF, N, M>  velocity_flow_;

        int last_use_[N][M];
        int dirs_[N][M];

        int UT = 0;

        std::mt19937 rnd{1337};

        Fixed   move_prob(int x, int y);
        bool    propagate_move(int x, int y, bool is_first);
        void    propagate_stop(int x, int y, bool force = false);
        std::tuple<Fixed, bool, std::pair<int, int>> propagate_flow(int x, int y, Fixed lim);
        Fixed random01();

    }; // class Simulation

    template <typename P, typename V, typename VF, size_t N, size_t M>
    Simulation<P, V, VF, N, M>::Simulation(char field[][M + 1])
    {
        for (size_t i = 0; i < N; i++)
            memcpy(field_[i], field[i], M + 1);
    }

    template <typename P, typename V, typename VF, size_t N, size_t M>
    void Simulation<P, V, VF, N, M>::set_g(P g)
    {
        g_ = g;
    }

    template <typename P, typename V, typename VF, size_t N, size_t M>
    void Simulation<P, V, VF, N, M>::set_rho(char c, P rho)
    {
        rho_[(size_t)c] = rho;
    }


    template <typename P, typename V, typename VF, size_t N, size_t M>
    void Simulation<P, V, VF, N, M>::run(std::ostream& os)
    {
        for (size_t x = 0; x < N; ++x) 
            for (size_t y = 0; y < M; ++y) 
            {
                if (field_[x][y] == '#')
                    continue;
                for (auto [dx, dy] : deltas) 
                    dirs_[x][y] += (field_[x + dx][y + dy] != '#');
            }

        for (size_t i = 0; i < T; ++i) 
        {    
            Fixed total_delta_p = 0;
            // Apply external forces
            for (size_t x = 0; x < N; ++x) {
                for (size_t y = 0; y < M; ++y) {
                    if (field_[x][y] == '#')
                        continue;
                    if (field_[x + 1][y] != '#')
                        velocity_.add(x, y, 1, 0, g_);
                }
            }

            // Apply forces from p_
            memcpy(old_p_, p_, sizeof(p_));
            for (size_t x = 0; x < N; ++x) 
                for (size_t y = 0; y < M; ++y) 
                {
                    if (field_[x][y] == '#')
                        continue;

                    for (auto [dx, dy] : deltas) 
                    {
                        int nx = x + dx, ny = y + dy;
                        if (field_[nx][ny] != '#' && old_p_[nx][ny] < old_p_[x][y]) 
                        {
                            auto delta_p = old_p_[x][y] - old_p_[nx][ny];
                            auto force = delta_p;
                            auto &contr = velocity_.get(nx, ny, -dx, -dy);

                            if (contr * rho_[(int) field_[nx][ny]] >= force) 
                            {
                                contr -= force / rho_[(int) field_[nx][ny]];
                                continue;
                            }

                            force -= contr * rho_[(int) field_[nx][ny]];
                            contr = 0;
                            velocity_.add(x, y, dx, dy, force / rho_[(int) field_[x][y]]);
                            p_[x][y] -= force / dirs_[x][y];
                            total_delta_p -= force / dirs_[x][y];
                        }
                    }
                }

            // Make flow from velocities
            velocity_flow_ = {};
            bool prop = false;
            do 
            {
                UT += 2;
                prop = 0;
                for (size_t x = 0; x < N; ++x) 
                    for (size_t y = 0; y < M; ++y) 
                        if (field_[x][y] != '#' && last_use_[x][y] != UT) 
                        {
                            auto [t, local_prop, _] = propagate_flow(x, y, 1);
                            if (t > 0)
                                prop = 1;
                        }
            } while (prop);

            // Recalculate p_ with kinetic energy
            for (size_t x = 0; x < N; ++x) 
            {
                for (size_t y = 0; y < M; ++y) 
                {
                    if (field_[x][y] == '#')
                        continue;
                    for (auto [dx, dy] : deltas) 
                    {
                        auto old_v = velocity_.get(x, y, dx, dy);
                        auto new_v = velocity_flow_.get(x, y, dx, dy);

                        if (old_v > 0) 
                        {
                            assert(new_v <= old_v);
                            velocity_.get(x, y, dx, dy) = new_v;
                            auto force = (old_v - new_v) * rho_[(int) field_[x][y]];

                            if (field_[x][y] == '.')
                                force *= 0.8;

                            if (field_[x + dx][y + dy] == '#') 
                            {
                                p_[x][y] += force / dirs_[x][y];
                                total_delta_p += force / dirs_[x][y];
                            } 
                            else 
                            {
                                p_[x + dx][y + dy] += force / dirs_[x + dx][y + dy];
                                total_delta_p += force / dirs_[x + dx][y + dy];
                            }
                        }
                    }
                }
            }

            UT += 2;
            prop = false;
            for (size_t x = 0; x < N; ++x) 
            {
                for (size_t y = 0; y < M; ++y) 
                {
                    if (field_[x][y] != '#' && last_use_[x][y] != UT) 
                    {
                        if (random01() < move_prob(x, y)) 
                        {
                            prop = true;
                            propagate_move(x, y, true);
                        } 
                        else 
                        {
                            propagate_stop(x, y, true);
                        }
                    }
                }
            }

            if (prop) 
            {
                os << "Tick " << i << ":\n";
                for (size_t x = 0; x < N; ++x) 
                {
                    os << field_[x] << "\n";
                }
            }
        }
    } // Simulation::run

    template <typename P, typename V, typename VF, size_t N, size_t M>
    Fixed Simulation<P, V, VF, N, M>::random01() 
    {
        return Fixed::from_raw((rnd() & ((1 << 16) - 1)));
    }

    template <typename P, typename V, typename VF, size_t N, size_t M>
    std::tuple<Fixed, bool, std::pair<int, int>> Simulation<P, V, VF, N, M>::propagate_flow(int x, int y, Fixed lim) 
    {
        last_use_[x][y] = UT - 1;
        Fixed ret = 0;
        for (auto [dx, dy] : deltas) 
        {
            int nx = x + dx, ny = y + dy;
            if (field_[nx][ny] != '#' && last_use_[nx][ny] < UT) 
            {
                auto cap = velocity_.get(x, y, dx, dy);
                auto flow = velocity_flow_.get(x, y, dx, dy);
                if (flow == cap)
                    continue;

                auto vp = std::min(lim, cap - flow);
                if (last_use_[nx][ny] == UT - 1) 
                {
                    velocity_flow_.add(x, y, dx, dy, vp);
                    last_use_[x][y] = UT;

                    return {vp, 1, {nx, ny}};
                }
                auto [t, prop, end] = propagate_flow(nx, ny, vp);
                ret += t;
                if (prop) 
                {
                    velocity_flow_.add(x, y, dx, dy, t);
                    last_use_[x][y] = UT;

                    return {t, prop && end != std::pair(x, y), end};
                }
            }
        }
        last_use_[x][y] = UT;
        return {ret, 0, {0, 0}};
    } // Simulation:propagate_flow

    template <typename P, typename V, typename VF, size_t N, size_t M>
    bool Simulation<P, V, VF, N, M>::propagate_move(int x, int y, bool is_first) 
    {
        last_use_[x][y] = UT - is_first;
        bool ret = false;
        int nx = -1, ny = -1;
        do 
        {
            std::array<Fixed, deltas.size()> tres;
            Fixed sum = 0;
            for (size_t i = 0; i < deltas.size(); ++i) 
            {
                auto [dx, dy] = deltas[i];
                int nx = x + dx, ny = y + dy;

                if (field_[nx][ny] == '#' || last_use_[nx][ny] == UT) 
                {
                    tres[i] = sum;
                    continue;
                }
                auto v = velocity_.get(x, y, dx, dy);
                if (v < 0) 
                {
                    tres[i] = sum;
                    continue;
                }
                sum += v;
                tres[i] = sum;
            }

            if (sum == 0)
                break;

            Fixed p_ = random01() * sum;
            size_t d = std::ranges::upper_bound(tres, p_) - tres.begin();

            auto [dx, dy] = deltas[d];
            nx = x + dx;
            ny = y + dy;
            assert(velocity_.get(x, y, dx, dy) > 0 && field_[nx][ny] != '#' && last_use_[nx][ny] < UT);

            ret = (last_use_[nx][ny] == UT - 1 || propagate_move(nx, ny, false));
        } while (!ret);

        last_use_[x][y] = UT;

        for (size_t i = 0; i < deltas.size(); ++i) 
        {
            auto [dx, dy] = deltas[i];
            int nx = x + dx, ny = y + dy;
            if (field_[nx][ny] != '#' && last_use_[nx][ny] < UT - 1 && velocity_.get(x, y, dx, dy) < 0)
                propagate_stop(nx, ny);
        }

        if (ret)
            if (!is_first) 
            {
                ParticleParams<P, V> pp{};
                pp.swap_with(*this, x, y);
                pp.swap_with(*this, nx, ny);
                pp.swap_with(*this, x, y);
            }

        return ret;
    } //  Simulation::propagate_move


    template <typename P, typename V, typename VF, size_t N, size_t M>
    void Simulation<P, V, VF, N, M>::propagate_stop(int x, int y, bool force) 
    {
        if (!force) {
            bool stop = true;
            for (auto [dx, dy] : deltas) {
                int nx = x + dx, ny = y + dy;
                if (field_[nx][ny] != '#' && last_use_[nx][ny] < UT - 1 && velocity_.get(x, y, dx, dy) > 0) {
                    stop = false;
                    break;
                }
            }
            if (!stop) {
                return;
            }
        }
        last_use_[x][y] = UT;
        for (auto [dx, dy] : deltas) {
            int nx = x + dx, ny = y + dy;
            if (field_[nx][ny] == '#' || last_use_[nx][ny] == UT || velocity_.get(x, y, dx, dy) > 0) {
                continue;
            }
            propagate_stop(nx, ny);
        }
    } // Simulation::propagate_stop

    template <typename P, typename V, typename VF, size_t N, size_t M>
    Fixed Simulation<P, V, VF, N, M>::move_prob(int x, int y) 
    {
        Fixed sum = 0;
        for (size_t i = 0; i < deltas.size(); ++i) {
            auto [dx, dy] = deltas[i];
            int nx = x + dx, ny = y + dy;
            if (field_[nx][ny] == '#' || last_use_[nx][ny] == UT) {
                continue;
            }
            auto v = velocity_.get(x, y, dx, dy);
            if (v < 0) {
                continue;
            }
            sum += v;
        }
        return sum;
    } // Simulation::move_prob

} // namespace fluid

#endif // HEADER_GUARD_SIMULATION_SIMULATION_HPP
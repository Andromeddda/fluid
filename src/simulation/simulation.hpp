#ifndef HEADER_GUARD_SIMULATION_SIMULATION_HPP
#define HEADER_GUARD_SIMULATION_SIMULATION_HPP

#include <cstdio>
#include <string>
#include <cstring>
#include <ostream>
#include <assert.h>
#include <memory>
#include <ranges>
#include <iterator>
#include <algorithm>
#include <fstream>
#include <optional>

#include <thread>
#include <atomic>

#include "vector_field.hpp"
#include "particle_params.hpp"
#include "type_processing.hpp"
#include "matrix.hpp"
#include "random.hpp"

namespace fluid
{
    constexpr size_t T = 1'000'000;

    class AbstractSimulation
    {
    public:
        AbstractSimulation() {};
        virtual ~AbstractSimulation() {};

        virtual void run(std::ostream& os, const std::string& save_to) = 0;

        virtual void set_field(size_t i, size_t j, char c) = 0;
    };

    // SizeArgs... can either be 0 or 2 parameters
    // Otherwise MatrixType will throw compile error

    // If no parameters passed as SizeArgs, choose DynamicMatrix and construct all fields in runtime
    // Otherwise, choose StaticMatrix and construct all fields in compile time
    //      (in this case passing N and M to constructor will only check matching with template)

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    class Simulation : public AbstractSimulation
    {
    public:
        Simulation();
        Simulation(size_t n, size_t m);

        Simulation& operator= (const Simulation& other) = default;

        void run(std::ostream& os, const std::string& save_to);

        void set_field(size_t i, size_t j, char c) override;
    private:
        friend class ParticleParams<P, V>;

        size_t N, M;
        MatrixType<char, SizeArgs...>::type field_;

        P   g_;
        P   rho_[256];

        MatrixType<P, SizeArgs...>::type   p_; 
        MatrixType<P, SizeArgs...>::type   old_p_;

        MatrixType<int, SizeArgs...>::type last_use_;
        MatrixType<int, SizeArgs...>::type dirs_;

        VectorField<V, SizeArgs...>   velocity_;
        VectorField<VF, SizeArgs...>  velocity_flow_;

        int UT = 0;
        std::atomic<bool> save_{false};

        void    tick(std::ostream& os, size_t tick_n);
        void    cycle(std::ostream& os, size_t& tick_n);

        void apply_gravity();
        void apply_forces_from_p();
        void make_flow_from_velocities();
        void recalculate_p();
        bool process_particles();

        void print_state(std::ostream& os, size_t tick_n);

        V       move_prob(int x, int y);
        bool    propagate_move(int x, int y, bool is_first);
        void    propagate_stop(int x, int y, bool force = false);
        std::tuple<V, std::optional<std::pair<int, int>>> propagate_flow(int x, int y, V lim);


        void    save(const std::string& save_to);


    }; // class Simulation


    template <typename P, typename V, typename VF, size_t... SizeArgs>
    Simulation<P, V, VF, SizeArgs...>::Simulation()
    : 
        N(GetSizes<SizeArgs...>::n), M(GetSizes<SizeArgs...>::m),
        field_(N, M), p_(N, M), old_p_(N, M), last_use_(N, M), dirs_(N, M), velocity_(N, M), velocity_flow_(N, M)
    {
        g_ = 0.1;
        rho_[' '] = 0.01;
        rho_['.'] = 1000; 
    }

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    Simulation<P, V, VF, SizeArgs...>::Simulation(size_t n, size_t m)
    : N(n), M(m), field_(n, m), p_(n, m), old_p_(n, m), last_use_(n, m), dirs_(n, m), velocity_(n, m), velocity_flow_(n, m)
    {
        assert(SizesMatch<SizeArgs...>(n, m));
        g_ = 0.1;
        rho_[' '] = 0.01;
        rho_['.'] = 1000; 
    }


    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::set_field(size_t i, size_t j, char c) 
    {
        field_[i][j] = c;
    }


    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::apply_gravity()
    {
        for (size_t x = 0; x < N; ++x) 
        {
            for (size_t y = 0; y < M; ++y) 
            {
                if (field_[x][y] == '#')
                    continue;
                if (field_[x + 1][y] != '#')
                    velocity_.add(x, y, 1, 0, g_);
            }
        }
    }

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::apply_forces_from_p()
    {
        old_p_ = p_;
        for (size_t x = 0; x < N; ++x) 
        {
            for (size_t y = 0; y < M; ++y) 
            {
                if (field_[x][y] == '#')
                    continue;

                for (auto [dx, dy] : deltas) 
                {
                    int nx = x + dx, ny = y + dy;

                    if (field_[nx][ny] == '#' )
                        continue; // skip walls

                    if (old_p_[nx][ny] >= old_p_[x][y])
                        continue;

                    auto force = old_p_[x][y] - old_p_[nx][ny];
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
                }
            }
        }
    }

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::make_flow_from_velocities()
    {
        velocity_flow_.v.reset();
        bool prop = false;
        do 
        {
            UT += 2;
            prop = false;
            for (size_t x = 0; x < N; ++x)
            {
                for (size_t y = 0; y < M; ++y) 
                {
                    if (field_[x][y] == '#')
                        continue; // skip walls

                    if (last_use_[x][y] == UT)
                        continue; // skip particles that are already processed

                    auto [t, coord] = propagate_flow(x, y, 1);
                    if (t > 0)
                        prop = true;
                }
            }
        } while (prop);
    }

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::recalculate_p()
    {
        // Recalculate p_ with kinetic energy
        for (size_t x = 0; x < N; ++x) 
        {
            for (size_t y = 0; y < M; ++y) 
            {
                if (field_[x][y] == '#')
                    continue;
                for (auto [dx, dy] : deltas) 
                {
                    V old_v = velocity_.get(x, y, dx, dy);
                    V new_v = velocity_flow_.get(x, y, dx, dy);

                    if (old_v <= 0)
                        continue;

                    if (new_v > old_v) 
                    {
                        std::cout << "SIMULATION ERROR: " << new_v << " > " << old_v << '\n';
                        assert(new_v <= old_v);
                    }

                    velocity_.get(x, y, dx, dy) = new_v;
                    V force = (old_v - new_v) * rho_[(int) field_[x][y]];

                    if (field_[x][y] == '.')
                        force *= 0.8;

                    if (field_[x + dx][y + dy] == '#') 
                        p_[x][y] += force / dirs_[x][y];
                    else 
                        p_[x + dx][y + dy] += force / dirs_[x + dx][y + dy];
                }
            }
        }
    }

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    bool Simulation<P, V, VF, SizeArgs...>::process_particles()
    {
        bool prop = false;
        for (size_t x = 0; x < N; ++x) 
        {
            for (size_t y = 0; y < M; ++y) 
            {
                if (field_[x][y] != '#' && last_use_[x][y] != UT) 
                {
                    if (random01<V>() < move_prob(x, y)) 
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
        return prop;
    }

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::print_state(std::ostream& os, size_t tick_n)
    {
        os << "Tick " << tick_n << ":\n";
        for (size_t x = 0; x < N; ++x) 
        {
            for (size_t y = 0; y < M; ++y)
                os << field_[x][y];
            os << std::endl;
        }
    }

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::tick(std::ostream& os, size_t tick_n)
    {
        apply_gravity();
        apply_forces_from_p();
        make_flow_from_velocities();
        recalculate_p();

        UT += 2;
        
        bool prop = process_particles();
        if (prop) 
            print_state(os, tick_n);
    }

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::cycle(std::ostream& os, size_t& tick_n)
    {
        for (; tick_n < T; ++tick_n)
        {
            tick(os, tick_n);
            std::this_thread::yield();
            if (save_.load())
                return;
        }
    }

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::save(const std::string& save_to)
    {
        std::ofstream of;

        // open to write
        of.open(save_to);

        assert(of.is_open());

        of << N << " " << M << "\n";

        for (size_t x = 0; x < N; ++x) 
        {
            for (size_t y = 0; y < M; ++y)
                of.put(field_[x][y]);
            of.put('\n');
        }

        of.close();
    }

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::run(std::ostream& os, const std::string& save_to)
    {
        // initialize dirs
        for (size_t x = 0; x < N; ++x) 
        {
            for (size_t y = 0; y < M; ++y) 
            {
                if (field_[x][y] == '#')
                    continue;
                for (auto [dx, dy] : deltas) 
                    dirs_[x][y] += (field_[x + dx][y + dy] != '#');
            }
        }

        size_t i = 0;

        // run the cycle in another thread
        std::thread thrd(&Simulation<P, V, VF, SizeArgs...>::cycle, this, std::ref(os), std::ref(i));

        // expect '\n'
        int c;
        while ((c = getchar() != 10)) {}

        save_.store(true);
        thrd.join();

        std::cout << "saving to file " << save_to << '\n';
        save(save_to);
    } // Simulation::run


    template <typename P, typename V, typename VF, size_t... SizeArgs>
    std::tuple<V, std::optional<std::pair<int, int>>> Simulation<P, V, VF, SizeArgs...>::propagate_flow(int x, int y, V lim) 
    {
        last_use_[x][y] = UT - 1;
        VF ret = 0;

        // look at every nearby particle
        for (auto [dx, dy] : deltas) 
        {
            int nx = x + dx, ny = y + dy;

            if (field_[nx][ny] == '#')
                continue; // skip walls

            if (last_use_[nx][ny] == UT)
                continue;// skip particles that are already processed

            V cap = velocity_.get(x, y, dx, dy);
            VF flow = velocity_flow_.get(x, y, dx, dy);

            if (flow == cap)
                continue;

            VF vp = std::min(lim, cap - flow);
            if (last_use_[nx][ny] == UT - 1) 
            {
                velocity_flow_.add(x, y, dx, dy, vp);
                last_use_[x][y] = UT;

                return {vp, std::optional<std::pair<int, int>>{std::make_pair(nx, ny)}};
            }

            auto [t, end] = propagate_flow(nx, ny, vp);
            ret += t;
            if (end.has_value()) 
            {
                velocity_flow_.add(x, y, dx, dy, t);
                last_use_[x][y] = UT;

                if (end.value() == std::pair(x, y))
                    return {t, std::optional<std::pair<int, int>>()};

                return {t, end};
            }
        }

        last_use_[x][y] = UT;
        return {ret, std::optional<std::pair<int, int>>()};
    } // Simulation:propagate_flow

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    bool Simulation<P, V, VF, SizeArgs...>::propagate_move(int x, int y, bool is_first) 
    {
        last_use_[x][y] = UT - is_first;
        bool ret = false;
        int nx = -1, ny = -1;
        do 
        {
            std::array<P, deltas.size()> tres;
            P sum = 0;
            for (size_t i = 0; i < deltas.size(); ++i) 
            {
                auto [dx, dy] = deltas[i];
                int nx = x + dx, ny = y + dy;

                if (field_[nx][ny] == '#' || last_use_[nx][ny] == UT) 
                {
                    tres[i] = sum;
                    continue;
                }
                P v = velocity_.get(x, y, dx, dy);
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

            P p = random01<P>() * sum;
            size_t d = std::ranges::upper_bound(tres, p) - tres.begin();

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


    template <typename P, typename V, typename VF, size_t... SizeArgs>
    void Simulation<P, V, VF, SizeArgs...>::propagate_stop(int x, int y, bool force) 
    {
        if (!force) 
        {
            bool stop = true;

            // look at every nearby cell
            for (auto [dx, dy] : deltas) 
            {
                int nx = x + dx, ny = y + dy;

                if (field_[nx][ny] == '#')
                    continue; // skip walls

                if (last_use_[nx][ny] == UT)
                    continue; // skip particles that are already processed

                if (velocity_.get(x, y, dx, dy) <= 0)
                    continue; // skip particles that are not in our way

                stop = false;
                break;
            }

            if (!stop) 
                return;
        }

        // update the time of last use of current particle
        last_use_[x][y] = UT;

        // look at every nearby cell
        for (auto [dx, dy] : deltas) 
        {
            int nx = x + dx, ny = y + dy;

            if (field_[nx][ny] == '#')
                continue; // skip walls

            if (last_use_[nx][ny] == UT)
                continue; // skip particles that are already processed

            if (velocity_.get(x, y, dx, dy) > 0)
                continue; // skip particles that are in our way

            propagate_stop(nx, ny); // stop the particle
        }
    } // Simulation::propagate_stop

    template <typename P, typename V, typename VF, size_t... SizeArgs>
    V Simulation<P, V, VF, SizeArgs...>::move_prob(int x, int y) 
    {
        V sum = 0;
        for (auto [dx, dy] : deltas) 
        {
            int nx = x + dx, ny = y + dy;
            if (field_[nx][ny] == '#' || last_use_[nx][ny] == UT)
                continue;

            auto v = velocity_.get(x, y, dx, dy);
            if (v < 0)
                continue;

            sum += v;
        }
        return sum;
    } // Simulation::move_prob

} // namespace fluid

#endif // HEADER_GUARD_SIMULATION_SIMULATION_HPP
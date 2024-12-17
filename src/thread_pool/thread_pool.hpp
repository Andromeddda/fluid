#ifndef HEADER_GUARD_THREAD_POOL_HPP
#define HEADER_GUARD_THREAD_POOL_HPP

#include <functional>
#include <utility>
#include <queue>
#include <unordered_set>
#include <concepts>

// concurrency
#include <mutex>
#include <future>
#include <thread>
#include <condition_variable>

namespace fluid
{
    class ThreadPool
    {
    public:
        ThreadPool();
        ThreadPool(size_t num_threads);

        template <typename Func, typename ...Args>
        size_t add_task(const Func& func, Args&&... args);

        void init(size_t num_threads);

        void wait(size_t task_id);

        void wait_all();

        bool is_completed(size_t task_id);

        void shutdown();

        ~ThreadPool();
    private:
        void run(); 

        std::queue<std::pair<std::future<void>, size_t>> task_queue;
        std::mutex                  queue_mutex;
        std::condition_variable     queue_condvar;

        std::unordered_set<size_t>  completed_tasks;
        std::mutex                  completed_tasks_mutex;
        std::condition_variable     completed_tasks_condvar;

        std::vector<std::thread>    threads;

        std::atomic_bool            shut{false};

        std::atomic<size_t>         next_id{0};
    };

    template <typename Func, typename ...Args>
    size_t ThreadPool::add_task(const Func& func, Args&&... args)
    {
        size_t task_id = next_id++;

        std::lock_guard<std::mutex> queue_guard(queue_mutex);

        task_queue.emplace(std::make_pair(std::async(std::launch::deferred, func, args...), task_id));

        queue_condvar.notify_one(); // wake one waiting worker

        return task_id;
    }

} // namespace fluid



#endif // HEADER_GUARD_THREAD_POOL_HPP
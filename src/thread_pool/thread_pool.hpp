#ifndef HEADER_GUARD_THREAD_POOL_HPP
#define HEADER_GUARD_THREAD_POOL_HPP

/*
    origin : https://habr.com/ru/articles/656515/
*/

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <deque>
#include <thread>
#include <unordered_set>
#include <vector>
#include <future>

namespace fluid
{

    struct Task 
    {
        size_t id;
        std::future<void> func;
    };

    class ThreadPool
    {
    public:
        ThreadPool();

        ThreadPool(size_t threads_count);

        ~ThreadPool();

        void init(size_t n_threads = std::thread::hardware_concurrency());

        void shutdown();

        template<typename Func, typename... Args>
        size_t add_task(const Func& f, Args&&... args);

        template<typename Func, typename... Args>
        size_t force_task(const Func& f, Args&&... args);

        void wait(size_t id);

        void wait_all();

    private:
        void run();

        std::atomic<bool>   shut = false;
        std::atomic<size_t> free_threads;

        std::vector<std::thread> threads;


        std::deque<Task>            task_queue;
        std::mutex                  task_queue_mutex;
        std::condition_variable     task_queue_condvar;


        std::unordered_set<size_t>  completed_tasks;
        std::mutex                  completed_tasks_mutex;
        std::condition_variable     completed_tasks_condvar;

        std::atomic<size_t>         next_id = 0;

    };

    template<typename Func, typename... Args>
    size_t ThreadPool::add_task(const Func& f, Args&&... args) 
    {
        std::lock_guard<std::mutex> tasks_queue_lock(task_queue_mutex);
        size_t id = next_id++;

        task_queue.push_back(Task{id, std::async(std::launch::deferred, f, args...)});

        task_queue_condvar.notify_one();
        return id;
    }

    template<typename Func, typename... Args>
    size_t ThreadPool::force_task(const Func& f, Args&&... args) 
    {
        std::lock_guard<std::mutex> tasks_queue_lock(task_queue_mutex);
        size_t id = next_id++;

        if (free_threads > 0) 
        {
            task_queue.push_front(Task{id, std::async(std::launch::deferred, f, args...)});
            task_queue_condvar.notify_one();
            return id;
        }

        f(args...);

        std::lock_guard<std::mutex> completed_tasks_lock(completed_tasks_mutex);
        completed_tasks.insert(id);
        completed_tasks_condvar.notify_one();

        return id;
    }

} // namespace fluid



#endif // HEADER_GUARD_THREAD_POOL_HPP
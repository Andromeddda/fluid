#include "thread_pool.hpp"

namespace fluid
{
    ThreadPool::ThreadPool() {}

    ThreadPool::ThreadPool(size_t n_threads) 
    {
        init(n_threads);
    }

    void ThreadPool::init(size_t n_threads) 
    {
        for (size_t i = 0; i < n_threads; ++i)
            threads.emplace_back(&ThreadPool::run, this);
        free_threads = n_threads;
    }


    ThreadPool::~ThreadPool() 
    {
        shutdown();
        for (auto& thread : threads) {
            thread.join();
        }
    }

    void ThreadPool::shutdown() 
    {
        shut = true;
        task_queue_condvar.notify_all();
    }

    void ThreadPool::wait(size_t id) 
    {
        std::unique_lock<std::mutex> completed_tasks_lock(completed_tasks_mutex);

        if (completed_tasks.contains(id))
            return;

        auto wait_condition = [this, id]() -> bool {
            return completed_tasks.contains(id);
        };

        completed_tasks_condvar.wait(completed_tasks_lock, wait_condition);
    }

    void ThreadPool::wait_all() 
    {
        std::unique_lock<std::mutex> completed_tasks_lock(completed_tasks_mutex);

        {
            std::lock_guard<std::mutex> tasks_queue_lock(task_queue_mutex);
            if (completed_tasks.size() == next_id)
                return;
        }

        auto wait_condition = [this]() -> bool {
            std::lock_guard<std::mutex> tasks_queue_lock(task_queue_mutex);
            return completed_tasks.size() == next_id;
        };

        completed_tasks_condvar.wait(completed_tasks_lock, wait_condition);
    }

    void ThreadPool::run() 
    {
        while (!shut) 
        {
            std::unique_lock<std::mutex> tasks_queue_lock(task_queue_mutex);

            auto wait_condition = [this]() -> bool {
                return shut || !task_queue.empty();
            };

            task_queue_condvar.wait(tasks_queue_lock, wait_condition);

            --free_threads;
            if (shut) break;

            auto task = std::move(task_queue.front());
            task_queue.pop();
            tasks_queue_lock.unlock();

            task.func.get();

            std::lock_guard<std::mutex> completed_tasks_lock(completed_tasks_mutex);
            completed_tasks.insert(task.id);

            ++free_threads;

            completed_tasks_condvar.notify_one();
        }
    }

} // namespace fluid
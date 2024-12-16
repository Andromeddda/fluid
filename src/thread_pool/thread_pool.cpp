#include "thread_pool/thread_pool.hpp"

namespace fluid
{

    ThreadPool::ThreadPool() { }


    ThreadPool::ThreadPool(size_t num_threads)
    {
        init(num_threads);
    }

    void ThreadPool::init(size_t num_threads)
    {
        for (auto i = 0LU; i < num_threads; i++)
            threads.push_back(std::thread(&ThreadPool::run, this));
    }

    void ThreadPool::run()
    {
        while (!shut.load())
        {
            std::unique_lock<std::mutex> lock(queue_mutex);

            auto wait_predicate = [this]() -> bool {return !task_queue.empty() || shut; };

            // wait until thread pool shuts down or until a new task appears
            // acquire queue_mutex on wake
            queue_condvar.wait(lock, wait_predicate);

            if (!task_queue.empty())
            {
                // get new task
                auto task = std::move(task_queue.front());
                task_queue.pop();
                lock.unlock();

                // start the task (future::get)
                task.first.get();

                std::lock_guard<std::mutex> completed_tasks_guard(completed_tasks_mutex);

                completed_tasks.insert(task.second);

                completed_tasks_condvar.notify_all();
            }
        }
    }

    void ThreadPool::wait(size_t task_id)
    {
        std::unique_lock<std::mutex> lock(completed_tasks_mutex);

        auto wait_predicate = 
            [this, task_id]() -> bool 
            { 
                return completed_tasks.find(task_id) != completed_tasks.end(); 
            };

        completed_tasks_condvar.wait(lock, wait_predicate);
    }

    void ThreadPool::wait_all()
    {
        std::unique_lock<std::mutex> lock(queue_mutex);

        auto wait_predicate = 
            [this]() -> bool 
            { 
                return task_queue.empty() && next_id == completed_tasks.size(); 
            };

        completed_tasks_condvar.wait(lock, wait_predicate);

        completed_tasks.clear();
        next_id.store(0);
    }

    bool ThreadPool::is_completed(size_t task_id)
    {
        std::lock_guard<std::mutex> completed_tasks_guard(completed_tasks_mutex);

        return completed_tasks.find(task_id) != completed_tasks.end(); 
    }

    void ThreadPool::shutdown()
    {
        shut.store(true);

        for (size_t i = 0; i < threads.size(); ++i) 
        {
            queue_condvar.notify_all();
            threads[i].join();
        }
    }

    ThreadPool::~ThreadPool()
    {
        shutdown();
    }

} // namespace fluid
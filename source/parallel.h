#pragma once

#include <vector>
#include <thread>
#include <functional>
#include <atomic>

namespace NavMesh {

// Simple parallel for-loop implementation
// Divides work among available hardware threads
class ParallelExecutor {
public:
    ParallelExecutor() : num_threads_(std::max(1u, std::thread::hardware_concurrency())) {}

    explicit ParallelExecutor(unsigned int num_threads)
        : num_threads_(std::max(1u, num_threads)) {}

    unsigned int NumThreads() const { return num_threads_; }

    // Parallel for loop: executes func(i) for i in [start, end)
    // Only parallelizes if work is large enough to justify thread overhead
    template<typename Func>
    void ParallelFor(size_t start, size_t end, Func&& func) {
        if (end <= start) return;

        size_t total = end - start;

        // Don't parallelize small workloads (thread overhead > benefit)
        const size_t min_work_per_thread = 100;
        if (total < min_work_per_thread * 2 || num_threads_ == 1) {
            for (size_t i = start; i < end; ++i) {
                func(i);
            }
            return;
        }

        // Determine actual threads to use
        unsigned int threads_to_use = std::min(
            num_threads_,
            static_cast<unsigned int>((total + min_work_per_thread - 1) / min_work_per_thread)
        );

        std::vector<std::thread> threads;
        threads.reserve(threads_to_use - 1);

        size_t chunk_size = (total + threads_to_use - 1) / threads_to_use;

        // Launch worker threads
        for (unsigned int t = 1; t < threads_to_use; ++t) {
            size_t chunk_start = start + t * chunk_size;
            size_t chunk_end = std::min(chunk_start + chunk_size, end);

            if (chunk_start < end) {
                threads.emplace_back([chunk_start, chunk_end, &func]() {
                    for (size_t i = chunk_start; i < chunk_end; ++i) {
                        func(i);
                    }
                });
            }
        }

        // Main thread handles first chunk
        size_t main_end = std::min(start + chunk_size, end);
        for (size_t i = start; i < main_end; ++i) {
            func(i);
        }

        // Wait for all threads
        for (auto& thread : threads) {
            thread.join();
        }
    }

    // Parallel for with thread-local storage
    // init_func() creates thread-local state
    // work_func(i, local_state) processes item i
    // merge_func(local_state) merges results (called serially)
    template<typename T, typename InitFunc, typename WorkFunc, typename MergeFunc>
    void ParallelForWithLocal(size_t start, size_t end,
                               InitFunc&& init_func,
                               WorkFunc&& work_func,
                               MergeFunc&& merge_func) {
        if (end <= start) return;

        size_t total = end - start;
        const size_t min_work_per_thread = 100;

        if (total < min_work_per_thread * 2 || num_threads_ == 1) {
            T local = init_func();
            for (size_t i = start; i < end; ++i) {
                work_func(i, local);
            }
            merge_func(local);
            return;
        }

        unsigned int threads_to_use = std::min(
            num_threads_,
            static_cast<unsigned int>((total + min_work_per_thread - 1) / min_work_per_thread)
        );

        std::vector<T> thread_locals(threads_to_use);
        std::vector<std::thread> threads;
        threads.reserve(threads_to_use - 1);

        size_t chunk_size = (total + threads_to_use - 1) / threads_to_use;

        // Initialize all thread-local storage
        for (unsigned int t = 0; t < threads_to_use; ++t) {
            thread_locals[t] = init_func();
        }

        // Launch worker threads
        for (unsigned int t = 1; t < threads_to_use; ++t) {
            size_t chunk_start = start + t * chunk_size;
            size_t chunk_end = std::min(chunk_start + chunk_size, end);

            if (chunk_start < end) {
                threads.emplace_back([chunk_start, chunk_end, &work_func, &thread_locals, t]() {
                    for (size_t i = chunk_start; i < chunk_end; ++i) {
                        work_func(i, thread_locals[t]);
                    }
                });
            }
        }

        // Main thread handles first chunk
        size_t main_end = std::min(start + chunk_size, end);
        for (size_t i = start; i < main_end; ++i) {
            work_func(i, thread_locals[0]);
        }

        // Wait for all threads
        for (auto& thread : threads) {
            thread.join();
        }

        // Merge results serially
        for (unsigned int t = 0; t < threads_to_use; ++t) {
            merge_func(thread_locals[t]);
        }
    }

private:
    unsigned int num_threads_;
};

// Global executor instance (lazy initialization)
inline ParallelExecutor& GetParallelExecutor() {
    static ParallelExecutor executor;
    return executor;
}

} // namespace NavMesh

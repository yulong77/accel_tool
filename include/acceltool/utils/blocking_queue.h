#pragma once

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <utility>

namespace acceltool
{
    template <typename T>
    class BlockingQueue
    {
    public:
        explicit BlockingQueue(std::size_t capacity)
            : m_capacity(capacity)
        {
            if (m_capacity == 0)
            {
                throw std::invalid_argument("BlockingQueue capacity must be > 0");
            }
        }

        bool push(T value)
        {
            std::unique_lock<std::mutex> lock(m_mutex);

            m_notFull.wait(lock, [this] {
                return m_closed || (m_queue.size() < m_capacity);
            });

            if (m_closed)
            {
                return false;
            }

            m_queue.push(std::move(value));

            if (m_queue.size() > m_peakSize)
            {
                m_peakSize = m_queue.size();
            }

            lock.unlock();
            m_notEmpty.notify_one();
            return true;
        }

        bool waitPop(T& out)
        {
            std::unique_lock<std::mutex> lock(m_mutex);

            m_notEmpty.wait(lock, [this] {
                return m_closed || !m_queue.empty();
            });

            if (m_queue.empty())
            {
                return false;
            }

            out = std::move(m_queue.front());
            m_queue.pop();

            lock.unlock();
            m_notFull.notify_one();
            return true;
        }

        void close()
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_closed = true;
            m_notEmpty.notify_all();
            m_notFull.notify_all();
        }

        bool closed() const
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_closed;
        }

        std::size_t size() const
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_queue.size();
        }

        std::size_t peakSize() const
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_peakSize;
        }

        std::size_t capacity() const noexcept
        {
            return m_capacity;
        }

    private:
        const std::size_t m_capacity;

        mutable std::mutex m_mutex;
        std::condition_variable m_notEmpty;
        std::condition_variable m_notFull;

        std::queue<T> m_queue;
        bool m_closed = false;
        std::size_t m_peakSize = 0;
    };
}

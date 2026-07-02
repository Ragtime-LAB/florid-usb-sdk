#ifndef HYSERIAL_STATS_HPP
#define HYSERIAL_STATS_HPP

#include <atomic>
#include <cstdint>

namespace HySerial
{
    struct Stasis
    {
        Stasis() = default;

        std::atomic<uint64_t> messages_sent{0};
        std::atomic<uint64_t> messages_received{0};
        std::atomic<uint64_t> bytes_sent{0};
        std::atomic<uint64_t> bytes_received{0};

        // record send/receive events
        void record_send(uint64_t bytes)
        {
            messages_sent.fetch_add(1, std::memory_order_relaxed);
            bytes_sent.fetch_add(bytes, std::memory_order_relaxed);
        }

        void record_receive(uint64_t bytes)
        {
            messages_received.fetch_add(1, std::memory_order_relaxed);
            bytes_received.fetch_add(bytes, std::memory_order_relaxed);
        }
    };
}

#endif //HYSERIAL_STATS_HPP

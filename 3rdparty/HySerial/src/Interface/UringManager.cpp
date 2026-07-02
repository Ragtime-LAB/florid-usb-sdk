#include <cerrno>
#include <cstring>
#include <format>
#include <HySerial/Interface/UringManager.hpp>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace HySerial
{
    // helper to convert between id and user_data pointer-sized storage
    static inline void set_user_data(io_uring_sqe* sqe, uint64_t id)
    {
        io_uring_sqe_set_data(sqe, reinterpret_cast<void*>(static_cast<uintptr_t>(id)));
    }

    static inline uint64_t get_user_data_id(const io_uring_cqe* cqe)
    {
        return static_cast<uint64_t>(reinterpret_cast<uintptr_t>(io_uring_cqe_get_data(cqe)));
    }

    tl::expected<std::unique_ptr<UringManager>, Error> UringManager::create(const unsigned int queue_depth)
    {
        struct enabler : UringManager
        {
        };
        auto manager = std::make_unique<enabler>();
        auto res = manager->initialize(queue_depth);
        if (res)
        {
            // initialize stasis if enabled
#ifdef HS_ENABLE_STASIS
            manager->m_stasis = std::make_unique<Stasis>();
#endif
            return manager;
        }
        return tl::make_unexpected(res.error());
    }

    UringManager::UringManager() = default;

    UringManager::~UringManager()
    {
        if (m_is_running.load())
        {
            stop();
        }
        io_uring_queue_exit(&m_ring);
    }

    void UringManager::register_read_callback(ReadCallback cb)
    {
        auto new_cb = std::make_unique<ReadCallback>(std::move(cb));
        ReadCallback* new_ptr = new_cb.get();
        auto old_storage = std::move(m_read_cb_storage);
        m_read_cb_storage = std::move(new_cb);
        m_read_cb_ptr.store(new_ptr, std::memory_order_release);
    }

    void UringManager::register_write_callback(WriteCallback cb)
    {
        auto new_cb = std::make_unique<WriteCallback>(std::move(cb));
        WriteCallback* new_ptr = new_cb.get();
        auto old_storage = std::move(m_write_cb_storage);
        m_write_cb_storage = std::move(new_cb);
        m_write_cb_ptr.store(new_ptr, std::memory_order_release);
    }

    void UringManager::register_error_callback(ErrorCallback cb)
    {
        auto new_cb = std::make_unique<ErrorCallback>(std::move(cb));
        ErrorCallback* new_ptr = new_cb.get();
        auto old_storage = std::move(m_error_cb_storage);
        m_error_cb_storage = std::move(new_cb);
        m_error_cb_ptr.store(new_ptr, std::memory_order_release);
    }

    void UringManager::start_read_for_fd(int fd, size_t buf_size)
    {
        {
            std::lock_guard lock(m_uring_lock);
            m_fd = fd;
            if (!m_read_in_flight.load(std::memory_order_acquire))
            {
                m_read_buffer.assign(buf_size, std::byte{0});
            }
        }
        m_continue_read.store(true, std::memory_order_relaxed);
        m_read_pending_rearm.store(false, std::memory_order_relaxed);
        submit_read();
    }

    void UringManager::stop_read_for_fd()
    {
        m_continue_read.store(false, std::memory_order_relaxed);
        m_read_pending_rearm.store(false, std::memory_order_relaxed);
    }

    void UringManager::submit_read()
    {
        if (m_fd < 0)
        {
            std::cerr << "[FATAL] UringManager: submit_read called with invalid fd\n";
            return;
        }

        // Single critical section: protect active_requests and submission with m_uring_lock
        std::lock_guard lock(m_uring_lock);
        if (m_read_in_flight.load(std::memory_order_acquire))
        {
            return;
        }

        // allocate id
        uint64_t id = m_next_request_id.fetch_add(1);

        io_uring_sqe* sqe = io_uring_get_sqe(&m_ring);
        if (!sqe)
        {
            m_read_pending_rearm.store(true, std::memory_order_release);
            return;
        }

        RequestRecord rec;
        rec.id = id; // Phase 2: Store id for arena validation
        rec.is_write = false;
        rec.fd = m_fd;
        m_requests.insert(id, rec);
        m_read_in_flight.store(true, std::memory_order_release);
        m_read_pending_rearm.store(false, std::memory_order_release);

        io_uring_prep_read(sqe, m_fd, m_read_buffer.data(), m_read_buffer.size(), -1);
        set_user_data(sqe, id);

        if (const int ret = io_uring_submit(&m_ring); ret < 0)
        {
            // cleanup
            m_requests.erase(id);
            m_read_in_flight.store(false, std::memory_order_release);
            m_read_pending_rearm.store(true, std::memory_order_release);
            auto* ecb_ptr = m_error_cb_ptr.load(std::memory_order_acquire);
            if (ecb_ptr && *ecb_ptr)
            {
                (*ecb_ptr)(ret);
            }
            return;
        }
    }

    void UringManager::submit_send(std::span<const std::byte> buffer)
    {
        if (m_fd < 0)
        {
            return;
        }

        // Phase 2: Use BufferPool for zero-copy writes
        auto buf = m_buffer_pool.acquire(buffer.size());
        buf->insert(buf->end(), buffer.begin(), buffer.end());

        // allocate id
        uint64_t id = m_next_request_id.fetch_add(1);

        // single critical section protecting both active_requests and submission
        m_uring_lock.lock();
        io_uring_sqe* sqe = io_uring_get_sqe(&m_ring);
        if (!sqe)
        {
            m_uring_lock.unlock();
            m_buffer_pool.release(buf);
            return;
        }

        RequestRecord rec;
        rec.id = id; // Phase 2: Store id for arena validation
        rec.is_write = true;
        rec.fd = m_fd;
        rec.buf = buf;
        rec.offset = 0;
        m_requests.insert(id, rec);

        // prepare initial write for the whole buffer
        io_uring_prep_write(sqe, m_fd, buf->data(), buf->size(), -1);
        set_user_data(sqe, id);

        if (const int ret = io_uring_submit(&m_ring); ret < 0)
        {
            // cleanup
            m_requests.erase(id);
            m_buffer_pool.release(buf);
            m_uring_lock.unlock();
            throw std::runtime_error(std::string("io_uring_submit failed with ") + std::to_string(ret));
        }

        m_uring_lock.unlock();
    }

    void UringManager::run()
    {
        m_is_running.store(true);

        while (m_is_running.load())
        {
            struct Completion
            {
                uint64_t id{0};
                int res{0};
                RequestRecord record{};
                bool found{false};
            };

            std::vector<Completion> completions;

            bool need_rearm_read = false;

            {
                std::lock_guard lock(m_uring_lock);
                if (const int ret = io_uring_submit(&m_ring); ret < 0)
                {
                    auto* ecb_ptr = m_error_cb_ptr.load(std::memory_order_acquire);
                    if (ecb_ptr && *ecb_ptr)
                    {
                        (*ecb_ptr)(ret);
                    }
                }
            }

            io_uring_cqe* first_cqe = nullptr;
            if (const int ret = io_uring_wait_cqe(&m_ring, &first_cqe); ret < 0)
            {
                auto* ecb_ptr = m_error_cb_ptr.load(std::memory_order_acquire);
                if (ecb_ptr && *ecb_ptr)
                {
                    (*ecb_ptr)(ret);
                }
                continue;
            }

            {
                std::lock_guard lock(m_uring_lock);
                io_uring_cqe* cqe;
                unsigned head;
                unsigned cqe_count = 0;

                io_uring_for_each_cqe(&m_ring, head, cqe)
                {
                    cqe_count++;

                    Completion completion;
                    completion.id = get_user_data_id(cqe);
                    completion.res = cqe->res;

                    if (completion.id != 0)
                    {
                        if (RequestRecord* rec_ptr = m_requests.find(completion.id))
                        {
                            completion.record = *rec_ptr;
                            completion.found = true;
                        }
                    }

                    completions.push_back(std::move(completion));
                }

                if (cqe_count > 0)
                {
                    io_uring_cq_advance(&m_ring, cqe_count);
                }
            }

            for (const auto& completion : completions)
            {
                if (completion.id == 0 || !completion.found)
                {
                    continue;
                }

                const uint64_t id = completion.id;
                RequestRecord record = completion.record;

                bool is_write = record.is_write;
                int res = completion.res;
                if (!is_write)
                {
                    // read completion: report bytes read and mark rearm if needed
                    if (res < 0)
                    {
                        // Phase 1: lock-free callback access
                        auto* ecb_ptr = m_error_cb_ptr.load(std::memory_order_acquire);
                        if (ecb_ptr && *ecb_ptr)
                        {
                            (*ecb_ptr)(res);
                        }
                        else
                        {
                            std::cerr << "[FATAL] UringManager: Read error res=" << res << ' ' << std::strerror(-res) << '\n';
                        }
                        
                        // cleanup record
                        m_uring_lock.lock();
                        m_requests.erase(id);
                        m_read_in_flight.store(false, std::memory_order_release);
                        m_uring_lock.unlock();
                        
                        // Restart read on ALL errors if continuous reading is enabled
                        // This ensures the read loop continues even after device errors
                        if (m_continue_read.load(std::memory_order_relaxed))
                        {
                            need_rearm_read = true;
                        }
                        
                        continue;
                    }

                    // Phase 1: lock-free callback access
                    auto* rcb_ptr = m_read_cb_ptr.load(std::memory_order_acquire);
                    if (rcb_ptr && *rcb_ptr)
                    {
                        // create a span referencing the internal read buffer of length 'res'
                        std::span<const std::byte> data(reinterpret_cast<const std::byte*>(m_read_buffer.data()),
                                                        static_cast<size_t>(res));
                        (*rcb_ptr)(data);
                    }

#ifdef HS_ENABLE_STASIS
                    if (m_stasis) m_stasis->record_receive(static_cast<uint64_t>(res));
#endif

                    // remove record and request rearm after CQ processing
                    m_uring_lock.lock();
                    m_requests.erase(id);
                    m_read_in_flight.store(false, std::memory_order_release);
                    m_uring_lock.unlock();
                    if (m_continue_read.load(std::memory_order_relaxed))
                    {
                        need_rearm_read = true;
                    }
                    continue;
                }

                // write path
                if (res == -EINTR)
                {
                    // retry same offset; prepare submit without holding m_uring_lock
                    m_uring_lock.lock();
                    io_uring_sqe* sqe_retry = io_uring_get_sqe(&m_ring);
                    if (!sqe_retry)
                    {
                        m_uring_lock.unlock();
                        auto* ecb_ptr = m_error_cb_ptr.load(std::memory_order_acquire);
                        if (ecb_ptr && *ecb_ptr) (*ecb_ptr)(-EINTR);
                        // cleanup record
                        m_uring_lock.lock();
                        m_requests.erase(id);
                        m_uring_lock.unlock();
                        continue;
                    }
                    size_t remaining = record.buf->size() - record.offset;
                    io_uring_prep_write(sqe_retry, record.fd, record.buf->data() + record.offset, remaining, -1);
                    set_user_data(sqe_retry, id);
                    if (const int ret = io_uring_submit(&m_ring); ret < 0)
                    {
                        m_uring_lock.unlock();
                        auto* ecb_ptr = m_error_cb_ptr.load(std::memory_order_acquire);
                        if (ecb_ptr && *ecb_ptr) (*ecb_ptr)(ret);
                        m_uring_lock.lock();
                        m_requests.erase(id);
                        m_uring_lock.unlock();
                    }
                    else
                    {
                        m_uring_lock.unlock();
                    }
                    continue;
                }

                if (res < 0)
                {
                    auto* ecb_ptr = m_error_cb_ptr.load(std::memory_order_acquire);
                    if (ecb_ptr && *ecb_ptr)
                    {
                        (*ecb_ptr)(res);
                    }
                    else
                    {
                        std::cerr << "[FATAL] UringManager: Write error res=" << res << ' ' << std::strerror(-res) << '\n';
                    }
                    
                    // cleanup record and release buffer
                    m_uring_lock.lock();
                    m_requests.erase(id);
                    if (record.buf) m_buffer_pool.release(record.buf);
                    m_uring_lock.unlock();
                    
                    continue;
                }

                // success: update offset and check completion
                size_t new_offset = record.offset + static_cast<size_t>(res);
                if (new_offset < record.buf->size())
                {
                    // partial write, resubmit remaining without holding m_uring_lock
                    m_uring_lock.lock();
                    io_uring_sqe* sqe_next = io_uring_get_sqe(&m_ring);
                    if (!sqe_next)
                    {
                        m_uring_lock.unlock();
                        auto* ecb_ptr = m_error_cb_ptr.load(std::memory_order_acquire);
                        if (ecb_ptr && *ecb_ptr) (*ecb_ptr)(-EAGAIN);
                        m_uring_lock.lock();
                        m_requests.erase(id);
                        if (record.buf) m_buffer_pool.release(record.buf);
                        m_uring_lock.unlock();
                        continue;
                    }
                    size_t remaining = record.buf->size() - new_offset;
                    io_uring_prep_write(sqe_next, record.fd, record.buf->data() + new_offset, remaining, -1);
                    set_user_data(sqe_next, id);
                    if (const int ret = io_uring_submit(&m_ring); ret < 0)
                    {
                        m_uring_lock.unlock();
                        auto* ecb_ptr = m_error_cb_ptr.load(std::memory_order_acquire);
                        if (ecb_ptr && *ecb_ptr) (*ecb_ptr)(ret);
                        m_uring_lock.lock();
                        m_requests.erase(id);
                        if (record.buf) m_buffer_pool.release(record.buf);
                        m_uring_lock.unlock();
                    }
                    else
                    {
                        m_uring_lock.unlock();
                        // update stored offset now that resubmit succeeded
                        m_uring_lock.lock();
                        RequestRecord* rec_ptr2 = m_requests.find(id);
                        if (rec_ptr2 != nullptr)
                        {
                            rec_ptr2->offset = new_offset;
                        }
                        m_uring_lock.unlock();
                    }
                    continue;
                }

                // fully written - call write callback and erase record
                auto* wcb_ptr = m_write_cb_ptr.load(std::memory_order_acquire);
                if (wcb_ptr && *wcb_ptr)
                {
                    (*wcb_ptr)(static_cast<IoResult>(new_offset));
                }

#ifdef HS_ENABLE_STASIS
                if (m_stasis) m_stasis->record_send(static_cast<uint64_t>(new_offset));
#endif

                // Phase 2: Release buffer back to pool and cleanup record
                if (record.is_write && record.buf)
                {
                    m_buffer_pool.release(record.buf);
                }
                m_uring_lock.lock();
                m_requests.erase(id);
                m_uring_lock.unlock();
            }

            // re-arm read if any completion requested it
            if (need_rearm_read || (m_continue_read.load(std::memory_order_relaxed) &&
                                    m_read_pending_rearm.load(std::memory_order_acquire)))
            {
                submit_read();
            }
        }
    }

    void UringManager::stop()
    {
        m_is_running.store(false);
        m_uring_lock.lock();
        if (io_uring_sqe* sqe = io_uring_get_sqe(&m_ring))
        {
            io_uring_prep_nop(sqe);
            set_user_data(sqe, 0);
            io_uring_submit(&m_ring);
        }
        m_uring_lock.unlock();
    }

    tl::expected<void, Error> UringManager::initialize(const unsigned int queue_depth)
    {
        if (io_uring_queue_init(queue_depth, &m_ring, 0) < 0)
        {
            return tl::make_unexpected(Error{
                ErrorCode::UringInitError, std::string("UringManager init failed: ") + strerror(errno)
            });
        }

        (void)queue_depth;

        // Phase 2: Initialize BufferPool for zero-copy writes
        // Pool size is 2x queue_depth to account for concurrent writes
        m_buffer_pool = BufferPool(static_cast<size_t>(queue_depth) * 2, 8192);

        return {};
    }


    void UringManager::submit_request(io_uring_sqe* sqe, CompletionCallback callback)
    {
        uint64_t id = m_next_request_id.fetch_add(1);

        // insert record first
        RequestRecord rec;
        rec.id = id;
        rec.cb = std::move(callback);
        rec.is_write = false;
        rec.fd = m_fd;
        m_uring_lock.lock();
        m_requests.insert(id, rec);

        // submit SQE
        set_user_data(sqe, id);

        if (const int ret = io_uring_submit(&m_ring); ret < 0)
        {
            m_requests.erase(id);
            m_uring_lock.unlock();
            throw std::runtime_error(std::string("io_uring_submit failed with ") + std::to_string(ret));
        }
        m_uring_lock.unlock();
    }

    void UringManager::bind_fd(int fd)
    {
        m_fd = fd;
    }
}

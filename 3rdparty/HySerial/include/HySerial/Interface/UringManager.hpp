#ifndef HYSERIAL_URINGMANAGER_HPP
#define HYSERIAL_URINGMANAGER_HPP

#include <liburing.h>
#include <functional>
#include <memory>
#include <span>
#include <atomic>
#include <unordered_map>
#include <vector>

#include <tl/expected.hpp>
#include <HySerial/Util/Error.hpp>

#include <HySerial/Util/SpinLock.hpp>

#ifdef HS_ENABLE_STASIS
#include <HySerial/Stats.hpp>
#endif


namespace HySerial
{
    // 使用ssize_t来表示I/O操作的结果，与read/write的返回值保持一致
    using IoResult = ssize_t;

    // 定义两种回调的类型签名
    // ReadCallback now receives the received data as a span so callers can examine payload
    using ReadCallback = std::function<void(std::span<const std::byte> data)>;
    using WriteCallback = std::function<void(IoResult bytes_written)>;
    using ErrorCallback = std::function<void(IoResult error_code)>;

    class UringManager
    {
    public:
        // 工厂函数，推荐的创建方式
        static tl::expected<std::unique_ptr<UringManager>, Error> create(unsigned int queue_depth = 256);

        // 析构函数负责清理io_uring
        ~UringManager();

        // 禁止拷贝和移动，因为它管理着一个唯一的内核资源
        UringManager(const UringManager&) = delete;
        UringManager& operator=(const UringManager&) = delete;

        /**
         * @brief 提交一个异步写请求。函数会将传入的 buffer 复制到内部堆内存，
         * 以保证在 I/O 完成前数据有效性。
         * @param buffer 要发送的数据（会被复制）
         */
        void submit_send(std::span<const std::byte> buffer);

        /**
         * @brief 启动事件循环。此函数将阻塞，直到 stop() 被调用。
         */
        void run();

        /**
         * @brief 停止事件循环。可以从另一个线程调用此函数。
         */
        void stop();

        /**
         * @brief 注册或修改默认的读取回调
         */
        void register_read_callback(ReadCallback cb);

        /**
         * @brief 注册或修改默认的写入回调
         */
        void register_write_callback(WriteCallback cb);

        void register_error_callback(ErrorCallback cb);

        /**
         * @brief 将本 manager 绑定到单个 fd 并为其启用连续读取
         * @param fd 文件描述符
         * @param buf_size 每次读取的缓冲区大小
         */
        void start_read_for_fd(int fd, size_t buf_size = 4096);

        /**
         * @brief 停止对已绑定 fd 的连续读取
         */
        void stop_read_for_fd();

        /**
         * @brief 将管理器绑定到 fd（仅设置 fd，用于 write 或延迟启动 read）
         */
        void bind_fd(int fd);

        // 通用的完成回调类型，它接收整个 CQE 作为参数
        using CompletionCallback = std::function<void(const io_uring_cqe*)>;

    private:
        // RequestRecord must be defined before RequestRegistry.
        struct RequestRecord
        {
            CompletionCallback cb;
            std::shared_ptr<std::vector<std::byte>> buf;
            uint64_t id{0}; // Phase 2: For arena collision detection
            size_t offset{0};
            int fd{-1};
            bool is_write{false};
        };

        class RequestRegistry
        {
        public:
            void insert(uint64_t id, const RequestRecord& rec)
            {
                m_records[id] = rec;
            }

            RequestRecord* find(uint64_t id)
            {
                auto it = m_records.find(id);
                return it == m_records.end() ? nullptr : &it->second;
            }

            void erase(uint64_t id)
            {
                m_records.erase(id);
            }

            void clear()
            {
                m_records.clear();
            }

        private:
            std::unordered_map<uint64_t, RequestRecord> m_records;
        };

        // Phase 2: BufferPool - Zero-copy buffer reuse for writes
        class BufferPool
        {
        public:
            explicit BufferPool(size_t pool_size = 0, size_t buffer_size = 8192)
                : m_pool_size(pool_size),
                  m_buffer_size(buffer_size),
                  m_buffers(pool_size),
                  m_available(pool_size)
            {
                for (size_t i = 0; i < pool_size; ++i)
                {
                    m_buffers[i] = std::make_shared<std::vector<std::byte>>(buffer_size);
                    m_available[i].store(true, std::memory_order_relaxed);
                }
            }

            std::shared_ptr<std::vector<std::byte>> acquire(size_t needed_size)
            {
                if (m_pool_size == 0)
                {
                    return std::make_shared<std::vector<std::byte>>(needed_size);
                }

                for (size_t i = 0; i < m_pool_size; ++i)
                {
                    bool expected = true;
                    if (m_available[i].compare_exchange_strong(
                        expected, false, std::memory_order_acquire, std::memory_order_relaxed))
                    {
                        auto& buf = m_buffers[i];
                        if (buf->capacity() < needed_size)
                        {
                            buf->reserve(needed_size);
                        }
                        buf->clear();
                        return buf;
                    }
                }

                return std::make_shared<std::vector<std::byte>>(needed_size);
            }

            void release(const std::shared_ptr<std::vector<std::byte>>& buf)
            {
                if (m_pool_size == 0) return;

                for (size_t i = 0; i < m_pool_size; ++i)
                {
                    if (m_buffers[i] == buf)
                    {
                        buf->clear();
                        m_available[i].store(true, std::memory_order_release);
                        return;
                    }
                }
            }

        private:
            size_t m_pool_size;
            size_t m_buffer_size{};
            std::vector<std::shared_ptr<std::vector<std::byte>>> m_buffers;
            std::vector<std::atomic<bool>> m_available;
        };

        // 私有构造函数，强制使用 create() 工厂
        UringManager();

        // 初始化 io_uring 实例
        tl::expected<void, Error> initialize(unsigned int queue_depth);

        // 提交一个请求并注册其回调
        void submit_request(struct io_uring_sqe* sqe, CompletionCallback callback);

        // 提交异步读请求（不对外公开）
        void submit_read();

        RequestRegistry m_requests;

        // Phase 2: Buffer pool for zero-copy writes
        BufferPool m_buffer_pool;

        io_uring m_ring{};
        std::atomic<bool> m_is_running{false};

        std::atomic<uint64_t> m_next_request_id{1};


        // single-fd helpers
        int m_fd{-1};
        std::vector<std::byte> m_read_buffer;
        std::atomic<bool> m_continue_read{false};
        std::atomic<bool> m_read_in_flight{false};
        std::atomic<bool> m_read_pending_rearm{false};

        // Phase 1 optimization: Single unified lock for io_uring and active_requests
        SpinLock m_uring_lock;

        // Phase 1 optimization: Atomic callback pointers for lock-free access
        std::unique_ptr<ReadCallback> m_read_cb_storage;
        std::unique_ptr<WriteCallback> m_write_cb_storage;
        std::unique_ptr<ErrorCallback> m_error_cb_storage;

        std::atomic<ReadCallback*> m_read_cb_ptr{nullptr};
        std::atomic<WriteCallback*> m_write_cb_ptr{nullptr};
        std::atomic<ErrorCallback*> m_error_cb_ptr{nullptr};

#ifdef HS_ENABLE_STASIS
        // optional stats collector
        std::unique_ptr<Stasis> m_stasis;
#endif
    };
}

#endif //HYSERIAL_URINGMANAGER_HPP

#ifndef HYSERIAL_BUILDER_HPP
#define HYSERIAL_BUILDER_HPP
#include <string_view>

#include <HySerial/Interface/Config.hpp>
#include <HySerial/Interface/Interface.hpp>
#include <HySerial/Interface/UringManager.hpp>

#include "HySerial/Socket/Socket.hpp"

namespace HySerial
{
    class Builder
    {
    public:
        Builder() = default;

        Builder& device(std::string_view p)
        {
            cfg.device_path = std::string(p);
            return *this;
        }

        Builder& baud_rate(uint32_t b)
        {
            cfg.baud_rate = b;
            return *this;
        }

        Builder& data_bits(DataBits db)
        {
            cfg.data_bits = db;
            return *this;
        }

        Builder& parity(Parity p)
        {
            cfg.parity = p;
            return *this;
        }

        Builder& stop_bits(StopBits s)
        {
            cfg.stop_bits = s;
            return *this;
        }

        Builder& flow_control(FlowControl f)
        {
            cfg.flow_control = f;
            return *this;
        }

        Builder& rts_dtr_on(bool on)
        {
            cfg.rts_dtr_on = on;
            return *this;
        }

        Builder& on_read(ReadCallback cb)
        {
            read_cb = std::move(cb);
            return *this;
        }

        Builder& on_write(WriteCallback cb)
        {
            write_cb = std::move(cb);
            return *this;
        }

        Builder& on_error(ErrorCallback cb)
        {
            error_cb = std::move(cb);
            return *this;
        }

        tl::expected<std::unique_ptr<Serial>, Error> build() const
        {
            if (cfg.device_path.empty())
            {
                return tl::make_unexpected(Error{ErrorCode::SocketCreateError, "device path empty"});
            }

            // 2) Create Socket with full config
            auto sock = std::make_unique<Socket>(cfg);
            if (auto res = sock->ensure_connected(); !res)
            {
                return tl::make_unexpected(res.error());
            }

            // 3) Create UringManager
            auto mgr_res = UringManager::create(/*queue_depth=*/256);
            if (!mgr_res)
            {
                return tl::make_unexpected(mgr_res.error());
            }
            auto mgr = std::move(mgr_res.value());

            // 4) register callbacks if provided
            if (read_cb) mgr->register_read_callback(read_cb);
            if (write_cb) mgr->register_write_callback(write_cb);
            if (error_cb) mgr->register_error_callback(error_cb);

            // 5) Let Serial::create handle start/read and thread
            return Serial::create(cfg, std::move(sock), std::move(mgr));
        };

    private:
        SerialConfig cfg;
        ReadCallback read_cb{};
        WriteCallback write_cb{};
        ErrorCallback error_cb{};
    };
}

#endif //HYSERIAL_BUILDER_HPP

#include <HySerial/Socket/Socket.hpp>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <format>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <unistd.h>

#ifndef TCIFLUSH
#define TCIFLUSH 0
#endif
#ifndef TCOFLUSH
#define TCOFLUSH 1
#endif
#ifndef TCIOFLUSH
#define TCIOFLUSH 2
#endif

using tl::unexpected, std::format;

namespace {
constexpr int kStaleRetryLimit = 3;
constexpr int kStaleRetryDelayUs = 50000;
constexpr int kResetPulseDelayUs = 1000;

bool is_fd_in_stale_state(int fd) {
  int pending = 0;
  if (ioctl(fd, TIOCINQ, &pending) == -1 && errno == EIO) {
    return true;
  }
  if (ioctl(fd, TIOCOUTQ, &pending) == -1 && errno == EIO) {
    return true;
  }
  return false;
}

void reset_stale_fd(int fd) {
  ioctl(fd, TCFLSH, TCIOFLUSH);

  int status = 0;
  if (ioctl(fd, TIOCMGET, &status) == -1) {
    return;
  }

  int lowered = status & ~(TIOCM_RTS | TIOCM_DTR);
  ioctl(fd, TIOCMSET, &lowered);
  usleep(kResetPulseDelayUs);

  int raised = status | TIOCM_RTS | TIOCM_DTR;
  ioctl(fd, TIOCMSET, &raised);
}
} // namespace

namespace HySerial {

Socket::Socket(const SerialConfig &cfg) : config(cfg) {}

Socket::~Socket() {
  if (sock_fd > 0) {
    close(sock_fd);
    sock_fd = -1;
  }
}

tl::expected<void, Error> Socket::ensure_connected() noexcept {
  if (sock_fd > 0) {
    close(sock_fd);
    sock_fd = -1;
  }

  const std::string dev_path = config.device_path;

  int attempt = 0;
  for (; attempt < kStaleRetryLimit; ++attempt) {
    sock_fd = open(dev_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (sock_fd == -1) {
      return unexpected(Error{ErrorCode::SocketCreateError,
                              format("Failed to open serial device '{}': {}",
                                     dev_path, strerror(errno))});
    }

    if (!is_fd_in_stale_state(sock_fd)) {
      break;
    }

    reset_stale_fd(sock_fd);
    if (!is_fd_in_stale_state(sock_fd)) {
      break;
    }

    close(sock_fd);
    sock_fd = -1;
    usleep(kStaleRetryDelayUs);
  }

  if (sock_fd < 0) {
    return unexpected(Error{
        ErrorCode::SocketCreateError,
        format("Serial device '{}' stuck in stale state after {} attempts",
               dev_path, kStaleRetryLimit)});
  }

  if (ioctl(sock_fd, TIOCEXCL) == -1 && errno == EBUSY) {
    close(sock_fd);
    sock_fd = -1;
    return unexpected(
        Error{ErrorCode::SocketCreateError,
              format("Serial device '{}' is already in use", dev_path)});
  }

  struct termios2 tio{};
  if (ioctl(sock_fd, TCGETS2, &tio) == -1) {
    close(sock_fd);
    sock_fd = -1;
    return unexpected(Error{ErrorCode::SocketBindError,
                            format("Failed to get attributes for '{}': {}",
                                   dev_path, strerror(errno))});
  }

  // Set baud rate using termios2 BOTHER (supports arbitrary baud rates)
  tio.c_cflag &= ~CBAUD;
  if (config.baud_rate == 0) {
    tio.c_cflag |= B0;
  } else {
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = config.baud_rate;
    tio.c_ospeed = config.baud_rate;
  }

  // Data bits
  tio.c_cflag &= ~CSIZE;
  switch (config.data_bits) {
  case DataBits::BITS_5:
    tio.c_cflag |= CS5;
    break;
  case DataBits::BITS_6:
    tio.c_cflag |= CS6;
    break;
  case DataBits::BITS_7:
    tio.c_cflag |= CS7;
    break;
  case DataBits::BITS_8:
    tio.c_cflag |= CS8;
    break;
  default:
    tio.c_cflag |= CS8;
    break;
  }

  // Parity
  if (config.parity == Parity::NONE) {
    tio.c_cflag &= ~PARENB;
  } else {
    tio.c_cflag |= PARENB;
    if (config.parity == Parity::ODD) {
      tio.c_cflag |= PARODD;
    } else {
      tio.c_cflag &= ~PARODD;
    }
  }

  // Stop bits
  if (config.stop_bits == StopBits::TWO) {
    tio.c_cflag |= CSTOPB;
  } else {
    tio.c_cflag &= ~CSTOPB;
  }

  // Flow control
  if (config.flow_control == FlowControl::RTS_CTS) {
    tio.c_cflag |= CRTSCTS;
  } else {
    tio.c_cflag &= ~CRTSCTS;
  }

  // Input flags - disable special handling
  tio.c_iflag &=
      ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  // Output flags - disable post processing
  tio.c_oflag &= ~OPOST;

  // Local flags - raw mode
  tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  // Control characters - block until at least 1 byte is available
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 0;

  // Apply attributes
  if (ioctl(sock_fd, TCSETS2, &tio) == -1) {
    close(sock_fd);
    sock_fd = -1;
    return unexpected(Error{ErrorCode::SocketBindError,
                            format("Failed to set attributes for '{}': {}",
                                   dev_path, strerror(errno))});
  }

  // Handle RTS/DTR if requested (platform dependent)
  if (config.rts_dtr_on) {
    int status;
    if (ioctl(sock_fd, TIOCMGET, &status) == -1) {
      // non-fatal; ignore
    } else {
      status |= TIOCM_RTS | TIOCM_DTR;
      if (ioctl(sock_fd, TIOCMSET, &status) == -1) {
        // non-fatal; ignore
      }
    }
  }

  // Flush and ensure blocking (clear O_NONBLOCK)
  ioctl(sock_fd, TCFLSH, TCIOFLUSH);
  const int flags = fcntl(sock_fd, F_GETFL, 0);
  fcntl(sock_fd, F_SETFL, flags & ~O_NONBLOCK);

  return {};
}

tl::expected<void, Error> Socket::validate_connection() noexcept {
  if (sock_fd <= 0) {
    return ensure_connected();
  }
  return {};
}

tl::expected<void, Error> Socket::flush() const noexcept {
  if (sock_fd < 0) {
    return unexpected(Error{ErrorCode::InvalidSocketError,
                            "Cannot flush with invalid socket descriptor"});
  }

  if (ioctl(sock_fd, TCFLSH, TCIOFLUSH) == -1) {
    return unexpected(
        Error{ErrorCode::SocketFlushError,
              format("Failed to flush serial device: {}", strerror(errno))});
  }

  return {};
}
} // namespace HySerial

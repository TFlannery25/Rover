// serial_port.h — minimal non-blocking Linux serial port (termios).
// Knows nothing about frames or ROS: it opens a port and moves raw bytes.
#pragma once
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace rover {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Returns true on success; on failure, lastError() says why.
    bool open(const std::string& path, int baud) {
        close();
        speed_t speed = baudConstant(baud);
        if (speed == 0) { err_ = "unsupported baud rate " + std::to_string(baud); return false; }

        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) { err_ = path + ": " + std::strerror(errno); return false; }

        termios tio{};
        if (tcgetattr(fd_, &tio) != 0) { err_ = std::string("tcgetattr: ") + std::strerror(errno); close(); return false; }
        cfmakeraw(&tio);                    // 8 data bits, no parity, no input/output processing
        tio.c_cflag |= (CLOCAL | CREAD);    // ignore modem lines, enable receiver
        tio.c_cflag &= ~CSTOPB;             // 1 stop bit
        tio.c_cflag &= ~CRTSCTS;            // no hardware flow control
        cfsetispeed(&tio, speed);
        cfsetospeed(&tio, speed);
        tio.c_cc[VMIN]  = 0;                // read() never blocks; returns 0 if nothing is waiting
        tio.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &tio) != 0) { err_ = std::string("tcsetattr: ") + std::strerror(errno); close(); return false; }
        tcflush(fd_, TCIOFLUSH);            // drop stale bytes from before we opened
        return true;
    }

    void close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    bool isOpen() const { return fd_ >= 0; }

    // Returns bytes read (0 if nothing available), or -1 on a real error (e.g. unplugged).
    ssize_t read(uint8_t* buf, size_t n) {
        if (fd_ < 0) return -1;
        ssize_t r = ::read(fd_, buf, n);
        if (r >= 0) return r;
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
        err_ = std::strerror(errno);
        return -1;
    }

    // For the command path later (Pi -> STM32). Returns bytes written or -1.
    ssize_t write(const uint8_t* buf, size_t n) {
        if (fd_ < 0) return -1;
        ssize_t r = ::write(fd_, buf, n);
        if (r >= 0) return r;
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
        err_ = std::strerror(errno);
        return -1;
    }

    const std::string& lastError() const { return err_; }

private:
    static speed_t baudConstant(int baud) {
        switch (baud) {
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 921600: return B921600;
            default:     return 0;
        }
    }
    int fd_ = -1;
    std::string err_;
};

}  // namespace rover
#include "gimbal_serial/serial_port.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

namespace gimbal_serial {

namespace {
speed_t baudToSpeed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B115200;
    }
}
}  // namespace

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& device, int baud) {
    close();
    device_ = device;
    baud_ = baud;
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "Open serial failed: " << device << " err=" << std::strerror(errno) << "\n";
        return false;
    }
    if (!setBaud(baud)) {
        close();
        return false;
    }
    return true;
}

bool SerialPort::reopen() {
    if (device_.empty() || baud_ <= 0) {
        return false;
    }
    return open(device_, baud_);
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
    fd_ = -1;
}

bool SerialPort::isOpen() const {
    return fd_ >= 0;
}

int SerialPort::read(uint8_t* buffer, int max_len, int timeout_ms) {
    if (fd_ < 0 || !buffer || max_len <= 0) {
        return -1;
    }
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd_, &read_set);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd_ + 1, &read_set, nullptr, nullptr, &tv);
    if (ret <= 0) {
        return 0;
    }
    int n = static_cast<int>(::read(fd_, buffer, static_cast<size_t>(max_len)));
    if (n < 0 && errno != EAGAIN) {
        std::cerr << "Serial read error: " << std::strerror(errno) << "\n";
        return -1;
    }
    return n > 0 ? n : 0;
}

int SerialPort::write(const uint8_t* data, int len) {
    if (fd_ < 0 || !data || len <= 0) {
        return -1;
    }
    int total = 0;
    while (total < len) {
        int n = static_cast<int>(::write(fd_, data + total, static_cast<size_t>(len - total)));
        if (n < 0) {
            if (errno == EAGAIN) {
                continue;
            }
            std::cerr << "Serial write error: " << std::strerror(errno) << "\n";
            return -1;
        }
        total += n;
    }
    return total;
}

bool SerialPort::setBaud(int baud) {
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "tcgetattr failed: " << std::strerror(errno) << "\n";
        return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    speed_t speed = baudToSpeed(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: " << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

}  // namespace gimbal_serial

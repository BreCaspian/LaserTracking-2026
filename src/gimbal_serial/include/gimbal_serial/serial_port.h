#ifndef GIMBAL_SERIAL_SERIAL_PORT_H
#define GIMBAL_SERIAL_SERIAL_PORT_H

#include <cstdint>
#include <string>
#include <vector>

namespace gimbal_serial {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool open(const std::string& device, int baud);
    bool reopen();
    void close();
    bool isOpen() const;

    int read(uint8_t* buffer, int max_len, int timeout_ms);
    int write(const uint8_t* data, int len);

private:
    bool setBaud(int baud);

    int fd_ = -1;
    std::string device_;
    int baud_ = 0;
};

}  // namespace gimbal_serial

#endif  // GIMBAL_SERIAL_SERIAL_PORT_H

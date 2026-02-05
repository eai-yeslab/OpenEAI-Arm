#include "serial_port.hpp"

#include <termios.h>
#include <sys/select.h>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <queue>

void print_data(const uint8_t* data, uint8_t len)
{
    for (int i = 0; i < len; i++)
    {
        printf("%02x ", data[i]);
    }
    printf("\n");
}

ssize_t SerialPort::send(const uint8_t* data, size_t len)
{
    // tcflush(fd_, TCIFLUSH);
    ssize_t ret = ::write(fd_, data, len);
    // tcdrain(fd_);
    return ret;
}

ssize_t SerialPort::recv(uint8_t* data, size_t len)
{
    FD_ZERO(&rSet_);
    FD_SET(fd_, &rSet_);
    ssize_t recv_len = 0;

    switch (select(fd_ + 1, &rSet_, NULL, NULL, &timeout_))
    {
    case -1: // error
        // std::cout << "communication error" << std::endl;
        break;
    case 0: // timeout
        // std::cout << "timeout" << std::endl;
        break;
    default:
        recv_len = ::read(fd_, data, len);
        break;
    }

    return recv_len;
}

void SerialPort::recv(uint8_t* data, uint8_t head, ssize_t len)
{
    // 存入队列
    ssize_t recv_len = this->recv(recv_buf.data(), len);
    for (int i = 0; i < recv_len; i++)
    {
        recv_queue.push(recv_buf[i]);
    }

    // 查找帧头
    while (recv_queue.size() >= len)
    {
        if(recv_queue.front() != head)
        {
        recv_queue.pop();
        continue;
        }
        break;
    }

    if(recv_queue.size() < len) return;

    // 读取数据
    for(int i = 0; i < len; i++)
    {
        data[i] = recv_queue.front();
        recv_queue.pop();
    }
}

void SerialPort::set_timeout(int timeout_ms)
{
    timeout_.tv_sec = timeout_ms / 1000;
    timeout_.tv_usec = (timeout_ms % 1000) * 1000;
}

void SerialPort::Init(std::string port, speed_t baudrate)
{
    int ret;
    // Open serial port
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
    {
      printf("Open serial port %s failed\n", port.c_str());
      exit(-1);
    }

    // Set attributes
    struct termios option;
    memset(&option, 0, sizeof(option));
    ret = tcgetattr(fd_, &option);

    option.c_oflag = 0;
    option.c_lflag = 0;
    option.c_iflag = 0;

    cfsetispeed(&option, baudrate);
    cfsetospeed(&option, baudrate);

    option.c_cflag &= ~CSIZE;
    option.c_cflag |= CS8; // 8
    option.c_cflag &= ~PARENB; // no parity
    option.c_iflag &= ~INPCK; // no parity
    option.c_cflag &= ~CSTOPB; // 1 stop bit

    option.c_cc[VTIME] = 0;
    option.c_cc[VMIN] = 0;
    option.c_lflag |= CBAUDEX;

    ret = tcflush(fd_, TCIFLUSH);
    ret = tcsetattr(fd_, TCSANOW, &option);
}
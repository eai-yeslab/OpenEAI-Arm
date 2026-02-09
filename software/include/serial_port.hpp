// Referenced from https://gitee.com/kit-miao/motor-control-routine/tree/master/C++%E4%BE%8B%E7%A8%8B/u2can/SerialPort.h

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

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

void print_data(const uint8_t* data, uint8_t len);

class SerialPort
{
public:
  using SharedPtr = std::shared_ptr<SerialPort>;

  SerialPort(std::string port, speed_t baudrate, int timeout_ms = 2)
  {
    set_timeout(timeout_ms);
    Init(port, baudrate);
  }

  ~SerialPort()
  {
    close(fd_);
  }

  ssize_t send(const uint8_t* data, size_t len);

  ssize_t recv(uint8_t* data, size_t len);

  void recv(uint8_t* data, uint8_t head, ssize_t len);

  void set_timeout(int timeout_ms);

private:
  int fd_;
  fd_set rSet_;
  timeval timeout_;

  std::queue<uint8_t> recv_queue;
  std::array<uint8_t, 1024> recv_buf;

  void Init(std::string port, speed_t baudrate);
};

#endif // SERIAL_PORT_H
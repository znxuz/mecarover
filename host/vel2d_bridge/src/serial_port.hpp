#pragma once

// init code from:
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <string_view>
#include <system_error>

template <size_t FRAME_SIZE>
class SerialPort {
 public:
  SerialPort(std::string_view name, speed_t baud_rate = B115200)
      : name_(name), baud_rate_(baud_rate) {
    open_fd();
    configure();
  }

  ~SerialPort() { close_fd(); }

  SerialPort(SerialPort &&rhs) noexcept
      : name_(std::move(rhs.name_)), baud_rate_(rhs.baud_rate_), fd_(rhs.fd_) {
    rhs.fd_ = -1;
  }

  SerialPort &operator=(SerialPort &&rhs) noexcept {
    if (this != &rhs) {
      close_fd();
      name_ = std::move(rhs.name_);
      baud_rate_ = rhs.baud_rate_;
      fd_ = rhs.fd_;
      rhs.fd_ = -1;
    }
    return *this;
  }

  bool send(const uint8_t *data) { return write(fd_, data, FRAME_SIZE); }

 private:
  std::string_view name_;
  speed_t baud_rate_;
  int fd_ = -1;

  void open_fd() {
    fd_ = open(name_.data(), O_RDWR);
    if (fd_ < 0)
      throw std::system_error(errno, std::generic_category(),
                              "Failed to open port");
  }

  void close_fd() {
    if (fd_ != -1) close(fd_);
  }

  void configure() {
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0)
      throw std::system_error(errno, std::generic_category(),
                              "Failed to get port attributes");

    tty.c_cflag &= ~PARENB;         // disabling parity bit
    tty.c_cflag &= ~CSTOPB;         // only one stop bit used
    tty.c_cflag &= ~CSIZE;          // clear all the size bits
    tty.c_cflag |= CS8;             // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;        // disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

    tty.c_lflag &= ~ICANON;  // disable canonical mode (receiving line by line)
    tty.c_lflag &= ~ECHO;    // disable echo
    tty.c_lflag &= ~ECHOE;   // disable erasure
    tty.c_lflag &= ~ECHONL;  // disable new-line echo
    tty.c_lflag &= ~ISIG;    // disable interpretation of INTR, QUIT and SUSP

    tty.c_oflag &= ~OPOST;  // prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR;  // prevent conversion of newline to crlf feed

    cfsetspeed(&tty, B115200);
    if (tcsetattr(fd_, TCSANOW, &tty))
      throw std::system_error(errno, std::generic_category(),
                              "Failed to set port attributes");
  }
};

/**
 * @file
 * This file declares an interface to SocketCAN,
 * to facilitates frame transmission and reception.
 */

#ifndef SOCKETCAN_H
#define SOCKETCAN_H

#include <linux/can.h>
#include <net/if.h>
#include <pthread.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <boost/lockfree/queue.hpp>
#include <cstdbool>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

constexpr const int INIT_FD = -1;
constexpr const int TIMEOUT_SEC = 1;
constexpr const int TIMEOUT_USEC = 0;
constexpr const int TX_QUEUE_SIZE = 1024;
constexpr const int MAX_RETRY_COUNT = 3;

using LFQueue = boost::lockfree::queue<can_frame, boost::lockfree::fixed_sized<true>>;
using CanCbkCondition = std::function<bool(const can_frame &)>;
using CanCbkFunc = std::function<void(const can_frame &)>;
using CanCbkId = std::string;
using CanCbkTuple = std::tuple<CanCbkId, CanCbkCondition, CanCbkFunc>;

class SocketCAN {
 private:
  std::atomic<bool> receiving_;

  std::string interface_;  // The network interface name
  int sockfd_ = -1;        // The file descriptor for the CAN socket
  sockaddr_can addr_;      // The address of the CAN socket
  ifreq if_request_;       // The network interface request

  /// Receiving
  std::thread receiver_thread_;
  std::vector<CanCbkTuple> can_callback_list_;
  std::mutex can_callback_mutex_;

  /// Transmitting
  std::thread sender_thread_;
  LFQueue tx_queue_;

  SocketCAN(std::string interface);

  static std::shared_ptr<SocketCAN> createInstance(const std::string &interface) {
    return std::shared_ptr<SocketCAN>(new SocketCAN(interface));
  }
  static std::shared_ptr<spdlog::logger> logger_;
  static std::unordered_map<std::string, std::shared_ptr<SocketCAN>> instances_;

 public:
  SocketCAN(const SocketCAN &) = delete;
  SocketCAN &operator=(const SocketCAN &) = delete;
  ~SocketCAN();
  static void init_logger(std::shared_ptr<spdlog::logger> logger) { logger_ = logger; }
  static std::shared_ptr<SocketCAN> get(std::string interface) {
    if (logger_.get() == nullptr) logger_ = spdlog::stdout_color_mt("SocketCAN");
    if (instances_.find(interface) == instances_.end()) instances_[interface] = createInstance(interface);
    return instances_[interface];
  }
  void open(std::string interface);
  void close();
  void transmit(const can_frame &frame);
  void receive();
  void add_can_callback(const CanCbkTuple &callback);
  void remove_can_callback(CanCbkId condition);
  void clear_can_callbacks();
};

#endif

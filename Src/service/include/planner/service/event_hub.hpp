#pragma once

// 进程内发布/订阅事件中心，用于 /plan/events（WebSocket）。
//
// 规划工作线程发布 JSON 事件（job_update / job_result）；每个已连接的
// WebSocket 订阅者拥有一个有界 FIFO 队列，由其自身的连接线程消费。慢速或
// 卡住的客户端不会阻塞工作线程：其队列溢出时订阅者被关闭并标记，连接线程
// 终止 WebSocket，客户端应重新连接并通过 HTTP（status/result 端点）重新同步。
//
// 事件中心本身线程安全，且独立于 HTTP 库。

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace planner::service {

class EventHub {
 public:
  // 一个连接的队列。线程安全。
  class Subscriber {
   public:
    explicit Subscriber(std::size_t capacity) : capacity_(capacity) {}

    Subscriber(const Subscriber&) = delete;
    Subscriber& operator=(const Subscriber&) = delete;

    // 入队一条消息。队列满时关闭订阅者并标记溢出（消息被丢弃；连接线程
    // 观察到关闭后终止套接字）。
    void push(std::string message);

    // 最多等待 `timeout` 时间获取一条消息。超时或订阅者已关闭时返回 false
    // （请检查 closed()/overflowed()）。
    bool pop(std::string* message, std::chrono::milliseconds timeout);

    bool closed() const;
    bool overflowed() const;
    void close();

   private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::string> queue_;
    bool closed_{false};
    bool overflowed_{false};
    std::size_t capacity_;
  };

  explicit EventHub(std::size_t per_subscriber_capacity = 4096)
      : capacity_(per_subscriber_capacity) {}

  // 注册一个订阅者。调用方必须在连接结束时调用 unsubscribe()。
  std::shared_ptr<Subscriber> subscribe();
  void unsubscribe(const std::shared_ptr<Subscriber>& subscriber);

  // 向当前每个订阅者发布一个事件文档。
  void publish(const nlohmann::json& event);

  // 关闭所有订阅者（服务关闭）。
  void close_all();

  std::size_t subscriber_count() const;

 private:
  std::size_t capacity_;
  mutable std::mutex mutex_;
  std::vector<std::shared_ptr<Subscriber>> subscribers_;
};

}  // namespace planner::service

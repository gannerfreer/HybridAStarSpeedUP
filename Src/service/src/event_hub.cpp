// 发布/订阅事件中心（见 event_hub.hpp）。

#include "planner/service/event_hub.hpp"

#include <algorithm>
#include <utility>

namespace planner::service {

void EventHub::Subscriber::push(std::string message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }
    if (queue_.size() >= capacity_) {
      // 卡住的客户端不能阻塞工作线程：断开连接，让其通过 HTTP 重新同步
      // （连接线程观察到的正是关闭标志）。
      overflowed_ = true;
      closed_ = true;
      queue_.clear();
    } else {
      queue_.push_back(std::move(message));
    }
  }
  cv_.notify_all();
}

bool EventHub::Subscriber::pop(std::string* message, std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!cv_.wait_for(lock, timeout, [&] { return closed_ || !queue_.empty(); })) {
    return false;
  }
  if (queue_.empty()) {
    return false;  // 已关闭且队列为空
  }
  *message = std::move(queue_.front());
  queue_.pop_front();
  return true;
}

bool EventHub::Subscriber::closed() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return closed_;
}

bool EventHub::Subscriber::overflowed() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return overflowed_;
}

void EventHub::Subscriber::close() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
  }
  cv_.notify_all();
}

std::shared_ptr<EventHub::Subscriber> EventHub::subscribe() {
  auto subscriber = std::make_shared<Subscriber>(capacity_);
  std::lock_guard<std::mutex> lock(mutex_);
  subscribers_.push_back(subscriber);
  return subscriber;
}

void EventHub::unsubscribe(const std::shared_ptr<Subscriber>& subscriber) {
  std::lock_guard<std::mutex> lock(mutex_);
  subscribers_.erase(
      std::remove(subscribers_.begin(), subscribers_.end(), subscriber),
      subscribers_.end());
}

void EventHub::publish(const nlohmann::json& event) {
  const std::string message = event.dump();
  std::lock_guard<std::mutex> lock(mutex_);
  for (const std::shared_ptr<Subscriber>& subscriber : subscribers_) {
    subscriber->push(message);
  }
}

void EventHub::close_all() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const std::shared_ptr<Subscriber>& subscriber : subscribers_) {
    subscriber->close();
  }
}

std::size_t EventHub::subscriber_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return subscribers_.size();
}

}  // namespace planner::service

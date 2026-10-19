// 规划输入快照的内存 LRU 存储实现（见 snapshot_store.hpp）。

#include "planner/service/snapshot_store.hpp"

#include <utility>

namespace planner::service {

SceneSnapshotStore::SceneSnapshotStore(std::size_t capacity) : capacity_(capacity) {}

bool SceneSnapshotStore::put(const std::string& scene_id, std::string snapshot) {
  if (capacity_ == 0U) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = index_.find(scene_id);
  if (it != index_.end()) {
    // 去重：更新内容并把条目移到最近使用位置。
    it->second->second = std::move(snapshot);
    entries_.splice(entries_.begin(), entries_, it->second);
    return false;
  }
  entries_.emplace_front(scene_id, std::move(snapshot));
  index_.emplace(scene_id, entries_.begin());
  while (entries_.size() > capacity_) {
    index_.erase(entries_.back().first);
    entries_.pop_back();
  }
  return true;
}

std::optional<std::string> SceneSnapshotStore::get(const std::string& scene_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = index_.find(scene_id);
  if (it == index_.end()) {
    return std::nullopt;
  }
  entries_.splice(entries_.begin(), entries_, it->second);
  return it->second->second;
}

std::size_t SceneSnapshotStore::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_.size();
}

}  // namespace planner::service

#pragma once

// 规划输入快照的内存 LRU 存储（栅格快照协议 v1 §3.4，X-M1）。
//
// 语义（协议 §3.4）：
//   * 保留最近 16 个规划输入快照，按 scene_id 去重（相同内容只存一份）；
//   * 重启失效（首版；纯内存）；
//   * 容量档/大图不进快照（扩展计划 §3.2/§6.1：在线验收 ≤1024²；2048²/
//     4096² 属离线容量档）。服务在调用 put 之前按 kMaxSnapshotCells 判断。
//
// 线程安全：规划工作线程写入、HTTP 处理线程读取，内部互斥锁串行化。

#include <cstddef>
#include <list>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

namespace planner::service {

// 协议 §3.4：LRU 容量 16。
inline constexpr std::size_t kDefaultSceneSnapshotCapacity = 16;

// 单条快照的最大单元数（在线验收范围 1024²；容量档不进快照）。
inline constexpr std::size_t kMaxSnapshotCells = 1024u * 1024u;

class SceneSnapshotStore {
 public:
  explicit SceneSnapshotStore(
      std::size_t capacity = kDefaultSceneSnapshotCapacity);

  SceneSnapshotStore(const SceneSnapshotStore&) = delete;
  SceneSnapshotStore& operator=(const SceneSnapshotStore&) = delete;

  // 写入或刷新一条快照（按 scene_id 去重）。相同 scene_id 已存在时更新
  // 内容并标记为最近使用；容量为 0 时不存储。返回 true 表示新插入。
  bool put(const std::string& scene_id, std::string snapshot);

  // 读取快照并标记为最近使用；未知/已淘汰时返回 std::nullopt。
  std::optional<std::string> get(const std::string& scene_id) const;

  // 当前条目数（测试与运行诊断用）。
  std::size_t size() const;

  std::size_t capacity() const noexcept { return capacity_; }

 private:
  using Entry = std::pair<std::string, std::string>;  // scene_id -> snapshot

  const std::size_t capacity_;
  mutable std::mutex mutex_;
  // 最近使用在前；get() 会刷新最近使用位置，因此为 mutable。
  mutable std::list<Entry> entries_;
  mutable std::unordered_map<std::string, std::list<Entry>::iterator> index_;
};

}  // namespace planner::service

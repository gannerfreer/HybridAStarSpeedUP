#pragma once

// W5 规划服务核心：作业注册表、单个规划工作线程和面向 HTTP 的操作。
// HTTP/WebSocket 接线位于 src/main.cpp；本类是库代码，以便测试直接调用，
// 服务二进制保持为薄适配层。
//
// 并发模型：
//   * 一个工作线程严格按提交顺序执行作业；
//   * submit/cancel/status/result 可从任意 HTTP 处理线程安全调用；
//     作业状态切换和事件发布在同一把互斥锁下串行化；
//   * /plan/events 订阅者在每次状态或阶段切换时收到 job_update 事件，
//     并在作业到达终态时收到一个 job_result 事件（EventHub 为注入；
//     空 hub 禁用发布）。
//
// 取消：排队中的作业立即取消；运行中的作业设置协作式标志，在阶段边界和
// Hybrid A* 搜索内部被观察。Dftpav 阶段无法中断，返回时才观察到取消
// （README“取消语义”）。

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include "planner/map/map.hpp"
#include "planner/map/types.hpp"
#include "planner/service/event_hub.hpp"
#include "planner/service/scene.hpp"
#include "planner/service/snapshot_store.hpp"
#include "planner/service/types.hpp"

namespace planner::service {

struct ServiceConfig {
  std::string host{"127.0.0.1"};
  int port{8766};
  std::string run_dir;
  SceneBuildOptions scene;
};

class PlanningService {
 public:
  // `map` 必须比本实例存活更久（服务只保存指针）。地图仅在规划工作线程上
  // 通过 const 接口使用（场景构建的可行驶域导出），因此满足
  // planner/map/map.hpp 的串行化要求；HTTP 处理线程只读取缓存的
  // map_info()。`hub` 可以为 null（不发布事件）。
  PlanningService(const map::Map& map, ServiceConfig config, EventHub* hub);
  ~PlanningService();

  PlanningService(const PlanningService&) = delete;
  PlanningService& operator=(const PlanningService&) = delete;

  // GET /health。
  HttpResponse health() const;

  // POST /plan/submit。400 invalid_input / 409 stale_map / 200 queued。
  HttpResponse submit(const nlohmann::json& body);

  // POST /plan/cancel。200，status 为 cancelled | not_found | already_finished。
  HttpResponse cancel(const nlohmann::json& body);

  // GET /plan/status/{job_id}。id 未知时返回 404 not_found。
  HttpResponse status(const std::string& job_id) const;

  // GET /plan/result/{job_id}。id 未知时返回 404 not_found；排队中和运行中的
  // 作业返回当前状态，不含几何数据。
  HttpResponse result(const std::string& job_id) const;

  // GET /plan/scene/{scene_id}（协议 §3.4，X-M1）。命中时返回规划实际使用的
  // grid_snapshot 原文（逐字节一致）；未知或已淘汰返回 404 not_found/
  // unknown_scene。
  HttpResponse scene_snapshot(const std::string& scene_id) const;

  // 停止工作线程：非终态作业被取消（"service_shutdown"），工作线程在运行中
  // 的作业观察到取消后 join。
  void stop();

  const map::MapInfo& map_info() const noexcept { return map_->info(); }
  const ServiceConfig& config() const noexcept { return config_; }
  std::uint64_t current_job_seq() const;

 private:
  struct Job;

  void worker_loop();
  void run_job(const std::shared_ptr<Job>& job);
  void publish_update(const Job& job);
  void publish_result(const Job& job);
  nlohmann::json result_body(const Job& job) const;
  nlohmann::json status_body(const Job& job) const;

  const map::Map* map_{nullptr};
  ServiceConfig config_;
  EventHub* hub_{nullptr};
  SceneSnapshotStore snapshots_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::map<std::string, std::shared_ptr<Job>> jobs_;
  std::deque<std::shared_ptr<Job>> queue_;
  std::uint64_t next_job_seq_{0};
  bool stopping_{false};
  std::thread worker_;

  std::chrono::steady_clock::time_point started_;
};

}  // namespace planner::service

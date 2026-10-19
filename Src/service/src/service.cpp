// 作业注册表、工作线程和面向 HTTP 的操作（见 service.hpp）。

#include "planner/service/service.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include "planner/common/serialization.hpp"
#include "planner/scene/content_id.hpp"
#include "planner/scene/snapshot.hpp"
#include "planner/service/parse.hpp"
#include "planner/service/pipeline.hpp"

namespace planner::service {

struct PlanningService::Job {
  std::string job_id;
  std::uint64_t job_seq{0};
  std::string request_id;
  std::string map_id;
  std::string map_version;
  ServicePlanRequest request;

  JobState state{JobState::kQueued};
  std::string stage{stage::kMap};
  std::string reason;
  std::string detail;
  std::atomic<bool> cancel_requested{false};

  // X-M1（协议 §3.4）：map 阶段场景构建成功后出现；成功与失败都携带。
  std::string scene_id;
  std::string scene_version;

  PipelineResult pipeline;
  bool has_pipeline{false};
};

namespace {

HttpResponse error_response(int http_status, const std::string& status,
                            const std::string& reason) {
  HttpResponse response;
  response.status = http_status;
  response.body = envelope("error");
  response.body["status"] = status;
  response.body["reason"] = reason;
  return response;
}

}  // namespace

PlanningService::PlanningService(const map::Map& map, ServiceConfig config,
                                 EventHub* hub)
    : map_(&map),
      config_(std::move(config)),
      hub_(hub),
      started_(std::chrono::steady_clock::now()) {
  worker_ = std::thread([this] { worker_loop(); });
}

PlanningService::~PlanningService() { stop(); }

void PlanningService::stop() {
  std::vector<std::shared_ptr<Job>> newly_cancelled;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {
      return;
    }
    stopping_ = true;
    for (auto& entry : jobs_) {
      const std::shared_ptr<Job>& job = entry.second;
      if (is_terminal(job->state)) {
        continue;
      }
      job->cancel_requested = true;
      if (job->state == JobState::kQueued) {
        job->state = JobState::kCancelled;
        job->stage = stage::kDone;
        job->reason = "cancelled";
        job->detail = "service shutdown";
        newly_cancelled.push_back(job);
      }
    }
  }
  cv_.notify_all();
  if (hub_ != nullptr) {
    hub_->close_all();
  }
  for (const std::shared_ptr<Job>& job : newly_cancelled) {
    publish_update(*job);
    publish_result(*job);
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

HttpResponse PlanningService::health() const {
  const map::MapInfo& info = map_->info();
  HttpResponse response;
  response.body = envelope("health");
  response.body["status"] = "ok";
  response.body["map_id"] = info.map_id;
  response.body["map_version"] = info.map_version;
  response.body["uptime_s"] =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
  // 协议 §3.5：版本一致性字段（grid_protocol_version / scene_rules_version）。
  response.body["grid_protocol_version"] = scene::kGridProtocolVersion;
  response.body["scene_rules_version"] = scene::kSceneVersion;
  return response;
}

HttpResponse PlanningService::submit(const nlohmann::json& body) {
  ServicePlanRequest request;
  const ParseOutcome parsed = parse_plan_request(body, &request);
  if (!parsed.ok) {
    return error_response(400, "invalid_input",
                          parsed.detail.empty() ? parsed.reason : parsed.detail);
  }
  const map::MapInfo& map_info = map_->info();
  if (request.map_id != map_info.map_id ||
      request.map_version != map_info.map_version) {
    return error_response(
        409, "stale_map",
        "request references map_id '" + request.map_id + "' version '" +
            request.map_version + "'; loaded map is '" + map_info.map_id +
            "' version '" + map_info.map_version + "'");
  }

  auto job = std::make_shared<Job>();
  job->request = std::move(request);
  job->request_id = job->request.request_id;
  job->map_id = job->request.map_id;
  job->map_version = job->request.map_version;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    job->job_seq = ++next_job_seq_;
    job->job_id = "job-" + std::to_string(job->job_seq);
    job->state = JobState::kQueued;
    job->stage = stage::kMap;
    jobs_[job->job_id] = job;
    queue_.push_back(job);
  }
  cv_.notify_all();
  publish_update(*job);

  HttpResponse response;
  response.body = envelope("plan_submit");
  response.body["job_id"] = job->job_id;
  response.body["job_seq"] = job->job_seq;
  response.body["request_id"] = job->request_id;
  response.body["map_id"] = job->map_id;
  response.body["map_version"] = job->map_version;
  response.body["status"] = "queued";
  return response;
}

HttpResponse PlanningService::cancel(const nlohmann::json& body) {
  if (!body.is_object() || !body.contains("job_id") ||
      !body.at("job_id").is_string() ||
      body.at("job_id").get<std::string>().empty()) {
    return error_response(400, "invalid_input",
                          "field 'job_id' must be a non-empty string");
  }
  const std::string job_id = body.at("job_id").get<std::string>();

  std::shared_ptr<Job> job;
  bool publish = false;
  HttpResponse response;
  response.body = envelope("plan_cancel");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = jobs_.find(job_id);
    if (it == jobs_.end()) {
      response.body["job_id"] = job_id;
      response.body["status"] = "not_found";
      response.body["reason"] = "unknown job_id: " + job_id;
      return response;
    }
    job = it->second;
    response.body["job_id"] = job->job_id;
    response.body["job_seq"] = job->job_seq;
    if (is_terminal(job->state)) {
      response.body["status"] = "already_finished";
    } else if (job->state == JobState::kQueued) {
      // 仍在排队：立即取消；工作线程会跳过已取消的作业。
      job->cancel_requested = true;
      job->state = JobState::kCancelled;
      job->stage = stage::kDone;
      job->reason = "cancelled";
      job->detail = "cancelled while queued";
      publish = true;
      response.body["status"] = "cancelled";
    } else {
      // 运行中：协作式取消；最终状态通过 GET /plan/status 和
      // /plan/events 报告。
      job->cancel_requested = true;
      response.body["status"] = "cancelled";
    }
  }
  if (publish) {
    publish_update(*job);
    publish_result(*job);
  }
  return response;
}

HttpResponse PlanningService::status(const std::string& job_id) const {
  std::shared_ptr<Job> job;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = jobs_.find(job_id);
    if (it == jobs_.end()) {
      return error_response(404, "not_found", "unknown job_id: " + job_id);
    }
    job = it->second;
  }
  HttpResponse response;
  response.body = status_body(*job);
  return response;
}

HttpResponse PlanningService::result(const std::string& job_id) const {
  std::shared_ptr<Job> job;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = jobs_.find(job_id);
    if (it == jobs_.end()) {
      return error_response(404, "not_found", "unknown job_id: " + job_id);
    }
    job = it->second;
  }
  HttpResponse response;
  try {
    response.body = result_body(*job);
  } catch (const std::exception& error) {
    return error_response(500, "internal_error",
                          std::string("result encoding failed: ") + error.what());
  }
  return response;
}

HttpResponse PlanningService::scene_snapshot(const std::string& scene_id) const {
  const std::optional<std::string> snapshot = snapshots_.get(scene_id);
  HttpResponse response;
  if (!snapshot.has_value()) {
    // 协议 §3.4：未知或已淘汰（含容量档未保留）。
    return error_response(404, "not_found", "unknown_scene");
  }
  response.status = 200;
  // 逐字节一致：返回规划时保存的原文，不重新序列化。
  response.raw_body = *snapshot;
  return response;
}

std::uint64_t PlanningService::current_job_seq() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return next_job_seq_;
}

// ---------------------------------------------------------------------------
// 工作线程。
// ---------------------------------------------------------------------------

void PlanningService::worker_loop() {
  for (;;) {
    std::shared_ptr<Job> job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
      if (queue_.empty()) {
        if (stopping_) {
          break;
        }
        continue;
      }
      job = queue_.front();
      queue_.pop_front();
      if (job->state == JobState::kCancelled) {
        continue;  // 排队期间已取消；已发布
      }
      job->state = JobState::kRunning;
      job->stage = stage::kMap;
    }
    publish_update(*job);
    run_job(job);
  }
}

void PlanningService::run_job(const std::shared_ptr<Job>& job) {
  const auto clock = [] { return std::chrono::steady_clock::now(); };
  const auto elapsed_s = [](const auto& from, const auto& to) {
    return std::chrono::duration<double>(to - from).count();
  };
  const auto set_state = [&](JobState state, const std::string& stage_name) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      job->state = state;
      job->stage = stage_name;
    }
    publish_update(*job);
  };
  const auto finish = [&](JobState state, const std::string& reason,
                          const std::string& detail) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      job->state = state;
      job->stage = stage::kDone;
      job->reason = reason;
      job->detail = detail;
    }
    publish_update(*job);
    publish_result(*job);
  };

  // "map" 阶段：本地网格 + 碰撞场景构建。
  set_state(JobState::kRunning, stage::kMap);
  if (job->cancel_requested.load()) {
    finish(JobState::kCancelled, "cancelled", "cancelled while queued");
    return;
  }
  const auto map_start = clock();
  const SceneBuildResult scene =
      build_scene(*map_, job->request.vehicle, job->request.start,
                  job->request.goal, job->request.obstacles, config_.scene);
  const double map_time_s = elapsed_s(map_start, clock());
  if (!scene.ok) {
    finish(JobState::kFailed, scene.reason, scene.detail);
    return;
  }
  // X-M1（协议 §3.4）：场景构建（map 阶段）完成后记录内容标识；成功与失败
  // 都携带。随后保存本次规划实际使用的输入快照（内存 LRU 16，按 scene_id
  // 去重；容量档/大图不进快照）。
  {
    std::lock_guard<std::mutex> lock(mutex_);
    job->scene_id = scene.scene_id;
    job->scene_version = scene.scene_version;
  }
  const std::size_t scene_cells = static_cast<std::size_t>(scene.grid.width) *
                                  static_cast<std::size_t>(scene.grid.height);
  if (scene_cells <= kMaxSnapshotCells && !scene.scene_id.empty()) {
    try {
      scene::GridSnapshotOptions snapshot_options;
      snapshot_options.source = "planning_scene";
      snapshot_options.scene_id = scene.scene_id;
      snapshot_options.scene_version = scene.scene_version;
      snapshot_options.window = scene.window;
      snapshot_options.requested_resolution_m = scene.requested_resolution_m;
      snapshot_options.budget_cells = config_.scene.max_cells;
      const nlohmann::json snapshot =
          scene::make_grid_snapshot_json(scene.grid, snapshot_options);
      snapshots_.put(scene.scene_id, snapshot.dump());
    } catch (const std::exception&) {
      // 快照是结果/状态的关联展示入口；编码失败不改变规划结果本身，
      // 仅不提供 GET /plan/scene/{scene_id}（客户端可用预览重新生成）。
    }
  }
  if (job->cancel_requested.load()) {
    finish(JobState::kCancelled, "cancelled", "cancelled after the map stage");
    return;
  }

  PipelineResult pipeline = run_pipeline(
      job->request, scene.grid, scene.scene,
      [&](const std::string& stage_name) { set_state(JobState::kRunning, stage_name); },
      [&] { return job->cancel_requested.load(); });
  if (pipeline.statistics.is_object()) {
    const double total_s = map_time_s + pipeline.hybrid_time_s +
                           pipeline.corridor_time_s + pipeline.optimize_time_s +
                           pipeline.validation_time_s;
    pipeline.statistics["timings_s"]["map"] = map_time_s;
    pipeline.statistics["timings_s"]["total"] = total_s;
  }

  const common::Status status = pipeline.status;
  const std::string reason = pipeline.reason;
  const std::string detail = pipeline.detail;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    job->pipeline = std::move(pipeline);
    job->has_pipeline = true;
  }
  if (status == common::Status::kSuccess) {
    finish(JobState::kSucceeded, "", "");
  } else if (status == common::Status::kCancelled) {
    finish(JobState::kCancelled, "cancelled", detail);
  } else {
    finish(JobState::kFailed, reason, detail);
  }
}

// ---------------------------------------------------------------------------
// 事件发布与响应体。
// ---------------------------------------------------------------------------

void PlanningService::publish_update(const Job& job) {
  if (hub_ == nullptr) {
    return;
  }
  JobState state = JobState::kQueued;
  std::string stage_name;
  std::string scene_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = job.state;
    stage_name = job.stage;
    scene_id = job.scene_id;
  }
  nlohmann::json event = envelope("job_update");
  event["job_id"] = job.job_id;
  event["request_id"] = job.request_id;
  event["job_seq"] = job.job_seq;
  event["map_version"] = job.map_version;
  // 协议 §3.4：map 阶段完成后可携带 scene_id（HTTP 为规范来源）。
  if (!scene_id.empty()) {
    event["scene_id"] = scene_id;
  }
  event["status"] = to_string(state);
  event["stage"] = stage_name;
  hub_->publish(event);
}

void PlanningService::publish_result(const Job& job) {
  if (hub_ == nullptr) {
    return;
  }
  std::string scene_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scene_id = job.scene_id;
  }
  nlohmann::json event = envelope("job_result");
  event["job_id"] = job.job_id;
  event["request_id"] = job.request_id;
  event["job_seq"] = job.job_seq;
  event["map_version"] = job.map_version;
  if (!scene_id.empty()) {
    event["scene_id"] = scene_id;
  }
  hub_->publish(event);
}

nlohmann::json PlanningService::status_body(const Job& job) const {
  JobState state = JobState::kQueued;
  std::string stage_name;
  std::string reason;
  std::string detail;
  std::string scene_id;
  std::string scene_version;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = job.state;
    stage_name = job.stage;
    reason = job.reason;
    detail = job.detail;
    scene_id = job.scene_id;
    scene_version = job.scene_version;
  }
  nlohmann::json body = envelope("plan_status");
  body["job_id"] = job.job_id;
  body["request_id"] = job.request_id;
  body["job_seq"] = job.job_seq;
  body["status"] = to_string(state);
  body["stage"] = stage_name;
  body["map_id"] = job.map_id;
  body["map_version"] = job.map_version;
  // 协议 §3.4：场景构建（map 阶段）完成后出现；之前缺省。
  if (!scene_id.empty()) {
    body["scene_id"] = scene_id;
    body["scene_version"] = scene_version;
  }
  if (!reason.empty()) {
    body["reason"] = reason;
  }
  if (!detail.empty()) {
    body["detail"] = detail;
  }
  return body;
}

nlohmann::json PlanningService::result_body(const Job& job) const {
  std::lock_guard<std::mutex> lock(mutex_);
  nlohmann::json body = envelope("plan_result");
  body["job_id"] = job.job_id;
  body["request_id"] = job.request_id;
  body["job_seq"] = job.job_seq;
  body["map_id"] = job.map_id;
  body["map_version"] = job.map_version;
  body["status"] = to_string(job.state);
  body["stage"] = job.stage;
  // 协议 §3.4：场景已构建即出现（成功与失败都出现）。
  if (!job.scene_id.empty()) {
    body["scene_id"] = job.scene_id;
    body["scene_version"] = job.scene_version;
  }
  if (!job.reason.empty()) {
    body["reason"] = job.reason;
  }
  if (!job.detail.empty()) {
    body["detail"] = job.detail;
  }
  if (job.has_pipeline) {
    const PipelineResult& pipeline = job.pipeline;
    if (job.state == JobState::kSucceeded) {
      nlohmann::json segments = nlohmann::json::array();
      for (const common::PathSegment& segment : pipeline.hybrid.path) {
        segments.push_back(common::encode_payload<common::PathSegment>(segment));
      }
      body["hybrid"] = nlohmann::json{{"backend", pipeline.hybrid_backend},
                                      {"segments", std::move(segments)},
                                      {"statistics", pipeline.hybrid.statistics}};
      nlohmann::json corridor_body =
          common::encode_payload<common::Corridor>(pipeline.corridor.corridor);
      corridor_body["statistics"] = pipeline.statistics.value("corridor", nlohmann::json::object());
      body["corridor"] = std::move(corridor_body);
      nlohmann::json trajectory = nlohmann::json::array();
      for (const common::TrajectoryPoint& point : pipeline.optimize.trajectory) {
        trajectory.push_back(common::encode_payload<common::TrajectoryPoint>(point));
      }
      body["trajectory"] = std::move(trajectory);
      body["validation"] = pipeline.statistics.value("validation", nlohmann::json::object());
    }
    body["statistics"] = pipeline.statistics;
  }
  return body;
}

}  // namespace planner::service

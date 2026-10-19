import type { PlanEvent, PlanSubmitResponse } from './types';

/**
 * 陈旧防护（验收关键）：客户端维护最新 `job_seq`（单调）与"当前代际"活动任务集合。
 *  - 一次用户提交动作开启新代际（beginGeneration）：清空活动集合（旧结果被清空），
 *    代际内可包含多个 job（对比模式两个后端）；
 *  - 提交响应：`job_seq < latest_seq` 视为乱序响应，忽略；
 *  - 推送事件：`job_seq < latest_seq` 一律丢弃（陈旧事件）；同 seq 仅接受活动任务；
 *  - 轮询状态/结果：仅接受当前代际的活动 job（旧 job 的迟到响应不得覆盖新任务）。
 */

export interface JobTrackerState {
  latest_seq: number;
  active_job_ids: readonly string[];
  stale_ignored: number;
}

export interface TrackerOutcome {
  state: JobTrackerState;
  accepted: boolean;
  reason: string;
}

export function createTrackerState(): JobTrackerState {
  return { latest_seq: 0, active_job_ids: [], stale_ignored: 0 };
}

/** 新提交动作：开启新代际（旧任务集合清空，seq 计数保留）。 */
export function beginGeneration(state: JobTrackerState): JobTrackerState {
  return { ...state, active_job_ids: [] };
}

export function isActiveJob(state: JobTrackerState, jobId: string): boolean {
  return state.active_job_ids.includes(jobId);
}

function withActiveJob(state: JobTrackerState, jobId: string): JobTrackerState {
  if (state.active_job_ids.includes(jobId)) return state;
  return { ...state, active_job_ids: [...state.active_job_ids, jobId] };
}

export function acceptSubmit(state: JobTrackerState, submit: PlanSubmitResponse): TrackerOutcome {
  if (submit.job_seq < state.latest_seq) {
    return {
      state: { ...state, stale_ignored: state.stale_ignored + 1 },
      accepted: false,
      reason: `提交响应 job_seq ${submit.job_seq} < 最新 ${state.latest_seq}（乱序响应，已忽略）`,
    };
  }
  return {
    state: withActiveJob({ ...state, latest_seq: submit.job_seq }, submit.job_id),
    accepted: true,
    reason: '',
  };
}

export function acceptEvent(state: JobTrackerState, event: PlanEvent): TrackerOutcome {
  if (event.job_seq < state.latest_seq) {
    return {
      state: { ...state, stale_ignored: state.stale_ignored + 1 },
      accepted: false,
      reason: `事件 job_seq ${event.job_seq} < 最新 ${state.latest_seq}（陈旧事件，已忽略）`,
    };
  }
  if (event.job_seq === state.latest_seq && !isActiveJob(state, event.job_id)) {
    return {
      state: { ...state, stale_ignored: state.stale_ignored + 1 },
      accepted: false,
      reason: `事件 job ${event.job_id} 不在当前代际活动任务中（同 seq，已忽略）`,
    };
  }
  return {
    state: withActiveJob({ ...state, latest_seq: Math.max(state.latest_seq, event.job_seq) }, event.job_id),
    accepted: true,
    reason: '',
  };
}

export function acceptJobUpdate(
  state: JobTrackerState,
  jobId: string,
  jobSeq?: number,
): TrackerOutcome {
  // 轮询以"当前代际活动任务"为界：对比模式下较早提交的 job（seq 较小）仍属有效；
  // 非活动任务（上一代际/未知 job）的迟到响应一律丢弃。
  if (!isActiveJob(state, jobId)) {
    return {
      state: { ...state, stale_ignored: state.stale_ignored + 1 },
      accepted: false,
      reason: `job ${jobId} 不在当前代际活动任务中（旧任务响应不得覆盖，已忽略）`,
    };
  }
  return { state, accepted: true, reason: '' };
}

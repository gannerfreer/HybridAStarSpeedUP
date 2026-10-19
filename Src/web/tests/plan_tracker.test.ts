import { describe, expect, it } from 'vitest';
import {
  acceptEvent,
  acceptJobUpdate,
  acceptSubmit,
  beginGeneration,
  createTrackerState,
  isActiveJob,
} from '../src/core/plan/tracker';

const submitResponse = (jobId: string, seq: number) => ({
  job_id: jobId,
  job_seq: seq,
  request_id: `req-${jobId}`,
  map_id: 'm',
  map_version: 'v1',
  status: 'queued',
});

describe('陈旧防护（验收关键）', () => {
  it('提交响应：seq 单调接受；乱序响应被忽略', () => {
    let state = createTrackerState();
    const first = acceptSubmit(state, submitResponse('job-1', 1));
    expect(first.accepted).toBe(true);
    state = first.state;
    expect(state.latest_seq).toBe(1);
    expect(isActiveJob(state, 'job-1')).toBe(true);

    const second = acceptSubmit(state, submitResponse('job-2', 2));
    expect(second.accepted).toBe(true);
    state = second.state;
    expect(state.latest_seq).toBe(2);

    // 乱序：旧 seq 的提交响应（例如网络迟到）被忽略
    const late = acceptSubmit(state, submitResponse('job-0', 1));
    expect(late.accepted).toBe(false);
    expect(late.state.stale_ignored).toBe(1);
    expect(isActiveJob(late.state, 'job-0')).toBe(false);
  });

  it('推送事件：陈旧 seq 一律丢弃', () => {
    let state = acceptSubmit(createTrackerState(), submitResponse('job-2', 2)).state;
    const stale = acceptEvent(state, { kind: 'job_update', job_seq: 1, job_id: 'job-1', status: 'running' });
    expect(stale.accepted).toBe(false);
    expect(stale.reason).toMatch(/陈旧事件/);
    expect(stale.state.stale_ignored).toBe(1);
    expect(stale.state.latest_seq).toBe(2);

    const fresh = acceptEvent(state, { kind: 'job_update', job_seq: 3, job_id: 'job-2', status: 'running', stage: 'corridor' });
    expect(fresh.accepted).toBe(true);
    expect(fresh.state.latest_seq).toBe(3);
  });

  it('推送事件：同 seq 非活动任务被忽略', () => {
    const state = acceptSubmit(createTrackerState(), submitResponse('job-2', 2)).state;
    const other = acceptEvent(state, { kind: 'job_update', job_seq: 2, job_id: 'job-x', status: 'running' });
    expect(other.accepted).toBe(false);
    expect(other.reason).toMatch(/不在当前代际/);
  });

  it('轮询状态/结果：旧 job 的迟到响应不得覆盖新任务', () => {
    let state = acceptSubmit(createTrackerState(), submitResponse('job-1', 1)).state;
    // 新提交动作开启新代际（App 提交路径）
    state = beginGeneration(state);
    state = acceptSubmit(state, submitResponse('job-2', 2)).state;
    const old = acceptJobUpdate(state, 'job-1');
    expect(old.accepted).toBe(false);
    expect(old.reason).toMatch(/不得覆盖/);
    const current = acceptJobUpdate(state, 'job-2');
    expect(current.accepted).toBe(true);
  });

  it('新提交开启新代际：旧任务集合清空（旧结果不显示）', () => {
    let state = acceptSubmit(createTrackerState(), submitResponse('job-1', 1)).state;
    state = acceptSubmit(state, submitResponse('job-2', 2)).state;
    expect(state.active_job_ids).toEqual(['job-1', 'job-2']);
    state = beginGeneration(state);
    expect(state.active_job_ids).toEqual([]);
    expect(acceptJobUpdate(state, 'job-1').accepted).toBe(false);
    expect(acceptJobUpdate(state, 'job-2').accepted).toBe(false);
    // 新代际提交后旧任务仍不可用
    const fresh = acceptSubmit(state, submitResponse('job-3', 3));
    expect(fresh.accepted).toBe(true);
    expect(acceptJobUpdate(fresh.state, 'job-3').accepted).toBe(true);
    expect(acceptJobUpdate(fresh.state, 'job-1').accepted).toBe(false);
  });

  it('对比模式：一次代际内两个 job 均有效', () => {
    let state = beginGeneration(createTrackerState());
    state = acceptSubmit(state, submitResponse('job-a', 1)).state;
    state = acceptSubmit(state, submitResponse('job-b', 2)).state;
    expect(state.active_job_ids).toEqual(['job-a', 'job-b']);
    expect(acceptJobUpdate(state, 'job-a').accepted).toBe(true);
    expect(acceptJobUpdate(state, 'job-b').accepted).toBe(true);
    // 事件更新 job-a（seq 落后于 job-b 提交）——属于当前代际活动任务但 seq 更旧：
    const lateEvent = acceptEvent(state, { kind: 'job_update', job_seq: 1, job_id: 'job-a', status: 'running' });
    expect(lateEvent.accepted).toBe(false); // 陈旧 seq 丢弃（结果仍可由轮询更新）
    const freshEvent = acceptEvent(state, { kind: 'job_result', job_seq: 3, job_id: 'job-a', status: 'success' });
    expect(freshEvent.accepted).toBe(true);
    expect(freshEvent.state.latest_seq).toBe(3);
  });
});

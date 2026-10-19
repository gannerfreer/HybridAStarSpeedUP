import { useCallback, useEffect, useMemo, useRef, useState, type DragEvent } from 'react';
import { GridPanel, type GridHealthView, type GridOnlineSourceView, type GridPanelSource } from './components/GridPanel';
import { InspectorPanel } from './components/InspectorPanel';
import { MapCanvas, type CanvasSize } from './components/MapCanvas';
import { PlanPanel, type PlanRunView } from './components/PlanPanel';
import { PlanScenePanel } from './components/PlanScenePanel';
import { PlaybackPanel } from './components/PlaybackPanel';
import { ScenePanel } from './components/ScenePanel';
import { StatusBar } from './components/StatusBar';
import { Toolbar } from './components/Toolbar';
import { cellInfo, countStates, gridWorldBounds, worldToCell } from './core/grid/coords';
import {
  GRID_CATALOG_URL,
  GRID_FIXTURE_URL_PREFIX,
  GridLoadTracker,
  loadGridFixture,
  parseGridCatalog,
} from './core/grid/load';
import {
  acceptOnlineGridResponse,
  checkGridVersionConsistency,
  fitDisplayResolution,
  GridSourceTracker,
  OnlineGridClient,
  parseGridHealth,
  viewportWindow,
  type GridHealthInfo,
  type GridSourceName,
} from './core/grid/online';
import { GRID_LAYER_DEFAULT_OPACITY } from './core/grid/palette';
import { parseGridDocument } from './core/grid/parse';
import { buildGridImageData } from './core/grid/texture';
import type { GridCellRef, GridLayerData, GridViewState } from './core/grid/types';
import { hitTestSnapshot } from './core/hit';
import type { InteractionMode } from './core/interaction';
import { DEFAULT_LAYERS, toggleLayer, type LayerName, type LayerState } from './core/layers';
import { parseSnapshot, SnapshotParseError } from './core/parse';
import { buildPlaybackOverlay, playbackDuration, playbackProfile } from './core/playback/overlay';
import { sha256HexBrowser } from './core/playback/hash';
import { loadPlaybackBundle } from './core/playback/load';
import { sampleTrajectoryAt } from './core/playback/time';
import {
  DEFAULT_PLAYBACK_LAYERS,
  type PlaybackBundle,
  type PlaybackLayers,
  type PlaybackSampleData,
} from './core/playback/types';
import { degToRad, normalizeYaw, poseFromLanelet, type Pose2d } from './core/pose';
import { PlanServiceClient, PlanServiceError, resolvePlanApiBase, resolvePlanWsBase, type PlanEventsState } from './core/plan/client';
import { parsePlanEvent } from './core/plan/parse';
import {
  createPlanSceneSnapshotState,
  PlanSceneSnapshotTracker,
  type PlanSceneSnapshotState,
} from './core/plan/scene_snapshot';
import { planCorridorsToOverlay, planResultToPlaybackSample } from './core/plan/convert';
import {
  acceptEvent,
  acceptJobUpdate,
  acceptSubmit,
  beginGeneration,
  createTrackerState,
  type JobTrackerState,
} from './core/plan/tracker';
import {
  DEFAULT_COMPARE_BACKENDS,
  DEFAULT_HYBRID_BACKEND,
  isTerminalStatus,
  KNOWN_HYBRID_BACKENDS,
  type PlanEvent,
  type PlanResult,
  type PlanSubmitRequest,
} from './core/plan/types';
import type {
  RouteState,
  SelectionInfo,
  ServiceStatus,
  TopologyKind,
  TopologyState,
  WindowQueryState,
} from './core/query_state';
import type { FootprintSpec } from './core/render';
import { chooseScaleBar } from './core/scalebar';
import {
  makeRectangleObstacle,
  moveObstacle,
  nextObstacleId,
  rectangleFromDrag,
  updateRectangleField,
} from './core/scene/obstacles';
import { buildScene, DEFAULT_ALGORITHM_PARAMS, DEFAULT_VEHICLE_REF, isSceneEmpty } from './core/scene/state';
import {
  checkSceneMap,
  sceneFromJson,
  serializeScene,
} from './core/scene/parse';
import {
  clearSceneStorage,
  loadSceneFromStorage,
  saveSceneToStorage,
  sceneFileName,
  type StorageLike,
} from './core/scene/storage';
import type { AlgorithmParams, SceneObstacle, WebScene } from './core/scene/types';
import { parseVehicleModel, type VehicleModel } from './core/scene/vehicle';
import { MapServiceClient, resolveApiBase } from './core/service/client';
import type { MapInfoPayload } from './core/service/types';
import { localFollowing, localPreceding } from './core/topology';
import { metersPerPixel, screenToWorld, worldToScreen, type Viewport } from './core/transform';
import type { BoundsM, Lanelet, MapSnapshot } from './core/types';
import { fitBounds, panBy, zoomAt } from './core/viewport';
import { filterSnapshotToWindow } from './core/window';

const DEFAULT_MAP_URL = '/maps/mapping_example.snapshot.json';
const DEFAULT_SOURCE_URL = '/maps/source.json';
const DEFAULT_VEHICLE_URL = '/fixtures/vehicles/default.json';
const VIEW_PADDING_PX = 24;
const DEFAULT_WINDOW_HALF_M = 50;
const AUTOSAVE_DEBOUNCE_MS = 400;
const DEFAULT_GRID_FIXTURE = 'snapshot_small.json';

export interface MapSourceInfo {
  rel_path: string;
  kind: string;
  sha256: string;
  bytes: number;
}

interface PlannerDebug {
  worldToScreen(x: number, y: number): [number, number] | null;
  screenToWorld(sx: number, sy: number): [number, number] | null;
  getState(): Record<string, unknown>;
  injectPlanEvent(event: unknown): { accepted: boolean; reason: string; stale_ignored: number } | null;
  /** X-W2：最近一次在线栅格响应原文（陈旧注入用例复用真实载荷）。 */
  gridPayload(source: GridSourceName): string | null;
  /** X-W2：把载荷送入与真实响应相同的验收管线（seq/来源/地图版本判定）。 */
  injectGridResponse(
    source: GridSourceName,
    payload: unknown,
  ): { decision: string; apply: boolean; reason: string; stale_ignored: number } | null;
}

declare global {
  interface Window {
    __plannerDebug?: PlannerDebug;
  }
}

function errorMessage(error: unknown): string {
  if (error instanceof SnapshotParseError) return `快照解析失败：${error.message}`;
  if (error instanceof Error) return error.message;
  return String(error);
}

async function sha256Hex(text: string): Promise<string> {
  if (!globalThis.crypto?.subtle) return '';
  const digest = await globalThis.crypto.subtle.digest('SHA-256', new TextEncoder().encode(text));
  return Array.from(new Uint8Array(digest), (byte) => byte.toString(16).padStart(2, '0')).join('');
}

/** 追加状态/阶段变迁记录（连续相同项去重；供进度展示与验收证据）。 */
function appendRunHistory(run: PlanRunView, status: string, stage: string): PlanRunView {
  const last = run.history[run.history.length - 1];
  if (last && last.status === status && last.stage === stage) return run;
  return { ...run, history: [...run.history, { status, stage, at_ms: Date.now() }] };
}

function selectionFromLanelet(
  lanelet: Lanelet,
  source: 'service' | 'local',
  geometry?: { left: number; right: number; centerline: number },
): SelectionInfo {
  return {
    kind: 'lanelet',
    id: lanelet.id,
    attributes: lanelet.attributes,
    geometry: geometry ?? {
      left: lanelet.left.length,
      right: lanelet.right.length,
      centerline: lanelet.centerline?.length ?? 0,
    },
    source,
  };
}

export function App() {
  const [snapshot, setSnapshot] = useState<MapSnapshot | null>(null);
  const [sourceInfo, setSourceInfo] = useState<MapSourceInfo | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);
  const [layers, setLayers] = useState<LayerState>({ ...DEFAULT_LAYERS });
  const [viewport, setViewport] = useState<Viewport | null>(null);
  const [canvasSize, setCanvasSize] = useState<CanvasSize | null>(null);
  const [cursorWorld, setCursorWorld] = useState<[number, number] | null>(null);
  const [dragOver, setDragOver] = useState(false);

  // W2 状态
  const [mode, setMode] = useState<InteractionMode>('select');
  const apiBase = useMemo(
    () => (typeof window !== 'undefined' ? resolveApiBase(window.location.search) : '/api'),
    [],
  );
  const client = useMemo(() => new MapServiceClient({ base_url: apiBase }), [apiBase]);
  const [service, setService] = useState<ServiceStatus>({ online: false, checked: false, base: apiBase });
  const [mapInfo, setMapInfo] = useState<MapInfoPayload | null>(null);
  const [selection, setSelection] = useState<SelectionInfo | null>(null);
  const [topology, setTopology] = useState<TopologyState | null>(null);
  const [routeState, setRouteState] = useState<RouteState>({
    from: '',
    to: '',
    loading: false,
    result: null,
    error: null,
  });
  const [startPose, setStartPose] = useState<Pose2d | null>(null);
  const [goalPose, setGoalPose] = useState<Pose2d | null>(null);
  const [windowState, setWindowState] = useState<WindowQueryState | null>(null);
  const [windowPreview, setWindowPreview] = useState<BoundsM | null>(null);
  const [busy, setBusy] = useState(false);
  const fittedSnapshotRef = useRef<MapSnapshot | null>(null);

  // 服务在线且地图版本与当前快照一致时才使用服务查询（回放/换图时自动回退本地）。
  const serviceMatchesMap =
    service.online &&
    snapshot !== null &&
    mapInfo !== null &&
    mapInfo.map_id === snapshot.map_id &&
    mapInfo.map_version === snapshot.map_version;

  // W3 场景编辑状态
  const [vehicle, setVehicle] = useState<VehicleModel | null>(null);
  const [vehicleSource, setVehicleSource] = useState<string | null>(null);
  const [vehicleError, setVehicleError] = useState<string | null>(null);
  const [showCursorFootprint, setShowCursorFootprint] = useState(false);
  const [obstacles, setObstacles] = useState<SceneObstacle[]>([]);
  const [selectedObstacleId, setSelectedObstacleId] = useState<string | null>(null);
  const [obstaclePreview, setObstaclePreview] = useState<BoundsM | null>(null);
  const [algorithmParams, setAlgorithmParams] = useState<AlgorithmParams>(() => ({
    hybrid_astar: { ...DEFAULT_ALGORITHM_PARAMS.hybrid_astar },
    corridor: { ...DEFAULT_ALGORITHM_PARAMS.corridor },
  }));
  const [autosaveStatus, setAutosaveStatus] = useState('未保存');
  const [mismatchWarning, setMismatchWarning] = useState<string | null>(null);
  const restoredSceneRef = useRef(false);
  const storage = useMemo<StorageLike | null>(
    () => (typeof window !== 'undefined' && window.localStorage ? window.localStorage : null),
    [],
  );

  // W4 回放状态
  const [playbackBundle, setPlaybackBundle] = useState<PlaybackBundle | null>(null);
  const [playbackLoading, setPlaybackLoading] = useState(false);
  const [playbackError, setPlaybackError] = useState<string | null>(null);
  const [playbackActive, setPlaybackActive] = useState(false);
  const [playbackSelectedId, setPlaybackSelectedId] = useState<string | null>(null);
  const [playbackLayers, setPlaybackLayers] = useState<PlaybackLayers>({ ...DEFAULT_PLAYBACK_LAYERS });
  const [playbackTime, setPlaybackTime] = useState(0);
  const [playbackPlaying, setPlaybackPlaying] = useState(false);
  const [playbackStaleWarning, setPlaybackStaleWarning] = useState<string | null>(null);

  // W5 规划服务状态
  const planApiBase = useMemo(
    () => (typeof window !== 'undefined' ? resolvePlanApiBase(window.location.search) : '/plan-api'),
    [],
  );
  const planWsBase = useMemo(
    () => (typeof window !== 'undefined' ? resolvePlanWsBase(window.location.search) : null),
    [],
  );
  const planClient = useMemo(
    () => new PlanServiceClient({ base_url: planApiBase, ws_base: planWsBase ?? undefined }),
    [planApiBase, planWsBase],
  );
  const [planWsState, setPlanWsState] = useState<PlanEventsState>('closed');
  const [planHealth, setPlanHealth] = useState<'unknown' | 'online' | 'offline'>('unknown');
  const [planRuns, setPlanRuns] = useState<Record<string, PlanRunView>>({});
  const [planActiveBackend, setPlanActiveBackend] = useState<string | null>(null);
  const [planTracker, setPlanTracker] = useState<JobTrackerState>(() => createTrackerState());
  const planTrackerRef = useRef<JobTrackerState>(planTracker);
  const [planError, setPlanError] = useState<string | null>(null);
  const [planBusy, setPlanBusy] = useState(false);
  const [planTime, setPlanTime] = useState(0);
  const [planPlaying, setPlanPlaying] = useState(false);
  const [planBackend, setPlanBackend] = useState(DEFAULT_HYBRID_BACKEND);
  const [planCompareA, setPlanCompareA] = useState(DEFAULT_COMPARE_BACKENDS[0] ?? DEFAULT_HYBRID_BACKEND);
  const [planCompareB, setPlanCompareB] = useState(DEFAULT_COMPARE_BACKENDS[1] ?? DEFAULT_HYBRID_BACKEND);
  const planRequestCounter = useRef(0);
  const planResultFetchedRef = useRef<Set<string>>(new Set());

  // X-W1 栅格夹具（离线显示档；合成数据，非算法正确性证据）
  const [gridCatalog, setGridCatalog] = useState<string[] | null>(null);
  const [gridCatalogError, setGridCatalogError] = useState<string | null>(null);
  const [gridFile, setGridFile] = useState(DEFAULT_GRID_FIXTURE);
  const [gridState, setGridState] = useState<GridViewState>({
    status: 'idle',
    layer: null,
    info: null,
    error: null,
  });
  const [gridEnabled, setGridEnabled] = useState(false);
  const [gridOpacity, setGridOpacity] = useState(GRID_LAYER_DEFAULT_OPACITY);
  const [gridPick, setGridPick] = useState(false);
  const [gridCell, setGridCell] = useState<GridCellRef | null>(null);
  const gridTrackerRef = useRef(new GridLoadTracker());
  const gridTextureRebuildsRef = useRef(0);
  const gridStateRef = useRef(gridState);
  const gridFileRef = useRef(gridFile);
  const viewportRef = useRef<Viewport | null>(null);
  gridStateRef.current = gridState;
  gridFileRef.current = gridFile;
  viewportRef.current = viewport;

  // X-W2 在线栅格（可行驶域 / 规划场景预览；来自服务，协议 §3/§6）
  const [gridSource, setGridSource] = useState<GridPanelSource>('fixture');
  const [gridOnlineResolution, setGridOnlineResolution] = useState(1.0);
  const [gridHealthInfo, setGridHealthInfo] = useState<GridHealthInfo | null>(null);
  const [gridHealthError, setGridHealthError] = useState<string | null>(null);
  const onlineClient = useMemo(() => new OnlineGridClient({ base_url: apiBase }), [apiBase]);
  const onlineTrackersRef = useRef<Record<GridSourceName, GridSourceTracker>>({
    driveable: new GridSourceTracker(),
    planning_scene: new GridSourceTracker(),
  });
  const emptyOnlineSourceView = (): GridOnlineSourceView => ({
    status: 'idle',
    layer: null,
    error: null,
    request_seq: null,
    requested_resolution_m: null,
    effective_resolution_m: null,
    resolution_m: null,
    degraded: false,
    display_adjusted: false,
    display_factor: 1,
    stale_ignored: 0,
    map_mismatch_ignored: 0,
    scene_id: null,
    bytes: null,
    fetched_at_ms: null,
  });
  const [onlineStates, setOnlineStates] = useState<Record<GridSourceName, GridOnlineSourceView>>({
    driveable: emptyOnlineSourceView(),
    planning_scene: emptyOnlineSourceView(),
  });
  const onlineStatesRef = useRef(onlineStates);
  onlineStatesRef.current = onlineStates;
  const gridSourceRef = useRef(gridSource);
  gridSourceRef.current = gridSource;
  /** 最近一次在线响应原文（调试注入陈旧载荷用）。 */
  const onlinePayloadsRef = useRef<Record<GridSourceName, string | null>>({
    driveable: null,
    planning_scene: null,
  });
  /** 当前画布实际绘制的栅格图层（夹具或在线；用于定位/拾取/证据）。 */
  const activeGridLayerRef = useRef<GridLayerData | null>(null);
  /** 在线请求入口（声明顺序解耦：供启用开关/重新加载回调调用）。 */
  const requestOnlineGridRef = useRef<((source: GridSourceName) => Promise<void>) | null>(null);

  // X-W2 规划实际场景快照（GET /plan/scene/{scene_id}）
  const [planSceneSnapshot, setPlanSceneSnapshot] = useState<PlanSceneSnapshotState>(() =>
    createPlanSceneSnapshotState(),
  );
  const planSceneSnapshotRef = useRef(planSceneSnapshot);
  planSceneSnapshotRef.current = planSceneSnapshot;
  const [planSceneOverlay, setPlanSceneOverlay] = useState(true);
  const planSceneTrackerRef = useRef(new PlanSceneSnapshotTracker());
  const [planHealthInfo, setPlanHealthInfo] = useState<GridHealthInfo | null>(null);

  const loadGridFixtureByFile = useCallback(async (file: string) => {
    const tracker = gridTrackerRef.current;
    const seq = tracker.begin();
    setGridState((previous) => ({ ...previous, status: 'loading', error: null }));
    try {
      const loaded = await loadGridFixture(
        (url) => fetch(url, { cache: 'no-store' }),
        file,
        (text) => sha256Hex(text).catch(() => ''),
      );
      const decision = tracker.accept(seq);
      if (decision === 'apply') {
        setGridState({ status: 'ready', layer: loaded.layer, info: loaded.info, error: null });
        setGridCell(null);
      } else if (decision === 'stale') {
        setNotice(`栅格夹具 ${file} 响应陈旧已丢弃（stale_ignored=${tracker.stale_ignored}）`);
      } else {
        setGridState({
          status: 'error',
          layer: null,
          info: null,
          error: `协议异常：响应 seq ${seq} > latest ${tracker.latest_seq}`,
        });
      }
    } catch (loadError) {
      const decision = tracker.accept(seq);
      if (decision === 'apply') {
        setGridState({ status: 'error', layer: null, info: null, error: errorMessage(loadError) });
      } else if (decision === 'stale') {
        setNotice(`栅格夹具 ${file} 失败响应陈旧已丢弃`);
      }
    }
  }, []);

  const handleToggleGridEnabled = useCallback(
    (next: boolean) => {
      setGridEnabled(next);
      const source = gridSourceRef.current;
      if (next && source === 'fixture' && gridStateRef.current.status === 'idle') {
        void loadGridFixtureByFile(gridFileRef.current);
      }
      if (next && source !== 'fixture' && onlineStatesRef.current[source].status === 'idle') {
        void requestOnlineGridRef.current?.(source);
      }
    },
    [loadGridFixtureByFile],
  );

  const handleSelectGridFile = useCallback(
    (file: string) => {
      setGridFile(file);
      setGridCell(null);
      void loadGridFixtureByFile(file);
    },
    [loadGridFixtureByFile],
  );

  const handleReloadGrid = useCallback(() => {
    const source = gridSourceRef.current;
    if (source === 'fixture') {
      void loadGridFixtureByFile(gridFileRef.current);
      return;
    }
    void requestOnlineGridRef.current?.(source);
  }, [loadGridFixtureByFile]);

  const handleGridTextureRebuilt = useCallback(() => {
    gridTextureRebuildsRef.current += 1;
  }, []);

  const gridDrawnAtRef = useRef(0);

  const handleGridDrawn = useCallback(() => {
    gridDrawnAtRef.current = performance.now();
  }, []);

  const handleClearGrid = useCallback(() => {
    const source = gridSourceRef.current;
    if (source === 'fixture') {
      setGridState({ status: 'idle', layer: null, info: null, error: null });
    } else {
      setOnlineStates((previous) => ({ ...previous, [source]: emptyOnlineSourceView() }));
      onlinePayloadsRef.current[source] = null;
    }
    setGridCell(null);
    setGridEnabled(false);
  }, []);

  const handleFitGrid = useCallback(() => {
    const layer = activeGridLayerRef.current;
    if (!layer || !canvasSize) return;
    setViewport(fitBounds(gridWorldBounds(layer), canvasSize.width_px, canvasSize.height_px, VIEW_PADDING_PX));
  }, [canvasSize]);

  // ---------------- X-W2：在线栅格（可行驶域 / 规划场景预览） ----------------

  const gridVersionCheck = useMemo(() => checkGridVersionConsistency(gridHealthInfo), [gridHealthInfo]);
  const gridHealthView = useMemo<GridHealthView>(
    () => ({
      status: service.online ? 'online' : service.checked ? 'offline' : 'unknown',
      protocol_version: gridHealthInfo?.grid_protocol_version ?? null,
      scene_rules_version: gridHealthInfo?.scene_rules_version ?? null,
      protocol_ok: service.online && gridVersionCheck.protocol_ok,
      scene_rules_ok: service.online && gridVersionCheck.scene_rules_ok,
      reason: !service.online
        ? '地图服务离线：无法校验协议版本（在线栅格禁用）'
        : gridHealthError ?? gridVersionCheck.reason,
      map_matches: serviceMatchesMap,
    }),
    [service.online, service.checked, gridHealthInfo, gridVersionCheck, gridHealthError, serviceMatchesMap],
  );
  const gridHealthViewRef = useRef(gridHealthView);
  gridHealthViewRef.current = gridHealthView;

  const onlineSourceAllowed = useCallback(
    (source: GridSourceName): boolean => {
      const health = gridHealthViewRef.current;
      if (!health.protocol_ok) return false;
      if (source === 'planning_scene' && !health.scene_rules_ok) return false;
      return true;
    },
    [],
  );

  /** 请求窗口 = 当前视口与地图边界的交集（无交集回退全图范围）。 */
  const computeOnlineWindow = useCallback((): BoundsM | null => {
    if (!snapshot) return null;
    const currentViewport = viewportRef.current;
    if (!currentViewport) return { ...snapshot.bounds_m };
    return viewportWindow(currentViewport, snapshot.bounds_m) ?? { ...snapshot.bounds_m };
  }, [snapshot]);

  const requestOnlineGrid = useCallback(
    async (source: GridSourceName) => {
      if (!snapshot) return;
      const health = gridHealthViewRef.current;
      const setSourceState = (
        updater: (previous: GridOnlineSourceView) => GridOnlineSourceView,
      ): void => {
        setOnlineStates((previous) => ({ ...previous, [source]: updater(previous[source]) }));
      };
      if (!onlineSourceAllowed(source)) {
        setSourceState((previous) => ({
          ...previous,
          status: 'disabled',
          layer: null,
          error: health.reason || '版本一致性检查未通过',
        }));
        return;
      }
      if (!serviceMatchesMap) {
        setSourceState((previous) => ({
          ...previous,
          status: 'error',
          layer: null,
          error: '地图版本不匹配：在线栅格请求被阻止（服务地图与当前快照不一致）',
        }));
        return;
      }
      const requestWindow = computeOnlineWindow();
      if (!requestWindow) {
        setSourceState((previous) => ({ ...previous, status: 'error', error: '无可用请求窗口（快照范围无效）' }));
        return;
      }
      const fit = fitDisplayResolution(requestWindow, gridOnlineResolution);
      if (!fit.within) {
        setSourceState((previous) => ({
          ...previous,
          status: 'error',
          layer: null,
          error: `请求窗口过大：显示上限 1024² 单元，2 的幂调整后仍为 ${fit.cells} 格`,
        }));
        return;
      }
      const tracker = onlineTrackersRef.current[source];
      const seq = tracker.begin();
      setSourceState((previous) => ({ ...previous, status: 'loading', error: null }));
      try {
        const request = { window: requestWindow, resolution_m: fit.resolution_m };
        const result =
          source === 'driveable'
            ? await onlineClient.driveable(request, seq)
            : await onlineClient.preview({ ...request, obstacles }, seq);
        const outcome = acceptOnlineGridResponse({
          tracker,
          seq,
          echoed_seq: result.layer.request_seq,
          source,
          layer: result.layer,
          map_id: snapshot.map_id,
          map_version: snapshot.map_version,
        });
        if (outcome.apply) {
          onlinePayloadsRef.current[source] = result.raw_text;
          setSourceState(() => ({
            status: 'ready',
            layer: result.layer,
            error: null,
            request_seq: result.layer.request_seq ?? seq,
            requested_resolution_m: result.layer.requested_resolution_m,
            effective_resolution_m: result.layer.effective_resolution_m,
            resolution_m: result.layer.resolution_m,
            degraded: result.layer.degraded,
            display_adjusted: fit.adjusted,
            display_factor: fit.factor,
            stale_ignored: tracker.stale_ignored,
            map_mismatch_ignored: tracker.map_mismatch_ignored,
            scene_id: result.layer.scene_id,
            bytes: result.bytes,
            fetched_at_ms: Date.now(),
          }));
          setGridCell(null);
        } else if (outcome.decision === 'stale') {
          setNotice(
            `在线栅格（${source}）响应陈旧已丢弃：seq ${seq} < latest ${tracker.latest_seq}（stale_ignored=${tracker.stale_ignored}）`,
          );
          setSourceState((previous) => ({ ...previous, stale_ignored: tracker.stale_ignored }));
        } else {
          setSourceState((previous) => ({
            ...previous,
            status: 'error',
            layer: null,
            error: outcome.reason,
            map_mismatch_ignored: tracker.map_mismatch_ignored,
          }));
          setNotice(`在线栅格（${source}）：${outcome.reason}`);
        }
      } catch (requestError) {
        const message =
          requestError instanceof Error ? requestError.message : String(requestError);
        setSourceState((previous) => ({ ...previous, status: 'error', layer: null, error: message }));
      }
    },
    [snapshot, serviceMatchesMap, gridOnlineResolution, obstacles, onlineClient, computeOnlineWindow, onlineSourceAllowed],
  );
  requestOnlineGridRef.current = requestOnlineGrid;

  const handleSelectGridSource = useCallback(
    (source: GridPanelSource) => {
      setGridSource(source);
      setGridCell(null);
      if (source === 'fixture') return;
      const sourceName: GridSourceName = source;
      const health = gridHealthViewRef.current;
      if (health.status === 'unknown') return; // 服务尚未检查：交由一致性 effect 决定
      if (!onlineSourceAllowed(sourceName)) {
        setOnlineStates((previous) => ({
          ...previous,
          [sourceName]: {
            ...previous[sourceName],
            status: 'disabled',
            layer: null,
            error: health.reason || '版本一致性检查未通过',
          },
        }));
        return;
      }
      if (onlineStatesRef.current[sourceName].status === 'idle') {
        void requestOnlineGrid(sourceName);
      }
    },
    [onlineSourceAllowed, requestOnlineGrid],
  );

  // 在线数据源自动请求（健康检查就绪后；已有图层不重复请求）。
  useEffect(() => {
    if (gridSource === 'fixture') return;
    const sourceName: GridSourceName = gridSource;
    const state = onlineStates[sourceName];
    if (state.status === 'idle' && onlineSourceAllowed(sourceName)) {
      void requestOnlineGrid(sourceName);
    }
  }, [gridSource, onlineStates, onlineSourceAllowed, requestOnlineGrid]);

  // 版本不匹配（协议主版本 / scene_rules_version）→ 禁用图层并显示可见状态；
  // 服务尚未检查（unknown）时不判定；从 disabled 恢复为可用时复位 idle 以便自动请求。
  useEffect(() => {
    if (gridSource === 'fixture') return;
    if (gridHealthView.status === 'unknown') return;
    const sourceName: GridSourceName = gridSource;
    if (onlineSourceAllowed(sourceName)) {
      setOnlineStates((previous) => {
        const current = previous[sourceName];
        if (current.status !== 'disabled') return previous;
        return { ...previous, [sourceName]: { ...current, status: 'idle', error: null } };
      });
      return;
    }
    const reason = gridHealthView.reason || '版本一致性检查未通过';
    setOnlineStates((previous) => {
      const current = previous[sourceName];
      if (current.status === 'disabled' && current.error === reason) return previous;
      return { ...previous, [sourceName]: { ...current, status: 'disabled', layer: null, error: reason } };
    });
  }, [gridSource, gridHealthView, onlineSourceAllowed]);

  const activeOnlineView: GridOnlineSourceView =
    gridSource === 'fixture' ? emptyOnlineSourceView() : onlineStates[gridSource];
  const activeGridLayer: GridLayerData | null =
    gridSource === 'fixture'
      ? gridState.layer
      : onlineSourceAllowed(gridSource as GridSourceName)
        ? onlineStates[gridSource as GridSourceName].layer
        : null;
  activeGridLayerRef.current = activeGridLayer;

  useEffect(() => {
    let cancelled = false;
    fetch(GRID_CATALOG_URL, { cache: 'no-store' })
      .then((response) => {
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        return response.json();
      })
      .then((value: unknown) => {
        if (cancelled) return;
        try {
          setGridCatalog(parseGridCatalog(value).files);
          setGridCatalogError(null);
        } catch (catalogError) {
          setGridCatalogError(errorMessage(catalogError));
        }
      })
      .catch((catalogError: unknown) => {
        if (!cancelled) setGridCatalogError(errorMessage(catalogError));
      });
    return () => {
      cancelled = true;
    };
  }, []);

  // 启动参数：`?grid=1` 自动启用并加载；`?gridFixture=<file>` 指定夹具；
  // `?gridSource=driveable|planning_scene` 选择在线数据源（X-W2）。
  useEffect(() => {
    const params = new URLSearchParams(window.location.search);
    const fixtureParam = params.get('gridFixture');
    const sourceParam = params.get('gridSource');
    const onlineSource =
      sourceParam === 'driveable' || sourceParam === 'planning_scene' ? sourceParam : null;
    const enabledParam = params.get('grid') === '1' || fixtureParam !== null || onlineSource !== null;
    if (fixtureParam) setGridFile(fixtureParam);
    if (onlineSource) setGridSource(onlineSource);
    if (enabledParam) {
      setGridEnabled(true);
      if (!onlineSource) {
        void loadGridFixtureByFile(fixtureParam ?? DEFAULT_GRID_FIXTURE);
      }
    }
  }, [loadGridFixtureByFile]);

  // 显示性能基准钩子（仅 `?gridBench=1` 暴露；测量协议见 benchmark_tiers §4.1）。
  useEffect(() => {
    const params = new URLSearchParams(window.location.search);
    if (params.get('gridBench') !== '1') return;
    const bench = {
      ready: true,
      loadFixture: async (file: string) => {
        const start = performance.now();
        await loadGridFixtureByFile(file);
        const loaded = performance.now();
        const drawn = await new Promise<boolean>((resolve) => {
          const deadline = performance.now() + 5000;
          const check = (): void => {
            if (gridDrawnAtRef.current >= loaded) resolve(true);
            else if (performance.now() > deadline) resolve(false);
            else window.requestAnimationFrame(check);
          };
          window.requestAnimationFrame(check);
        });
        return { file, load_ms: loaded - start, first_frame_ms: performance.now() - start, drawn };
      },
      hotRedraw: async (reps: number) => {
        const samples: number[] = [];
        for (let index = 0; index < reps; index += 1) {
          const start = performance.now();
          setViewport((previous) => {
            if (!previous) return previous;
            return zoomAt(
              previous,
              1 + 0.0004 * (index % 2 === 0 ? 1 : -1),
              previous.width_px / 2,
              previous.height_px / 2,
            );
          });
          const elapsed = await new Promise<number>((resolve) => {
            const check = (): void => {
              if (gridDrawnAtRef.current >= start) resolve(performance.now() - start);
              else window.requestAnimationFrame(check);
            };
            window.requestAnimationFrame(check);
          });
          samples.push(elapsed);
        }
        return { reps, samples_ms: samples };
      },
      runPipeline: async (file: string, reps: number) => {
        const perRep: Array<{ fetch_ms: number; parse_ms: number; texture_ms: number }> = [];
        let bytes = 0;
        let width = 0;
        let height = 0;
        for (let index = 0; index < reps; index += 1) {
          const t0 = performance.now();
          const response = await fetch(`${GRID_FIXTURE_URL_PREFIX}${file}`, { cache: 'no-store' });
          if (!response.ok) throw new Error(`HTTP ${response.status}`);
          const text = await response.text();
          const t1 = performance.now();
          const layer = parseGridDocument(JSON.parse(text) as unknown);
          const t2 = performance.now();
          const image = buildGridImageData(layer);
          const canvas = new OffscreenCanvas(layer.width, layer.height);
          const context = canvas.getContext('2d');
          if (!context) throw new Error('OffscreenCanvas 2d 上下文不可用');
          context.putImageData(
            new ImageData(image.data as ImageDataArray, image.width, image.height),
            0,
            0,
          );
          const t3 = performance.now();
          perRep.push({ fetch_ms: t1 - t0, parse_ms: t2 - t1, texture_ms: t3 - t2 });
          bytes = new TextEncoder().encode(text).byteLength;
          width = layer.width;
          height = layer.height;
        }
        return { file, reps, per_rep: perRep, bytes, width, height };
      },
      interact: async (durationMs: number) => {
        return await new Promise<{
          frames: number;
          frame_p50_ms: number;
          frame_p95_ms: number;
          long_frames: number;
          duration_ms: number;
        }>((resolve) => {
          const frames: number[] = [];
          const start = performance.now();
          let last = start;
          let phase = 0;
          const step = (now: number) => {
            frames.push(now - last);
            last = now;
            phase += 1;
            const current = viewportRef.current;
            const anchorX = current ? current.width_px / 2 : 0;
            const anchorY = current ? current.height_px / 2 : 0;
            setViewport((previous) => {
              if (!previous) return previous;
              const zoomed = zoomAt(previous, 1 + 0.0025 * Math.sin(phase / 30), anchorX, anchorY);
              return panBy(zoomed, Math.cos(phase / 45) * 1.5, Math.sin(phase / 45) * 1.5);
            });
            if (now - start < durationMs) {
              window.requestAnimationFrame(step);
              return;
            }
            const sorted = [...frames].sort((a, b) => a - b);
            const quantile = (q: number): number => {
              if (sorted.length === 0) return 0;
              const index = Math.min(sorted.length - 1, Math.max(0, Math.ceil(q * sorted.length) - 1));
              return sorted[index];
            };
            resolve({
              frames: frames.length,
              frame_p50_ms: quantile(0.5),
              frame_p95_ms: quantile(0.95),
              long_frames: frames.filter((value) => value > 50).length,
              duration_ms: now - start,
            });
          };
          window.requestAnimationFrame(step);
        });
      },
    };
    (window as unknown as { __plannerGridBench?: typeof bench }).__plannerGridBench = bench;
    return () => {
      delete (window as unknown as { __plannerGridBench?: typeof bench }).__plannerGridBench;
    };
  }, [loadGridFixtureByFile]);

  const applySnapshot = useCallback((next: MapSnapshot, source: MapSourceInfo) => {
    setSnapshot(next);
    setSourceInfo(source);
    setError(null);
    setSelection(null);
    setTopology(null);
    setWindowState(null);
    setWindowPreview(null);
    restoredSceneRef.current = false;
    // X-W2：底图变化（回放/换图）→ 在线栅格与规划快照失效重置。
    setOnlineStates({ driveable: emptyOnlineSourceView(), planning_scene: emptyOnlineSourceView() });
    onlinePayloadsRef.current = { driveable: null, planning_scene: null };
    setPlanSceneSnapshot(createPlanSceneSnapshotState());
    planSceneTrackerRef.current = new PlanSceneSnapshotTracker();
  }, []);

  const loadFromText = useCallback(
    async (text: string, label: string) => {
      try {
        const parsed = parseSnapshot(JSON.parse(text) as unknown);
        let hash = '';
        try {
          hash = await sha256Hex(text);
        } catch {
          hash = '';
        }
        applySnapshot(parsed, {
          rel_path: label,
          kind: 'local_file',
          sha256: hash,
          bytes: new TextEncoder().encode(text).byteLength,
        });
      } catch (loadError) {
        setError(errorMessage(loadError));
      }
    },
    [applySnapshot],
  );

  const loadDefaultMap = useCallback(async () => {
    const params = new URLSearchParams(window.location.search);
    const mapUrl = params.get('map') ?? DEFAULT_MAP_URL;
    const response = await fetch(mapUrl, { cache: 'no-store' });
    if (!response.ok) throw new Error(`加载 ${mapUrl} 失败：HTTP ${response.status}`);
    const value: unknown = await response.json();
    const parsed = parseSnapshot(value);
    applySnapshot(parsed, { rel_path: mapUrl, kind: 'server', sha256: '', bytes: 0 });
    if (mapUrl === DEFAULT_MAP_URL) {
      try {
        const sourceResponse = await fetch(DEFAULT_SOURCE_URL, { cache: 'no-store' });
        if (sourceResponse.ok) {
          setSourceInfo((await sourceResponse.json()) as MapSourceInfo);
        }
      } catch {
        // 来源描述是可选的展示信息。
      }
    }
  }, [applySnapshot]);

  useEffect(() => {
    let cancelled = false;
    void loadDefaultMap().catch((loadError: unknown) => {
      if (!cancelled) setError(errorMessage(loadError));
    });
    return () => {
      cancelled = true;
    };
  }, [loadDefaultMap]);

  // bounds 自动取景：仅在新快照加载或尚无视图时取景；画布尺寸变化只更新尺寸
  // （避免通知横幅/窗口尺寸变化触发视图跳变），手动缩放不受影响。
  useEffect(() => {
    if (!snapshot || !canvasSize) return;
    if (fittedSnapshotRef.current === snapshot && viewport) {
      if (viewport.width_px !== canvasSize.width_px || viewport.height_px !== canvasSize.height_px) {
        setViewport({ ...viewport, width_px: canvasSize.width_px, height_px: canvasSize.height_px });
      }
      return;
    }
    fittedSnapshotRef.current = snapshot;
    setViewport(fitBounds(snapshot.bounds_m, canvasSize.width_px, canvasSize.height_px, VIEW_PADDING_PX));
  }, [snapshot, canvasSize, viewport]);

  const handleSizeChange = useCallback((size: CanvasSize) => {
    setCanvasSize((previous) =>
      previous && previous.width_px === size.width_px && previous.height_px === size.height_px
        ? previous
        : size,
    );
  }, []);

  const handleResetView = useCallback(() => {
    if (!snapshot || !canvasSize) return;
    setViewport(fitBounds(snapshot.bounds_m, canvasSize.width_px, canvasSize.height_px, VIEW_PADDING_PX));
  }, [snapshot, canvasSize]);

  const handleToggleLayer = useCallback((name: LayerName) => {
    setLayers((previous) => toggleLayer(previous, name));
  }, []);

  const handleOpenFile = useCallback(
    (file: File) => {
      void file
        .text()
        .then((text) => loadFromText(text, file.name))
        .catch((readError: unknown) => setError(errorMessage(readError)));
    },
    [loadFromText],
  );

  const handleDrop = useCallback(
    (event: DragEvent<HTMLDivElement>) => {
      event.preventDefault();
      setDragOver(false);
      const file = event.dataTransfer.files.item(0);
      if (file) handleOpenFile(file);
    },
    [handleOpenFile],
  );

  // ---------------- W2：服务与查询 ----------------

  const refreshService = useCallback(async () => {
    setBusy(true);
    try {
      const info = await client.info();
      setMapInfo(info);
      setService({ online: true, checked: true, base: apiBase });
      try {
        const health = await client.health();
        setGridHealthInfo(parseGridHealth(health));
        setGridHealthError(null);
      } catch (healthError) {
        setGridHealthInfo(null);
        setGridHealthError(`地图服务版本字段读取失败：${errorMessage(healthError)}`);
      }
    } catch (serviceError) {
      setMapInfo(null);
      setGridHealthInfo(null);
      setGridHealthError(null);
      setService({ online: false, checked: true, base: apiBase, error: errorMessage(serviceError) });
    } finally {
      setBusy(false);
    }
  }, [client, apiBase]);

  useEffect(() => {
    void refreshService();
  }, [refreshService]);

  // 服务离线时每 10 秒复查一次（服务后启动时可自动接入）。
  useEffect(() => {
    if (service.online) return;
    const timer = window.setInterval(() => {
      void refreshService();
    }, 10000);
    return () => window.clearInterval(timer);
  }, [service.online, refreshService]);

  // ---------------- W3：车辆 / 场景编辑 ----------------

  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        const response = await fetch(DEFAULT_VEHICLE_URL, { cache: 'no-store' });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const model = parseVehicleModel(await response.json());
        if (cancelled) return;
        setVehicle(model);
        setVehicleSource(DEFAULT_VEHICLE_REF.path);
        setVehicleError(null);
      } catch (loadError) {
        if (!cancelled) setVehicleError(`车辆模型加载失败：${errorMessage(loadError)}`);
      }
    })();
    return () => {
      cancelled = true;
    };
  }, []);

  const currentScene = useCallback((): WebScene | null => {
    if (!snapshot) return null;
    return buildScene({
      map: snapshot,
      vehicle: DEFAULT_VEHICLE_REF,
      start: startPose,
      goal: goalPose,
      obstacles,
      algorithmParams,
    });
  }, [snapshot, startPose, goalPose, obstacles, algorithmParams]);

  const applyScene = useCallback((scene: WebScene, sourceLabel: string) => {
    setObstacles(scene.obstacles);
    setStartPose(scene.start);
    setGoalPose(scene.goal);
    setAlgorithmParams(scene.algorithm_params);
    setSelectedObstacleId(null);
    setMismatchWarning(null);
    setAutosaveStatus(`已恢复（${sourceLabel}）`);
  }, []);

  const restoreFromStorage = useCallback(() => {
    if (!snapshot || !storage) {
      setNotice('本地存储不可用');
      return;
    }
    const result = loadSceneFromStorage(storage);
    if (result.kind === 'none') {
      setNotice('本地没有已保存的场景');
      return;
    }
    if (result.kind === 'error') {
      setNotice(`本地场景读取失败：${result.error}`);
      return;
    }
    const check = checkSceneMap(result.scene, snapshot);
    if (!check.ok) {
      setMismatchWarning(`本地场景与当前地图不匹配，已拒绝套用：${check.reason}`);
      return;
    }
    applyScene(result.scene, 'localStorage');
    setNotice(`场景已恢复：${result.scene.obstacles.length} 个障碍`);
  }, [snapshot, storage, applyScene]);

  // 快照加载后自动恢复本地场景（版本匹配才套用；不匹配显式警告并拒绝）。
  // 回放合成地图（playback_synthetic）不参与场景恢复，避免误报警告。
  useEffect(() => {
    if (!snapshot || !storage || restoredSceneRef.current) return;
    restoredSceneRef.current = true;
    if (sourceInfo?.kind === 'playback_synthetic') return;
    const result = loadSceneFromStorage(storage);
    if (result.kind === 'none') return;
    if (result.kind === 'error') {
      setNotice(`本地场景读取失败：${result.error}`);
      return;
    }
    const check = checkSceneMap(result.scene, snapshot);
    if (!check.ok) {
      setMismatchWarning(`本地场景与当前地图不匹配，已拒绝套用：${check.reason}`);
      return;
    }
    applyScene(result.scene, 'localStorage');
  }, [snapshot, storage, applyScene, sourceInfo]);

  // localStorage 自动保存（去抖；空场景清除存储；回放合成地图不写场景）。
  useEffect(() => {
    if (!snapshot || !storage) return;
    if (sourceInfo?.kind === 'playback_synthetic') return;
    const scene = currentScene();
    if (!scene) return;
    const timer = window.setTimeout(() => {
      try {
        if (isSceneEmpty(scene)) {
          clearSceneStorage(storage);
          setAutosaveStatus('场景为空（未保存）');
        } else {
          saveSceneToStorage(storage, scene);
          setAutosaveStatus(`已自动保存 ${new Date().toLocaleTimeString('zh-CN', { hour12: false })}`);
        }
      } catch {
        setAutosaveStatus('自动保存失败');
      }
    }, AUTOSAVE_DEBOUNCE_MS);
    return () => window.clearTimeout(timer);
  }, [snapshot, storage, currentScene, sourceInfo]);

  const handleObstacleCreate = useCallback(
    (bounds: BoundsM | null, committed: boolean) => {
      if (!bounds) {
        setObstaclePreview(null);
        return;
      }
      if (!committed) {
        setObstaclePreview(bounds);
        return;
      }
      setObstaclePreview(null);
      const id = nextObstacleId(obstacles);
      const rect = rectangleFromDrag(id, bounds.min_x_m, bounds.min_y_m, bounds.max_x_m, bounds.max_y_m);
      if (!rect) {
        setNotice('障碍过小（< 0.5 m），未创建');
        return;
      }
      setObstacles((previous) => [...previous, rect]);
      setSelectedObstacleId(rect.id);
      setNotice(
        `已添加障碍 ${rect.id}：${(rect.x_max_m - rect.x_min_m).toFixed(1)} × ${(rect.y_max_m - rect.y_min_m).toFixed(1)} m`,
      );
    },
    [obstacles],
  );

  const handleObstacleMove = useCallback((id: string, dxM: number, dyM: number) => {
    setObstacles((previous) =>
      previous.map((obstacle) => (obstacle.id === id ? moveObstacle(obstacle, dxM, dyM) : obstacle)),
    );
  }, []);

  const handleObstacleClick = useCallback((id: string) => {
    setSelectedObstacleId(id);
  }, []);

  const handleUpdateObstacleRect = useCallback(
    (id: string, field: 'x_min_m' | 'y_min_m' | 'x_max_m' | 'y_max_m', value: number) => {
      setObstacles((previous) =>
        previous.map((obstacle) =>
          obstacle.id === id && obstacle.type === 'rectangle'
            ? updateRectangleField(obstacle, field, value)
            : obstacle,
        ),
      );
    },
    [],
  );

  const handleDeleteObstacle = useCallback((id: string) => {
    setObstacles((previous) => previous.filter((obstacle) => obstacle.id !== id));
    setSelectedObstacleId((previous) => (previous === id ? null : previous));
    setNotice(`已删除障碍 ${id}`);
  }, []);

  const handleAddObstacleAtCenter = useCallback(() => {
    if (!viewport) {
      setNotice('视图未就绪');
      return;
    }
    const [centerX, centerY] = screenToWorld(viewport, viewport.width_px / 2, viewport.height_px / 2);
    const id = nextObstacleId(obstacles);
    const rect = makeRectangleObstacle(id, centerX - 10, centerY - 5, centerX + 10, centerY + 5);
    setObstacles((previous) => [...previous, rect]);
    setSelectedObstacleId(id);
    setNotice(`已添加障碍 ${id}（视口中心 20 × 10 m）`);
  }, [viewport, obstacles]);

  const handleParamsChange = useCallback((params: AlgorithmParams) => {
    setAlgorithmParams(params);
  }, []);

  const handleExportScene = useCallback(() => {
    const scene = currentScene();
    if (!scene) return;
    try {
      const fileName = sceneFileName(scene);
      const blob = new Blob([serializeScene(scene)], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const anchor = document.createElement('a');
      anchor.href = url;
      anchor.download = fileName;
      document.body.appendChild(anchor);
      anchor.click();
      anchor.remove();
      URL.revokeObjectURL(url);
      setNotice(`场景已导出：${fileName}`);
    } catch (exportError) {
      setNotice(`场景导出失败：${errorMessage(exportError)}`);
    }
  }, [currentScene]);

  const handleImportScene = useCallback(
    async (file: File) => {
      if (!snapshot) return;
      try {
        const text = await file.text();
        const scene = sceneFromJson(text);
        const check = checkSceneMap(scene, snapshot);
        if (!check.ok) {
          setMismatchWarning(`导入场景与当前地图不匹配，已拒绝套用：${check.reason}`);
          return;
        }
        applyScene(scene, `文件 ${file.name}`);
        setNotice(`场景已导入：${scene.obstacles.length} 个障碍`);
      } catch (importError) {
        setNotice(`场景导入失败：${errorMessage(importError)}`);
      }
    },
    [snapshot, applyScene],
  );

  const handleClearScene = useCallback(() => {
    setObstacles([]);
    setStartPose(null);
    setGoalPose(null);
    setAlgorithmParams({
      hybrid_astar: { ...DEFAULT_ALGORITHM_PARAMS.hybrid_astar },
      corridor: { ...DEFAULT_ALGORITHM_PARAMS.corridor },
    });
    setSelectedObstacleId(null);
    setObstaclePreview(null);
    setMismatchWarning(null);
    if (storage) clearSceneStorage(storage);
    setAutosaveStatus('已清除');
    setNotice('场景已清除（地图数据未受影响）');
  }, [storage]);

  // ---------------- W4：回放样例 ----------------

  const playbackSample = useMemo<PlaybackSampleData | null>(
    () => playbackBundle?.samples.find((sample) => sample.sample.id === playbackSelectedId) ?? null,
    [playbackBundle, playbackSelectedId],
  );
  const playbackDurationS = playbackActive ? playbackDuration(playbackSample) : 0;
  const playbackCurrent = useMemo(
    () =>
      playbackSample?.kind === 'plan_result' && playbackSample.result.trajectory.length > 0
        ? sampleTrajectoryAt(playbackSample.result.trajectory, playbackTime)
        : null,
    [playbackSample, playbackTime],
  );
  const playbackProfiles = useMemo(() => {
    const profile = playbackActive ? playbackProfile(playbackSample) : [];
    return {
      velocity: profile.map((point) => ({ t_s: point.t_s, value: point.velocity_mps })),
      curvature: profile.map((point) => ({ t_s: point.t_s, value: point.curvature_inv_m })),
    };
  }, [playbackActive, playbackSample]);

  const handleLoadPlayback = useCallback(async () => {
    setPlaybackLoading(true);
    setPlaybackError(null);
    try {
      const indexResponse = await fetch('/fixtures/playback/index.json', { cache: 'no-store' });
      if (!indexResponse.ok) throw new Error(`加载回放索引失败：HTTP ${indexResponse.status}`);
      const indexBytes = new Uint8Array(await indexResponse.arrayBuffer());
      const bundle = await loadPlaybackBundle({
        index_bytes: indexBytes,
        load_file_bytes: async (file) => {
          const response = await fetch(`/fixtures/playback/${encodeURIComponent(file)}`, { cache: 'no-store' });
          if (!response.ok) throw new Error(`HTTP ${response.status}`);
          return new Uint8Array(await response.arrayBuffer());
        },
        hash_hex: sha256HexBrowser,
      });
      if (!bundle.map) throw new Error('回放包缺少小型地图快照样例');
      setPlaybackBundle(bundle);
      applySnapshot(bundle.map, {
        rel_path: `playback: ${bundle.map_file ?? 'map_snapshot_minimal.json'}`,
        kind: 'playback_synthetic',
        sha256: '',
        bytes: 0,
      });
      setPlaybackActive(true);
      setPlaybackSelectedId(bundle.samples[0]?.sample.id ?? null);
      setPlaybackTime(0);
      setPlaybackPlaying(false);
      setPlaybackStaleWarning(null);
      setNotice(
        `回放样例已加载：${bundle.samples.length} 个通过 SHA256 校验${bundle.issues.length > 0 ? `，${bundle.issues.length} 项被拒绝` : ''}（合成显示样例）`,
      );
    } catch (loadError) {
      setPlaybackError(errorMessage(loadError));
    } finally {
      setPlaybackLoading(false);
    }
  }, [applySnapshot]);

  const handleExitPlayback = useCallback(async () => {
    setPlaybackActive(false);
    setPlaybackPlaying(false);
    setPlaybackSelectedId(null);
    setPlaybackStaleWarning(null);
    setPlaybackTime(0);
    try {
      await loadDefaultMap();
      setNotice('已退出回放，恢复默认地图');
    } catch (loadError) {
      setError(errorMessage(loadError));
    }
  }, [loadDefaultMap]);

  const handleSelectPlaybackSample = useCallback(
    (id: string) => {
      setPlaybackSelectedId(id);
      setPlaybackTime(0);
      setPlaybackPlaying(false);
      const sample = playbackBundle?.samples.find((entry) => entry.sample.id === id);
      if (sample?.kind === 'stale_result') {
        setPlaybackStaleWarning(
          `过期结果：map_version ${sample.stale.map_version} ≠ 当前 ${sample.stale.current_map_version}；` +
            `request_seq ${sample.stale.request_seq} < ${sample.stale.current_request_seq}；结果图层已忽略（不得覆盖当前结果）`,
        );
      } else {
        setPlaybackStaleWarning(null);
      }
    },
    [playbackBundle],
  );

  const handleTogglePlaybackLayer = useCallback((name: keyof PlaybackLayers) => {
    setPlaybackLayers((previous) => ({ ...previous, [name]: !previous[name] }));
  }, []);

  const handlePlaybackSeek = useCallback((t: number) => {
    setPlaybackTime(Math.max(0, t));
  }, []);

  const handlePlaybackPlayPause = useCallback(() => {
    if (playbackDurationS <= 0) return;
    if (!playbackPlaying && playbackTime >= playbackDurationS) {
      setPlaybackTime(0);
      setPlaybackPlaying(true);
      return;
    }
    setPlaybackPlaying((previous) => !previous);
  }, [playbackDurationS, playbackTime, playbackPlaying]);

  // 回放动画（rAF；到达终点自动暂停）。
  useEffect(() => {
    if (!playbackPlaying || playbackDurationS <= 0) return;
    let raf = 0;
    let last = performance.now();
    const step = (now: number) => {
      const dt = Math.min(0.25, (now - last) / 1000);
      last = now;
      setPlaybackTime((previous) => Math.min(playbackDurationS, previous + dt));
      raf = window.requestAnimationFrame(step);
    };
    raf = window.requestAnimationFrame(step);
    return () => window.cancelAnimationFrame(raf);
  }, [playbackPlaying, playbackDurationS]);

  useEffect(() => {
    if (playbackPlaying && playbackDurationS > 0 && playbackTime >= playbackDurationS) {
      setPlaybackPlaying(false);
    }
  }, [playbackPlaying, playbackTime, playbackDurationS]);

  // ---------------- W5：规划服务（提交/取消/状态/结果 + 陈旧防护） ----------------

  const updateTracker = useCallback((state: JobTrackerState) => {
    planTrackerRef.current = state;
    setPlanTracker(state);
  }, []);

  const refreshPlanHealth = useCallback(async () => {
    try {
      const health = await planClient.health();
      setPlanHealth('online');
      try {
        setPlanHealthInfo(parseGridHealth(health));
      } catch {
        setPlanHealthInfo(null);
      }
    } catch {
      setPlanHealth('offline');
      setPlanHealthInfo(null);
    }
  }, [planClient]);

  useEffect(() => {
    void refreshPlanHealth();
  }, [refreshPlanHealth]);

  useEffect(() => {
    if (planHealth === 'online') return;
    const timer = window.setInterval(() => {
      void refreshPlanHealth();
    }, 10000);
    return () => window.clearInterval(timer);
  }, [planHealth, refreshPlanHealth]);

  const planSceneSummary = useMemo(
    () => ({
      map_id: snapshot?.map_id ?? '',
      map_version: snapshot?.map_version ?? '',
      map_matches: serviceMatchesMap,
      has_poses: startPose !== null && goalPose !== null,
      obstacle_count: obstacles.length,
      vehicle_ref: vehicle?.id ?? DEFAULT_VEHICLE_REF.ref,
    }),
    [snapshot, serviceMatchesMap, startPose, goalPose, obstacles, vehicle],
  );

  const applyPlanStatus = useCallback(
    (backend: string, jobId: string, status: { status: string; stage?: string; reason?: string; scene_id?: string; scene_version?: string }) => {
      setPlanRuns((previous) => {
        const run = previous[backend];
        if (!run || run.job_id !== jobId) return previous; // 旧任务迟到响应不得覆盖
        return {
          ...previous,
          [backend]: appendRunHistory(
            {
              ...run,
              status: status.status,
              stage: status.stage ?? '',
              reason: status.reason ?? run.reason,
              scene_id: status.scene_id ?? run.scene_id,
              scene_version: status.scene_version ?? run.scene_version,
            },
            status.status,
            status.stage ?? '',
          ),
        };
      });
    },
    [],
  );

  const applyPlanResult = useCallback((backend: string, jobId: string, result: PlanResult) => {
    setPlanRuns((previous) => {
      const run = previous[backend];
      if (!run || run.job_id !== jobId) return previous;
      return {
        ...previous,
        [backend]: appendRunHistory(
          {
            ...run,
            status: result.status,
            stage: '',
            reason: result.reason,
            result,
            scene_id: result.scene_id ?? run.scene_id,
            scene_version: result.scene_version ?? run.scene_version,
          },
          result.status,
          '',
        ),
      };
    });
  }, []);

  const submitPlanToBackend = useCallback(
    async (backend: string): Promise<boolean> => {
      if (!snapshot || !startPose || !goalPose || !serviceMatchesMap) return false;
      planRequestCounter.current += 1;
      const request: PlanSubmitRequest = {
        request_id: `web-${Date.now().toString(36)}-${planRequestCounter.current}`,
        map_id: snapshot.map_id,
        map_version: snapshot.map_version,
        start: startPose,
        goal: goalPose,
        vehicle_ref: vehicleSource ?? DEFAULT_VEHICLE_REF.path,
        obstacles,
        algorithm_params: {
          hybrid_backend: backend,
          time_budget_s: algorithmParams.hybrid_astar.time_budget_s,
          node_budget: algorithmParams.hybrid_astar.node_budget,
          // 真实服务 algorithm_params.corridor 使用 CorridorOptions 键（clearance_m）；
          // 场景 margin_m 映射为走廊最小间隙。
          corridor: { clearance_m: algorithmParams.corridor.margin_m },
        },
      };
      try {
        const submit = await planClient.submit(request);
        const outcome = acceptSubmit(planTrackerRef.current, submit);
        updateTracker(outcome.state);
        if (!outcome.accepted) {
          setPlanError(outcome.reason);
          return false;
        }
        setPlanRuns((previous) => ({
          ...previous,
          [backend]: {
            backend,
            request_id: submit.request_id,
            job_id: submit.job_id,
            job_seq: submit.job_seq,
            status: submit.status,
            stage: '',
            reason: '',
            error: null,
            result: null,
            scene_id: null,
            scene_version: null,
            history: [{ status: submit.status, stage: '', at_ms: Date.now() }],
          },
        }));
        setPlanActiveBackend(backend);
        return true;
      } catch (error) {
        const message =
          error instanceof PlanServiceError && error.isStaleMap
            ? `地图版本不匹配（stale_map）：提交被拒绝 —— ${error.reason ?? ''}`
            : errorMessage(error);
        setPlanRuns((previous) => ({
          ...previous,
          [backend]: {
            backend,
            request_id: request.request_id,
            job_id: '',
            job_seq: 0,
            status: 'error',
            stage: '',
            reason: '',
            error: message,
            result: null,
            scene_id: null,
            scene_version: null,
            history: [{ status: 'error', stage: '', at_ms: Date.now() }],
          },
        }));
        setPlanActiveBackend(backend);
        setPlanError(message);
        return false;
      }
    },
    [snapshot, startPose, goalPose, serviceMatchesMap, vehicle, obstacles, algorithmParams, planClient, updateTracker],
  );

  const handlePlanSubmit = useCallback(
    async (mode: 'single' | 'compare') => {
      if (!snapshot || !startPose || !goalPose) {
        setPlanError('需要起终位姿（使用“设起点/设终点”模式在画布上设置）');
        return;
      }
      if (!serviceMatchesMap) {
        setPlanError('地图版本不匹配：提交被阻止（服务地图与当前快照不一致）');
        return;
      }
      if (planHealth !== 'online') {
        setPlanError('规划服务离线，无法提交');
        return;
      }
      const backends = mode === 'compare' ? [planCompareA, planCompareB] : [planBackend];
      setPlanBusy(true);
      setPlanError(null);
      // 新提交：开启新代际并清空旧结果（陈旧防护）。
      updateTracker(beginGeneration(planTrackerRef.current));
      planResultFetchedRef.current.clear();
      setPlanRuns({});
      setPlanActiveBackend(null);
      setPlanTime(0);
      setPlanPlaying(false);
      // X-W2：旧规划实际场景随新代际清除（历史场景不覆盖新任务）。
      setPlanSceneSnapshot(createPlanSceneSnapshotState());
      try {
        for (const backend of backends) {
          await submitPlanToBackend(backend);
        }
      } finally {
        setPlanBusy(false);
      }
    },
    [snapshot, startPose, goalPose, serviceMatchesMap, planHealth, planBackend, planCompareA, planCompareB, submitPlanToBackend, updateTracker],
  );

  const handlePlanCancel = useCallback(async () => {
    const run = planActiveBackend ? planRuns[planActiveBackend] : undefined;
    if (!run || !run.job_id) return;
    try {
      const status = await planClient.cancel(run.job_id);
      const outcome = acceptJobUpdate(planTrackerRef.current, run.job_id);
      updateTracker(outcome.state);
      if (!outcome.accepted) {
        setPlanError(outcome.reason);
        return;
      }
      applyPlanStatus(run.backend, run.job_id, {
        status: status.status || 'cancelled',
        stage: status.stage,
        reason: status.reason,
      });
    } catch (error) {
      setPlanError(errorMessage(error));
    }
  }, [planActiveBackend, planRuns, planClient, applyPlanStatus, updateTracker]);

  const handlePlanClear = useCallback(() => {
    setPlanRuns({});
    setPlanActiveBackend(null);
    setPlanError(null);
    setPlanTime(0);
    setPlanPlaying(false);
    planResultFetchedRef.current.clear();
    updateTracker(beginGeneration(planTrackerRef.current));
    setPlanSceneSnapshot(createPlanSceneSnapshotState());
  }, [updateTracker]);

  // 状态轮询（仅当前代际活动任务；终态结果由下方 effect 统一拉取；旧任务响应由 tracker 丢弃）。
  useEffect(() => {
    const pending = Object.values(planRuns).filter(
      (run) => run.job_id && !run.error && !isTerminalStatus(run.status),
    );
    if (pending.length === 0) return;
    const timer = window.setInterval(() => {
      for (const run of pending) {
        void (async () => {
          try {
            const status = await planClient.status(run.job_id);
            const outcome = acceptJobUpdate(planTrackerRef.current, run.job_id, status.job_seq);
            updateTracker(outcome.state);
            if (!outcome.accepted) return;
            applyPlanStatus(run.backend, run.job_id, status);
          } catch (error) {
            setPlanError(errorMessage(error));
          }
        })();
      }
    }, 800);
    return () => {
      window.clearInterval(timer);
    };
  }, [planRuns, planClient, applyPlanStatus, updateTracker]);

  // 终态结果获取：事件推送只带状态（job_result 无几何），结果经 HTTP 拉取；每个 job 仅一次。
  useEffect(() => {
    const pendingResults = Object.values(planRuns).filter(
      (run) =>
        run.job_id &&
        !run.error &&
        run.result === null &&
        isTerminalStatus(run.status) &&
        !planResultFetchedRef.current.has(run.job_id),
    );
    for (const run of pendingResults) {
      planResultFetchedRef.current.add(run.job_id);
      void (async () => {
        try {
          const result = await planClient.result(run.job_id);
          const outcome = acceptJobUpdate(planTrackerRef.current, run.job_id);
          updateTracker(outcome.state);
          if (!outcome.accepted) return;
          applyPlanResult(run.backend, run.job_id, result);
        } catch (error) {
          planResultFetchedRef.current.delete(run.job_id); // 允许后续重试
          setPlanError(errorMessage(error));
        }
      })();
    }
  }, [planRuns, planClient, applyPlanResult, updateTracker]);

  // 推送事件注入（WS 推送或测试）：经 tracker 陈旧判定后应用。
  const injectPlanEvent = useCallback(
    (event: PlanEvent): { accepted: boolean; reason: string; stale_ignored: number } => {
      const outcome = acceptEvent(planTrackerRef.current, event);
      updateTracker(outcome.state);
      if (outcome.accepted) {
        setPlanRuns((previous) => {
          const entry = Object.values(previous).find((run) => run.job_id === event.job_id);
          if (!entry) return previous;
          const status = event.status ?? entry.status;
          const stage = event.stage ?? entry.stage;
          const updated: PlanRunView = appendRunHistory(
            {
              ...entry,
              status,
              stage,
              reason: event.reason ?? entry.reason,
              result: event.result ?? entry.result,
              scene_id: event.result?.scene_id ?? event.scene_id ?? entry.scene_id,
              scene_version: event.result?.scene_version ?? entry.scene_version,
            },
            status,
            stage,
          );
          return { ...previous, [entry.backend]: updated };
        });
      }
      return { accepted: outcome.accepted, reason: outcome.reason, stale_ignored: outcome.state.stale_ignored };
    },
    [updateTracker],
  );

  // /plan/events 推送订阅（WebSocket；job_update/job_result 经 tracker 陈旧判定后应用）。
  useEffect(() => {
    if (planHealth !== 'online') return;
    const handle = planClient.connectEvents(
      (event) => {
        injectPlanEvent(event);
      },
      (state) => setPlanWsState(state),
    );
    return () => handle.close();
  }, [planHealth, planClient, injectPlanEvent]);

  const planActiveRun = planActiveBackend ? planRuns[planActiveBackend] : undefined;
  const planResult = planActiveRun?.result ?? null;
  const planResultMapMatches = useMemo(
    () =>
      planResult !== null &&
      snapshot !== null &&
      planResult.map_id === snapshot.map_id &&
      planResult.map_version === snapshot.map_version,
    [planResult, snapshot],
  );

  // ---------------- X-W2：规划实际场景快照（GET /plan/scene/{scene_id}） ----------------

  const planSceneId = planActiveRun?.scene_id ?? null;
  const planSceneVersion = planActiveRun?.scene_version ?? null;
  const planSceneRulesVersion = planHealthInfo?.scene_rules_version ?? null;
  const planProtocolCheck = useMemo(() => checkGridVersionConsistency(planHealthInfo), [planHealthInfo]);
  const planSceneProtocolOk = planHealth === 'online' ? planProtocolCheck.protocol_ok : false;
  const sceneRulesMismatch = useMemo(() => {
    const mapVersion = gridHealthInfo?.scene_rules_version ?? null;
    if (mapVersion === null || planSceneRulesVersion === null) return null;
    if (mapVersion === planSceneRulesVersion) return null;
    return (
      `地图服务 scene_rules_version=${mapVersion} ≠ 规划服务 scene_rules_version=${planSceneRulesVersion}；` +
      '预览与规划快照不保证逐格一致，禁止把当前预览当作规划输入证据'
    );
  }, [gridHealthInfo, planSceneRulesVersion]);

  const fetchPlanSceneSnapshot = useCallback(
    async (sceneId: string, source: { job_id: string | null; backend: string | null; scene_version: string | null }) => {
      const tracker = planSceneTrackerRef.current;
      tracker.request(sceneId);
      setPlanSceneSnapshot((previous) => ({
        ...previous,
        status: 'loading',
        scene_id: sceneId,
        scene_version: source.scene_version,
        layer: null,
        error: null,
        job_id: source.job_id,
        backend: source.backend,
      }));
      try {
        const layer = await planClient.scene(sceneId);
        if (!tracker.accept(sceneId)) {
          setNotice(
            `规划场景快照响应陈旧已丢弃（scene_id ${sceneId.slice(0, 12)}… ≠ 最新 ${String(tracker.latest_scene_id).slice(0, 12)}…）`,
          );
          return;
        }
        setPlanSceneSnapshot({
          status: 'ready',
          scene_id: sceneId,
          scene_version: layer.scene_version ?? source.scene_version,
          layer,
          error: null,
          job_id: source.job_id,
          backend: source.backend,
          fetched_at_ms: Date.now(),
        });
      } catch (snapshotError) {
        if (!tracker.accept(sceneId)) return;
        const message = errorMessage(snapshotError);
        const notFound = snapshotError instanceof PlanServiceError && snapshotError.isUnknownScene;
        setPlanSceneSnapshot({
          status: notFound ? 'not_found' : 'error',
          scene_id: sceneId,
          scene_version: source.scene_version,
          layer: null,
          error: notFound
            ? '规划服务未保留该场景（未知或已淘汰：404 unknown_scene）'
            : message,
          job_id: source.job_id,
          backend: source.backend,
          fetched_at_ms: Date.now(),
        });
      }
    },
    [planClient],
  );

  // 规划结果/状态携带 scene_id → 自动拉取；历史场景不覆盖当前预览（独立状态）。
  useEffect(() => {
    if (!planSceneId || planHealth !== 'online') return;
    if (!planSceneProtocolOk) {
      setPlanSceneSnapshot((previous) => ({
        ...previous,
        status: 'disabled',
        scene_id: planSceneId,
        scene_version: planSceneVersion,
        error: planProtocolCheck.reason || '协议版本一致性检查未通过',
      }));
      return;
    }
    const current = planSceneSnapshotRef.current;
    if (
      current.scene_id === planSceneId &&
      (current.status === 'ready' || current.status === 'loading' || current.status === 'not_found')
    ) {
      return;
    }
    void fetchPlanSceneSnapshot(planSceneId, {
      job_id: planActiveRun?.job_id ?? null,
      backend: planActiveRun?.backend ?? null,
      scene_version: planSceneVersion,
    });
  }, [
    planSceneId,
    planSceneVersion,
    planHealth,
    planSceneProtocolOk,
    planProtocolCheck.reason,
    planActiveRun?.job_id,
    planActiveRun?.backend,
    fetchPlanSceneSnapshot,
  ]);

  const planSceneMapMatches =
    planSceneSnapshot.layer !== null &&
    snapshot !== null &&
    planSceneSnapshot.layer.map_id === snapshot.map_id &&
    planSceneSnapshot.layer.map_version === snapshot.map_version;
  const planSceneOverlayLayer =
    planSceneOverlay && planSceneSnapshot.status === 'ready' && planSceneMapMatches
      ? planSceneSnapshot.layer
      : null;
  const previewSceneId = onlineStates.planning_scene.layer?.scene_id ?? null;

  const handleFitPlanScene = useCallback(() => {
    const layer = planSceneSnapshotRef.current.layer;
    if (!layer || !canvasSize) return;
    setViewport(fitBounds(gridWorldBounds(layer), canvasSize.width_px, canvasSize.height_px, VIEW_PADDING_PX));
  }, [canvasSize]);

  const handleClearPlanScene = useCallback(() => {
    setPlanSceneSnapshot(createPlanSceneSnapshotState());
  }, []);

  const planPlaybackSample = useMemo(
    () => (planResult && planResultMapMatches ? planResultToPlaybackSample(planResult) : null),
    [planResult, planResultMapMatches],
  );
  const planDurationS = planPlaybackSample ? playbackDuration(planPlaybackSample) : 0;
  const planCurrent = useMemo(
    () =>
      planPlaybackSample?.kind === 'plan_result' && planPlaybackSample.result.trajectory.length > 0
        ? sampleTrajectoryAt(planPlaybackSample.result.trajectory, planTime)
        : null,
    [planPlaybackSample, planTime],
  );
  const planProfiles = useMemo(() => {
    const profile = planPlaybackSample ? playbackProfile(planPlaybackSample) : [];
    return {
      velocity: profile.map((point) => ({ t_s: point.t_s, value: point.velocity_mps })),
      curvature: profile.map((point) => ({ t_s: point.t_s, value: point.curvature_inv_m })),
    };
  }, [planPlaybackSample]);
  const planOverlay = useMemo(() => {
    if (!planPlaybackSample || !planResult) return null;
    return buildPlaybackOverlay(planPlaybackSample, playbackLayers, planTime, {
      corridors: planCorridorsToOverlay(planResult),
    });
  }, [planPlaybackSample, planResult, playbackLayers, planTime]);

  useEffect(() => {
    if (!planPlaying || planDurationS <= 0) return;
    let raf = 0;
    let last = performance.now();
    const step = (now: number) => {
      const dt = Math.min(0.25, (now - last) / 1000);
      last = now;
      setPlanTime((previous) => Math.min(planDurationS, previous + dt));
      raf = window.requestAnimationFrame(step);
    };
    raf = window.requestAnimationFrame(step);
    return () => window.cancelAnimationFrame(raf);
  }, [planPlaying, planDurationS]);

  useEffect(() => {
    if (planPlaying && planDurationS > 0 && planTime >= planDurationS) {
      setPlanPlaying(false);
    }
  }, [planPlaying, planTime, planDurationS]);

  const handlePlanPlayPause = useCallback(() => {
    if (planDurationS <= 0) return;
    if (!planPlaying && planTime >= planDurationS) {
      setPlanTime(0);
      setPlanPlaying(true);
      return;
    }
    setPlanPlaying((previous) => !previous);
  }, [planDurationS, planTime, planPlaying]);

  // 服务在线且地图版本与当前快照一致时才使用服务查询（回放/换图时自动回退本地）。
  const selectLanelet = useCallback(
    async (id: string) => {
      if (!snapshot) return;
      setTopology(null);
      const lanelet = snapshot.lanelets.find((item) => item.id === id) ?? null;
      if (serviceMatchesMap) {
        setBusy(true);
        try {
          const geometry = await client.lanelet(id);
          setSelection({
            kind: 'lanelet',
            id,
            attributes: geometry.attributes,
            geometry: {
              left: geometry.left.length,
              right: geometry.right.length,
              centerline: geometry.centerline.length,
            },
            source: 'service',
          });
          return;
        } catch (serviceError) {
          setNotice(`lanelet ${id} 服务查询失败：${errorMessage(serviceError)}；使用本地快照`);
        } finally {
          setBusy(false);
        }
      }
      if (lanelet) setSelection(selectionFromLanelet(lanelet, 'local'));
      else setSelection(null);
    },
    [snapshot, serviceMatchesMap, client],
  );

  const selectArea = useCallback(
    async (id: string) => {
      if (!snapshot) return;
      setTopology(null);
      const area = snapshot.areas.find((item) => item.id === id) ?? null;
      if (serviceMatchesMap) {
        setBusy(true);
        try {
          const geometry = await client.area(id);
          const innerPoints = geometry.inner.reduce((sum, ring) => sum + ring.length, 0);
          setSelection({
            kind: 'area',
            id,
            attributes: geometry.attributes,
            geometry: { outer: geometry.outer.length, inner_rings: geometry.inner.length, inner_points: innerPoints },
            source: 'service',
          });
          return;
        } catch (serviceError) {
          setNotice(`区域 ${id} 服务查询失败：${errorMessage(serviceError)}；使用本地快照`);
        } finally {
          setBusy(false);
        }
      }
      if (area) {
        const innerPoints = area.inner.reduce((sum, ring) => sum + ring.length, 0);
        setSelection({
          kind: 'area',
          id,
          attributes: area.attributes,
          geometry: { outer: area.outer.length, inner_rings: area.inner.length, inner_points: innerPoints },
          source: 'local',
        });
      } else {
        setSelection(null);
      }
    },
    [snapshot, serviceMatchesMap, client],
  );

  const selectAt = useCallback(
    async (x: number, y: number) => {
      if (!snapshot) return;
      if (serviceMatchesMap) {
        try {
          const result = await client.queryPoint(x, y);
          const laneletId = result.lanelets[0]?.id;
          const areaId = result.areas[0]?.id;
          if (laneletId) {
            await selectLanelet(laneletId);
            return;
          }
          if (areaId) {
            await selectArea(areaId);
            return;
          }
          setSelection(null);
          setTopology(null);
          setNotice(`点 (${x.toFixed(2)}, ${y.toFixed(2)}) m 未命中对象（服务查询）`);
          return;
        } catch (serviceError) {
          setNotice(`点查询失败：${errorMessage(serviceError)}；回退本地命中`);
        }
      }
      const local = hitTestSnapshot(snapshot, x, y);
      if (local.lanelet_ids.length > 0) {
        await selectLanelet(local.lanelet_ids[0]);
        return;
      }
      if (local.area_ids.length > 0) {
        await selectArea(local.area_ids[0]);
        return;
      }
      setSelection(null);
      setTopology(null);
      setNotice(`点 (${x.toFixed(2)}, ${y.toFixed(2)}) m 未命中对象（本地命中）`);
    },
    [snapshot, serviceMatchesMap, client, selectLanelet, selectArea],
  );

  const setPoseAt = useCallback(
    (kind: 'start' | 'goal', x: number, y: number) => {
      let lanelet: Lanelet | null = null;
      if (snapshot) {
        const local = hitTestSnapshot(snapshot, x, y);
        if (local.lanelet_ids.length > 0) {
          lanelet = snapshot.lanelets.find((item) => item.id === local.lanelet_ids[0]) ?? null;
        }
      }
      const pose = poseFromLanelet(lanelet, x, y, kind === 'start' ? 0 : Math.PI);
      if (kind === 'start') setStartPose(pose);
      else setGoalPose(pose);
      if (lanelet) {
        const laneletId = lanelet.id;
        setRouteState((previous) =>
          kind === 'start' ? { ...previous, from: laneletId } : { ...previous, to: laneletId },
        );
      }
      setNotice(
        `${kind === 'start' ? '起点' : '终点'}已设置：(${x.toFixed(2)}, ${y.toFixed(2)}) m${lanelet ? ` · lanelet ${lanelet.id}` : ''}`,
      );
    },
    [snapshot],
  );

  const handleCanvasClick = useCallback(
    (world: [number, number]) => {
      const [x, y] = world;
      const gridLayer = activeGridLayerRef.current;
      if (gridPick && gridLayer) {
        const hit = worldToCell(gridLayer, x, y);
        if (hit.in_bounds) {
          setGridCell({ row: hit.row, col: hit.col });
        } else {
          setGridCell(null);
          setNotice('点击位置在栅格范围外（不静默夹取）');
        }
        return;
      }
      if (mode === 'set_start') {
        setPoseAt('start', x, y);
        return;
      }
      if (mode === 'set_goal') {
        setPoseAt('goal', x, y);
        return;
      }
      if (mode === 'select') void selectAt(x, y);
    },
    [mode, setPoseAt, selectAt, gridPick],
  );

  const handleHeadingDrag = useCallback((kind: 'start' | 'goal', yawRad: number) => {
    const update = (previous: Pose2d | null): Pose2d | null =>
      previous ? { ...previous, yaw_rad: normalizeYaw(yawRad) } : previous;
    if (kind === 'start') setStartPose(update);
    else setGoalPose(update);
  }, []);

  const handlePoseFieldChange = useCallback(
    (kind: 'start' | 'goal', field: 'x' | 'y' | 'yaw_deg', value: number) => {
      const update = (previous: Pose2d | null): Pose2d | null => {
        if (!previous) return previous;
        if (field === 'x') return { ...previous, x_m: value };
        if (field === 'y') return { ...previous, y_m: value };
        return { ...previous, yaw_rad: normalizeYaw(degToRad(value)) };
      };
      if (kind === 'start') setStartPose(update);
      else setGoalPose(update);
    },
    [],
  );

  const handleClearPose = useCallback((kind: 'start' | 'goal') => {
    if (kind === 'start') setStartPose(null);
    else setGoalPose(null);
  }, []);

  const handleSetPoseFromSelection = useCallback(
    (kind: 'start' | 'goal') => {
      if (!snapshot || !selection || selection.kind !== 'lanelet') return;
      const lanelet = snapshot.lanelets.find((item) => item.id === selection.id) ?? null;
      if (!lanelet) return;
      const centerline = lanelet.centerline;
      const anchor =
        centerline && centerline.length > 0
          ? centerline[Math.floor(centerline.length / 2)]
          : lanelet.left[0];
      if (!anchor) return;
      const pose = poseFromLanelet(lanelet, anchor.x_m, anchor.y_m, 0);
      if (kind === 'start') setStartPose(pose);
      else setGoalPose(pose);
      setRouteState((previous) =>
        kind === 'start' ? { ...previous, from: selection.id } : { ...previous, to: selection.id },
      );
    },
    [snapshot, selection],
  );

  const runTopology = useCallback(
    async (kind: TopologyKind) => {
      if (!selection || selection.kind !== 'lanelet' || !snapshot) return;
      const id = selection.id;
      setBusy(true);
      try {
        if (serviceMatchesMap) {
          const result =
            kind === 'following'
              ? await client.following(id)
              : kind === 'preceding'
                ? await client.preceding(id)
                : await client.adjacent(id, kind === 'adjacent_left' ? 'left' : 'right');
          setTopology({ kind, ids: result.ids, status: result.status, reason: result.reason, source: 'service' });
        } else if (kind === 'following' || kind === 'preceding') {
          const ids = kind === 'following' ? localFollowing(snapshot, id) : localPreceding(snapshot, id);
          setTopology({ kind, ids, status: 'success', reason: '本地近似（服务离线）', source: 'local' });
        } else {
          setTopology({ kind, ids: [], status: 'invalid_input', reason: '邻接查询需要服务在线', source: 'local' });
        }
      } catch (serviceError) {
        setTopology({ kind, ids: [], status: 'error', reason: errorMessage(serviceError), source: 'service' });
      } finally {
        setBusy(false);
      }
    },
    [selection, snapshot, serviceMatchesMap, client],
  );

  const computeRoute = useCallback(async () => {
    const from = routeState.from.trim();
    const to = routeState.to.trim();
    if (!from || !to) {
      setRouteState((previous) => ({ ...previous, error: '请填写起终 lanelet ID（可先设位姿自动填充）', result: null }));
      return;
    }
    if (!serviceMatchesMap) {
      setRouteState((previous) => ({ ...previous, error: '路由查询需要地图服务在线且地图版本一致', result: null }));
      return;
    }
    setRouteState((previous) => ({ ...previous, loading: true, error: null }));
    try {
      const result = await client.route(from, to);
      setRouteState((previous) => ({
        ...previous,
        loading: false,
        result,
        error: result.status === 'success' ? null : result.reason || `状态：${result.status}`,
      }));
    } catch (serviceError) {
      setRouteState((previous) => ({ ...previous, loading: false, result: null, error: errorMessage(serviceError) }));
    }
  }, [routeState.from, routeState.to, serviceMatchesMap, client]);

  const runWindowQuery = useCallback(
    async (bounds: BoundsM) => {
      setWindowPreview(bounds);
      setWindowState({ bounds, loading: true, approximate: false, crop: null, local: null, error: null });
      if (serviceMatchesMap) {
        try {
          const crop = await client.queryWindow(bounds);
          setWindowState({ bounds, loading: false, approximate: false, crop, local: null, error: null });
          return;
        } catch (serviceError) {
          const local = snapshot ? filterSnapshotToWindow(snapshot, bounds) : null;
          setWindowState({
            bounds,
            loading: false,
            approximate: true,
            crop: null,
            local,
            error: `服务查询失败：${errorMessage(serviceError)}（回退本地近似）`,
          });
          return;
        }
      }
      const local = snapshot ? filterSnapshotToWindow(snapshot, bounds) : null;
      setWindowState({ bounds, loading: false, approximate: true, crop: null, local, error: null });
    },
    [serviceMatchesMap, client, snapshot],
  );

  const handleWindowDrag = useCallback(
    (bounds: BoundsM | null, committed: boolean) => {
      if (!bounds) {
        setWindowPreview(null);
        return;
      }
      if (committed) void runWindowQuery(bounds);
      else setWindowPreview(bounds);
    },
    [runWindowQuery],
  );

  const handleWindowBoundsChange = useCallback((bounds: BoundsM) => {
    setWindowPreview(bounds);
    setWindowState((previous) =>
      previous
        ? { ...previous, bounds }
        : { bounds, loading: false, approximate: false, crop: null, local: null, error: null },
    );
  }, []);

  const handleQueryWindow = useCallback(() => {
    if (windowState) {
      void runWindowQuery(windowState.bounds);
      return;
    }
    if (viewport) {
      const [centerX, centerY] = screenToWorld(viewport, viewport.width_px / 2, viewport.height_px / 2);
      void runWindowQuery({
        min_x_m: centerX - DEFAULT_WINDOW_HALF_M,
        min_y_m: centerY - DEFAULT_WINDOW_HALF_M,
        max_x_m: centerX + DEFAULT_WINDOW_HALF_M,
        max_y_m: centerY + DEFAULT_WINDOW_HALF_M,
      });
    }
  }, [windowState, viewport, runWindowQuery]);

  const handleClearWindow = useCallback(() => {
    setWindowState(null);
    setWindowPreview(null);
  }, []);

  const handleClearSelection = useCallback(() => {
    setSelection(null);
    setTopology(null);
  }, []);

  const handleClearRoute = useCallback(() => {
    setRouteState((previous) => ({ ...previous, result: null, error: null }));
  }, []);

  const footprints = useMemo<FootprintSpec[]>(() => {
    const list: FootprintSpec[] = [];
    if (vehicle) {
      if (startPose) list.push({ pose: startPose, kind: 'start' });
      if (goalPose) list.push({ pose: goalPose, kind: 'goal' });
      if (showCursorFootprint && cursorWorld) {
        list.push({ pose: { x_m: cursorWorld[0], y_m: cursorWorld[1], yaw_rad: 0 }, kind: 'cursor' });
      }
    }
    return list;
  }, [vehicle, startPose, goalPose, showCursorFootprint, cursorWorld]);

  const playbackOverlay = useMemo(
    () =>
      playbackActive && playbackSample
        ? buildPlaybackOverlay(playbackSample, playbackLayers, playbackTime)
        : null,
    [playbackActive, playbackSample, playbackLayers, playbackTime],
  );

  const overlays = useMemo(() => {
    // 服务裁剪结果的路径/点位于地图坐标系（与地图组件 test_crop 一致）；
    // window_frame 仅描述窗口局部系，用于坐标变换说明。
    return {
      selected_lanelets: selection?.kind === 'lanelet' ? [selection.id] : [],
      selected_areas: selection?.kind === 'area' ? [selection.id] : [],
      following: topology?.kind === 'following' ? topology.ids : [],
      preceding: topology?.kind === 'preceding' ? topology.ids : [],
      adjacent:
        topology && (topology.kind === 'adjacent_left' || topology.kind === 'adjacent_right')
          ? topology.ids
          : [],
      route: routeState.result?.status === 'success' ? routeState.result.lanelet_ids : [],
      start_pose: startPose,
      goal_pose: goalPose,
      window: windowPreview,
      crop_paths: windowState?.crop?.paths ?? [],
      crop_points: windowState?.crop?.points ?? [],
      obstacles,
      selected_obstacle_id: selectedObstacleId,
      obstacle_preview: obstaclePreview,
      vehicle,
      footprints,
      playback: playbackActive ? playbackOverlay : planOverlay,
    };
  }, [
    selection,
    topology,
    routeState.result,
    startPose,
    goalPose,
    windowPreview,
    windowState,
    obstacles,
    selectedObstacleId,
    obstaclePreview,
    vehicle,
    footprints,
    playbackOverlay,
    playbackActive,
    planOverlay,
  ]);

  // 测试钩子（CDP 检查/集成冒烟使用）：世界↔屏幕换算与关键状态。
  useEffect(() => {
    window.__plannerDebug = {
      worldToScreen: (x, y) => (viewport ? worldToScreen(viewport, x, y) : null),
      screenToWorld: (sx, sy) => (viewport ? screenToWorld(viewport, sx, sy) : null),
      getState: () => ({
        selection: selection ? { kind: selection.kind, id: selection.id, source: selection.source } : null,
        mode,
        service_online: service.online,
        topology: topology ? { kind: topology.kind, ids: topology.ids, source: topology.source } : null,
        route: routeState.result?.lanelet_ids ?? null,
        route_length_m: routeState.result?.length_m ?? null,
        route_from: routeState.from,
        route_to: routeState.to,
        start_pose: startPose,
        goal_pose: goalPose,
        viewport: viewport
          ? {
              width_px: viewport.width_px,
              height_px: viewport.height_px,
              scale_px_per_m: viewport.scale_px_per_m,
              center_x_m: viewport.center_x_m,
              center_y_m: viewport.center_y_m,
            }
          : null,
        window_result: windowState
          ? {
              loading: windowState.loading,
              approximate: windowState.approximate,
              lanelets: windowState.crop?.lanelet_ids ?? windowState.local?.lanelet_ids ?? [],
              bounds: windowState.bounds,
              error: windowState.error,
            }
          : null,
        scene: {
          map_id: snapshot?.map_id ?? null,
          map_version: snapshot?.map_version ?? null,
          obstacles,
          selected_obstacle_id: selectedObstacleId,
          start: startPose,
          goal: goalPose,
          algorithm_params: algorithmParams,
          vehicle_id: vehicle?.id ?? null,
          vehicle_source: vehicleSource,
          footprints: footprints.map((footprint) => footprint.kind),
          show_cursor_footprint: showCursorFootprint,
          autosave_status: autosaveStatus,
          mismatch_warning: mismatchWarning,
        },
        grid: (() => {
          const onlineMode = gridSource !== 'fixture';
          const onlineName: GridSourceName | null = onlineMode ? (gridSource as GridSourceName) : null;
          const online = onlineName ? onlineStates[onlineName] : null;
          const onlineTracker = onlineName ? onlineTrackersRef.current[onlineName] : null;
          const layer = activeGridLayer;
          const stateSummary = (name: GridSourceName) => {
            const view = onlineStates[name];
            const tracker = onlineTrackersRef.current[name];
            return {
              status: view.status,
              error: view.error,
              request_seq: view.request_seq,
              degraded: view.degraded,
              scene_id: view.scene_id,
              stale_ignored: tracker.stale_ignored,
              latest_seq: tracker.latest_seq,
              map_mismatch_ignored: tracker.map_mismatch_ignored,
              anomaly_count: tracker.anomaly_count,
            };
          };
          return {
            enabled: gridEnabled,
            opacity: gridOpacity,
            pick_mode: gridPick,
            source_kind: gridSource,
            status: onlineMode ? online?.status ?? 'idle' : gridState.status,
            error: onlineMode ? online?.error ?? null : gridState.error,
            file: onlineMode ? null : gridState.info?.file ?? null,
            kind: layer?.kind ?? null,
            source: layer?.source ?? null,
            width: layer?.width ?? null,
            height: layer?.height ?? null,
            resolution_m: layer?.resolution_m ?? null,
            requested_resolution_m: layer?.requested_resolution_m ?? null,
            effective_resolution_m: layer?.effective_resolution_m ?? null,
            degraded: layer?.degraded ?? false,
            scene_id: layer?.scene_id ?? null,
            window: layer?.window ?? null,
            map_id: layer?.map_id ?? null,
            map_version: layer?.map_version ?? null,
            cells_sha256: onlineMode ? null : gridState.info?.sha256 ?? null,
            bytes: onlineMode ? online?.bytes ?? null : gridState.info?.bytes ?? null,
            stale_ignored: gridTrackerRef.current.stale_ignored,
            latest_seq: gridTrackerRef.current.latest_seq,
            texture_rebuilds: gridTextureRebuildsRef.current,
            legend: ['空闲 free', '占据 occupied', '未知 unknown'],
            selected_cell: (() => {
              if (!gridCell || !layer) return null;
              const info = cellInfo(layer, gridCell.row, gridCell.col);
              return {
                row: info.row,
                col: info.col,
                state: info.state,
                center_x_m: info.center_x_m,
                center_y_m: info.center_y_m,
                in_bounds: info.in_bounds,
              };
            })(),
            online: {
              source: gridSource,
              active_status: online?.status ?? null,
              request_seq: online?.request_seq ?? null,
              requested_resolution_m: online?.requested_resolution_m ?? null,
              effective_resolution_m: online?.effective_resolution_m ?? null,
              degraded: online?.degraded ?? false,
              display_adjusted: online?.display_adjusted ?? false,
              display_factor: online?.display_factor ?? 1,
              scene_id: online?.scene_id ?? null,
              sample_cell_state: layer && layer.cells.length > 0 ? layer.cells[0] : null,
              stale_ignored: onlineTracker?.stale_ignored ?? 0,
              latest_seq: onlineTracker?.latest_seq ?? 0,
              map_mismatch_ignored: onlineTracker?.map_mismatch_ignored ?? 0,
              anomaly_count: onlineTracker?.anomaly_count ?? 0,
              fetched_at_ms: online?.fetched_at_ms ?? null,
              states: {
                driveable: stateSummary('driveable'),
                planning_scene: stateSummary('planning_scene'),
              },
              health: {
                status: gridHealthView.status,
                protocol_version: gridHealthView.protocol_version,
                scene_rules_version: gridHealthView.scene_rules_version,
                protocol_ok: gridHealthView.protocol_ok,
                scene_rules_ok: gridHealthView.scene_rules_ok,
                map_matches: gridHealthView.map_matches,
                reason: gridHealthView.reason,
              },
            },
          };
        })(),
        playback: {
          active: playbackActive,
          loaded: playbackBundle !== null,
          loading: playbackLoading,
          error: playbackError,
          sample_count: playbackBundle?.samples.length ?? 0,
          issues: playbackBundle?.issues ?? [],
          selected_id: playbackSelectedId,
          selected_kind: playbackSample?.sample.kind ?? null,
          selected_type: playbackSample?.sample.type ?? null,
          status: playbackSample?.kind === 'plan_result' ? playbackSample.result.status : null,
          path_segments: playbackOverlay?.paths[0]?.segments.length ?? 0,
          path_points:
            playbackOverlay?.paths[0]?.segments.reduce((sum, segment) => sum + segment.points.length, 0) ?? 0,
          cusps: playbackOverlay?.paths[0]?.cusps.length ?? 0,
          curve_points: playbackOverlay?.curves[0]?.points.length ?? 0,
          corridor_polygons: playbackOverlay?.corridors[0]?.polygons.length ?? 0,
          trajectory_points: playbackOverlay?.trajectories[0]?.points.length ?? 0,
          collision_visible: playbackOverlay?.collision !== null && playbackOverlay?.collision !== undefined,
          playback_time_s: playbackTime,
          duration_s: playbackDurationS,
          playing: playbackPlaying,
          profile_points: playbackProfiles.velocity.length,
          current_pose: playbackCurrent
            ? {
                x_m: playbackCurrent.pose.x_m,
                y_m: playbackCurrent.pose.y_m,
                yaw_rad: playbackCurrent.pose.yaw_rad,
                velocity_mps: playbackCurrent.velocity_mps,
                curvature_inv_m: playbackCurrent.curvature_inv_m,
              }
            : null,
          stale_warning: playbackStaleWarning,
          layers: playbackLayers,
          map_file: playbackBundle?.map_file ?? null,
          synthetic_note: playbackBundle?.index.synthetic_note ?? null,
        },
        plan: {
          health: planHealth,
          base: planApiBase,
          ws_url: planClient.eventsUrl(),
          ws_state: planWsState,
          push_transport: planWsState === 'open' ? 'websocket' : 'polling',
          latest_seq: planTracker.latest_seq,
          active_job_ids: planTracker.active_job_ids,
          stale_ignored: planTracker.stale_ignored,
          error: planError,
          busy: planBusy,
          active_backend: planActiveBackend,
          result_map_matches: planResultMapMatches,
          time_s: planTime,
          duration_s: planDurationS,
          playing: planPlaying,
          path_points:
            planOverlay?.paths[0]?.segments.reduce((sum, segment) => sum + segment.points.length, 0) ?? 0,
          cusps: planOverlay?.paths[0]?.cusps.length ?? 0,
          corridor_polygons: planOverlay?.corridors[0]?.polygons.length ?? 0,
          trajectory_points: planOverlay?.trajectories[0]?.points.length ?? 0,
          runs: Object.values(planRuns).map((run) => ({
            backend: run.backend,
            job_id: run.job_id,
            request_id: run.request_id,
            status: run.status,
            stage: run.stage,
            reason: run.reason,
            error: run.error,
            has_result: run.result !== null,
            validation: run.result?.validation ?? null,
            map_version: run.result?.map_version ?? null,
            scene_id: run.scene_id,
            scene_version: run.scene_version,
            history: run.history,
          })),
        },
        plan_scene: {
          status: planSceneSnapshot.status,
          scene_id: planSceneSnapshot.scene_id,
          scene_version: planSceneSnapshot.scene_version,
          job_id: planSceneSnapshot.job_id,
          backend: planSceneSnapshot.backend,
          error: planSceneSnapshot.error,
          overlay_enabled: planSceneOverlay,
          overlay_visible: planSceneOverlayLayer !== null,
          map_matches: planSceneMapMatches,
          degraded: planSceneSnapshot.layer?.degraded ?? false,
          resolution_m: planSceneSnapshot.layer?.resolution_m ?? null,
          width: planSceneSnapshot.layer?.width ?? null,
          height: planSceneSnapshot.layer?.height ?? null,
          counts: planSceneSnapshot.layer ? countStates(planSceneSnapshot.layer) : null,
          preview_scene_id: previewSceneId,
          same_as_preview:
            planSceneSnapshot.scene_id !== null && planSceneSnapshot.scene_id === previewSceneId,
          stale_ignored: planSceneTrackerRef.current.stale_ignored,
          rules_mismatch: sceneRulesMismatch,
        },
      }),
      injectPlanEvent: (event: unknown) => {
        try {
          return injectPlanEvent(parsePlanEvent(event));
        } catch {
          return null;
        }
      },
      gridPayload: (source: GridSourceName) => onlinePayloadsRef.current[source] ?? null,
      injectGridResponse: (source: GridSourceName, payload: unknown) => {
        const tracker = onlineTrackersRef.current[source];
        try {
          const layer = parseGridDocument(payload);
          const seq = layer.request_seq;
          if (seq === null) {
            return {
              decision: 'anomaly',
              apply: false,
              reason: '注入载荷缺少 request_seq（协议 §6）',
              stale_ignored: tracker.stale_ignored,
            };
          }
          const outcome = acceptOnlineGridResponse({
            tracker,
            seq,
            echoed_seq: seq,
            source,
            layer,
            map_id: snapshot?.map_id ?? '',
            map_version: snapshot?.map_version ?? '',
          });
          if (outcome.apply) {
            setOnlineStates((previous) => ({
              ...previous,
              [source]: {
                status: 'ready',
                layer,
                error: null,
                request_seq: seq,
                requested_resolution_m: layer.requested_resolution_m,
                effective_resolution_m: layer.effective_resolution_m,
                resolution_m: layer.resolution_m,
                degraded: layer.degraded,
                display_adjusted: false,
                display_factor: 1,
                stale_ignored: tracker.stale_ignored,
                map_mismatch_ignored: tracker.map_mismatch_ignored,
                scene_id: layer.scene_id,
                bytes: null,
                fetched_at_ms: Date.now(),
              },
            }));
          }
          return {
            decision: outcome.decision,
            apply: outcome.apply,
            reason: outcome.reason,
            stale_ignored: tracker.stale_ignored,
          };
        } catch (error) {
          return {
            decision: 'parse_error',
            apply: false,
            reason: error instanceof Error ? error.message : String(error),
            stale_ignored: tracker.stale_ignored,
          };
        }
      },
    };
    return () => {
      delete window.__plannerDebug;
    };
  }, [
    viewport,
    selection,
    mode,
    service.online,
    topology,
    routeState.result,
    startPose,
    goalPose,
    windowState,
    snapshot,
    obstacles,
    selectedObstacleId,
    algorithmParams,
    vehicle,
    vehicleSource,
    footprints,
    showCursorFootprint,
    autosaveStatus,
    mismatchWarning,
    playbackActive,
    playbackBundle,
    playbackLoading,
    playbackError,
    playbackSelectedId,
    playbackSample,
    playbackOverlay,
    playbackTime,
    playbackDurationS,
    playbackPlaying,
    playbackProfiles,
    playbackStaleWarning,
    playbackLayers,
    playbackCurrent,
    planHealth,
    planApiBase,
    planClient,
    planWsState,
    planTracker,
    planError,
    planBusy,
    planActiveBackend,
    planResultMapMatches,
    planTime,
    planDurationS,
    planPlaying,
    planOverlay,
    planRuns,
    injectPlanEvent,
    gridEnabled,
    gridOpacity,
    gridPick,
    gridState,
    gridCell,
    gridTrackerRef,
    gridSource,
    gridHealthView,
    onlineStates,
    activeGridLayer,
    planSceneSnapshot,
    planSceneOverlay,
    planSceneOverlayLayer,
    planSceneMapMatches,
    previewSceneId,
    sceneRulesMismatch,
  ]);

  const scaleBar = useMemo(() => (viewport ? chooseScaleBar(metersPerPixel(viewport)) : null), [viewport]);

  return (
    <div className="app">
      <Toolbar
        layers={layers}
        hasSnapshot={snapshot !== null}
        mode={mode}
        playbackActive={playbackActive}
        onToggleLayer={handleToggleLayer}
        onModeChange={setMode}
        onResetView={handleResetView}
        onOpenFile={handleOpenFile}
        onTogglePlayback={() => {
          if (playbackActive) void handleExitPlayback();
          else void handleLoadPlayback();
        }}
      />
      {error && (
        <div className="error-banner" role="alert">
          {error}
        </div>
      )}
      <div className="workspace">
        <div
          className="canvas-wrap"
          onDragOver={(event) => {
            event.preventDefault();
            setDragOver(true);
          }}
          onDragLeave={() => setDragOver(false)}
          onDrop={handleDrop}
        >
          <MapCanvas
            snapshot={snapshot}
            viewport={viewport}
            layers={layers}
            overlays={overlays}
            gridLayer={gridEnabled ? activeGridLayer : null}
            gridOpacity={gridOpacity}
            gridSelectedCell={gridPick ? gridCell : null}
            planSceneLayer={planSceneOverlayLayer}
            mode={mode}
            onViewportChange={setViewport}
            onSizeChange={handleSizeChange}
            onCursorWorld={setCursorWorld}
            onCanvasClick={handleCanvasClick}
            onHeadingDrag={handleHeadingDrag}
            onWindowDrag={handleWindowDrag}
            onObstacleCreate={handleObstacleCreate}
            onObstacleMove={handleObstacleMove}
            onObstacleClick={handleObstacleClick}
            onGridTextureRebuilt={handleGridTextureRebuilt}
            onGridDrawn={handleGridDrawn}
          />
          {notice && (
            <div className="notice-banner" role="status">
              {notice}
              <button type="button" className="notice-close" onClick={() => setNotice(null)}>
                ×
              </button>
            </div>
          )}
          {scaleBar && (
            <div className="scalebar" aria-hidden="true">
              <div className="scalebar-bar" style={{ width: `${scaleBar.width_px}px` }} />
              <span className="scalebar-label">{scaleBar.label}</span>
            </div>
          )}
          {cursorWorld && (
            <div className="readout">
              x = {cursorWorld[0].toFixed(2)} m · y = {cursorWorld[1].toFixed(2)} m
            </div>
          )}
          {!snapshot && !error && <div className="placeholder">正在加载地图快照…（可拖入 JSON 快照文件）</div>}
          {dragOver && <div className="drop-overlay">松开以加载地图快照</div>}
        </div>
        <InspectorPanel
          service={service}
          mapInfo={mapInfo}
          mode={mode}
          selection={selection}
          topology={topology}
          routeState={routeState}
          startPose={startPose}
          goalPose={goalPose}
          windowState={windowState}
          busy={busy}
          onRefreshService={() => void refreshService()}
          onClearSelection={handleClearSelection}
          onSelectLanelet={(id) => void selectLanelet(id)}
          onSelectArea={(id) => void selectArea(id)}
          onTopology={(kind) => void runTopology(kind)}
          onSetPoseFromSelection={handleSetPoseFromSelection}
          onPoseFieldChange={handlePoseFieldChange}
          onClearPose={handleClearPose}
          onRouteChange={(field, value) =>
            setRouteState((previous) => ({ ...previous, [field]: value, error: null }))
          }
          onComputeRoute={() => void computeRoute()}
          onClearRoute={handleClearRoute}
          onWindowBoundsChange={handleWindowBoundsChange}
          onQueryWindow={handleQueryWindow}
          onClearWindow={handleClearWindow}
        >
          <ScenePanel
            mapId={snapshot?.map_id ?? ''}
            mapVersion={snapshot?.map_version ?? ''}
            vehicle={vehicle}
            vehicleSource={vehicleSource}
            vehicleError={vehicleError}
            showCursorFootprint={showCursorFootprint}
            obstacles={obstacles}
            selectedObstacleId={selectedObstacleId}
            algorithmParams={algorithmParams}
            autosaveStatus={autosaveStatus}
            mismatchWarning={mismatchWarning}
            onSelectObstacle={setSelectedObstacleId}
            onUpdateObstacleRect={handleUpdateObstacleRect}
            onDeleteObstacle={handleDeleteObstacle}
            onAddObstacleAtCenter={handleAddObstacleAtCenter}
            onToggleCursorFootprint={setShowCursorFootprint}
            onParamsChange={handleParamsChange}
            onExportScene={handleExportScene}
            onImportScene={(file) => void handleImportScene(file)}
            onRestoreScene={restoreFromStorage}
            onClearScene={handleClearScene}
            onDismissMismatch={() => setMismatchWarning(null)}
          />
          <GridPanel
            catalog={gridCatalog}
            catalogError={gridCatalogError}
            selectedFile={gridFile}
            state={gridState}
            enabled={gridEnabled}
            opacity={gridOpacity}
            pickMode={gridPick}
            selectedCell={gridCell}
            source={gridSource}
            online={activeOnlineView}
            health={gridHealthView}
            resolution={gridOnlineResolution}
            requestWindowLabel={(() => {
              const requestWindow = computeOnlineWindow();
              if (!requestWindow) return '无可用窗口';
              return (
                `x [${requestWindow.min_x_m.toFixed(1)}, ${requestWindow.max_x_m.toFixed(1)}] · ` +
                `y [${requestWindow.min_y_m.toFixed(1)}, ${requestWindow.max_y_m.toFixed(1)}] m`
              );
            })()}
            onSelectSource={handleSelectGridSource}
            onResolutionChange={setGridOnlineResolution}
            onRequestOnline={() => {
              if (gridSource !== 'fixture') void requestOnlineGrid(gridSource);
            }}
            onSelectFile={handleSelectGridFile}
            onReload={handleReloadGrid}
            onFitView={handleFitGrid}
            onToggleEnabled={handleToggleGridEnabled}
            onOpacityChange={setGridOpacity}
            onTogglePick={() => setGridPick((previous) => !previous)}
            onClearCell={() => setGridCell(null)}
            onClear={handleClearGrid}
          />
          <PlanPanel
            health={planHealth}
            base={planApiBase}
            scene={planSceneSummary}
            backends={KNOWN_HYBRID_BACKENDS}
            backend={planBackend}
            compareA={planCompareA}
            compareB={planCompareB}
            runs={planRuns}
            activeBackend={planActiveBackend}
            tracker={planTracker}
            busy={planBusy}
            error={planError}
            layers={playbackLayers}
            time_s={planTime}
            duration_s={planDurationS}
            playing={planPlaying}
            current={planCurrent}
            profile={planProfiles.velocity}
            curvatureProfile={planProfiles.curvature}
            onRefreshHealth={() => void refreshPlanHealth()}
            onBackendChange={setPlanBackend}
            onCompareChange={(slot, backend) =>
              slot === 'a' ? setPlanCompareA(backend) : setPlanCompareB(backend)
            }
            onSubmit={() => void handlePlanSubmit('single')}
            onSubmitCompare={() => void handlePlanSubmit('compare')}
            onCancel={() => void handlePlanCancel()}
            onToggleLayer={handleTogglePlaybackLayer}
            onPlayPause={handlePlanPlayPause}
            onSeek={(t) => setPlanTime(Math.max(0, t))}
            onClearResults={handlePlanClear}
          />
          <PlanScenePanel
            state={planSceneSnapshot}
            overlayEnabled={planSceneOverlay}
            previewSceneId={previewSceneId}
            mapMatches={planSceneMapMatches}
            protocolOk={planSceneProtocolOk}
            protocolReason={planProtocolCheck.reason}
            sceneRulesMismatch={sceneRulesMismatch}
            onToggleOverlay={setPlanSceneOverlay}
            onRefetch={() => {
              if (planSceneId) {
                void fetchPlanSceneSnapshot(planSceneId, {
                  job_id: planActiveRun?.job_id ?? null,
                  backend: planActiveRun?.backend ?? null,
                  scene_version: planSceneVersion,
                });
              }
            }}
            onFitView={handleFitPlanScene}
            onClear={handleClearPlanScene}
          />
          <PlaybackPanel
            active={playbackActive}
            loading={playbackLoading}
            error={playbackError}
            bundle={playbackBundle}
            selected={playbackSample}
            layers={playbackLayers}
            time_s={playbackTime}
            duration_s={playbackDurationS}
            playing={playbackPlaying}
            staleWarning={playbackStaleWarning}
            current={playbackCurrent}
            profile={playbackProfiles.velocity}
            curvatureProfile={playbackProfiles.curvature}
            onLoad={() => void handleLoadPlayback()}
            onExit={() => void handleExitPlayback()}
            onSelect={handleSelectPlaybackSample}
            onToggleLayer={handleTogglePlaybackLayer}
            onPlayPause={handlePlaybackPlayPause}
            onSeek={handlePlaybackSeek}
          />
        </InspectorPanel>
      </div>
      <StatusBar
        snapshot={snapshot}
        sourceInfo={sourceInfo}
        viewport={viewport}
        canvasSize={canvasSize}
        service={service}
        mode={mode}
        playbackActive={playbackActive}
      />
    </div>
  );
}

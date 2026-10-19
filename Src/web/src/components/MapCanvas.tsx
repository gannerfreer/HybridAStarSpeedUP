import { useEffect, useRef, useState, type PointerEvent as ReactPointerEvent } from 'react';
import { executeDrawCommands } from '../core/canvas';
import { gridCellScreenCorners, drawGridLayer } from '../core/grid/render';
import { buildPlanSceneImageData, GridTextureCache } from '../core/grid/texture';
import type { GridCellRef, GridLayerData } from '../core/grid/types';
import type { InteractionMode } from '../core/interaction';
import type { LayerState } from '../core/layers';
import { normalizeYaw, yawFromDelta, type Pose2d } from '../core/pose';
import { buildDrawCommands, buildOverlayCommands, MAP_COLORS, type MapOverlays } from '../core/render';
import { hitTestObstacles } from '../core/scene/obstacles';
import { screenToWorld, worldToScreen, type Viewport } from '../core/transform';
import type { BoundsM, MapSnapshot } from '../core/types';
import { normalizeBounds } from '../core/window';
import { panBy, zoomAt } from '../core/viewport';

export interface CanvasSize {
  width_px: number;
  height_px: number;
}

export interface MapCanvasProps {
  snapshot: MapSnapshot | null;
  viewport: Viewport | null;
  layers: LayerState;
  overlays: MapOverlays;
  /** 栅格夹具图层（合成显示数据；绘制在矢量边界之下）。 */
  gridLayer: GridLayerData | null;
  gridOpacity: number;
  gridSelectedCell: GridCellRef | null;
  /** 规划实际场景快照叠加层（X-W2；独立于当前预览，空闲透明、占据品红）。 */
  planSceneLayer?: GridLayerData | null;
  mode: InteractionMode;
  onViewportChange: (next: Viewport) => void;
  onSizeChange: (size: CanvasSize) => void;
  onCursorWorld: (position: [number, number] | null) => void;
  /** 单击（未拖动）画布：世界坐标；由 App 按模式处理。 */
  onCanvasClick: (world: [number, number]) => void;
  /** 拖动起点/终点圆点：新航向（弧度）。 */
  onHeadingDrag: (kind: 'start' | 'goal', yawRad: number) => void;
  /** 窗口查询拖拽：bounds 为世界坐标矩形；committed=true 表示拖拽结束。 */
  onWindowDrag: (bounds: BoundsM | null, committed: boolean) => void;
  /** W3 障碍新建拖拽：bounds 为世界坐标矩形；committed=true 表示拖拽结束。 */
  onObstacleCreate: (bounds: BoundsM | null, committed: boolean) => void;
  /** W3 障碍移动：世界坐标增量。 */
  onObstacleMove: (id: string, dxM: number, dyM: number) => void;
  /** W3 单击选中障碍。 */
  onObstacleClick: (id: string) => void;
  /** 栅格纹理重建通知（缩放/平移缓存复用证据）。 */
  onGridTextureRebuilt?: () => void;
  /** 栅格绘制完成通知（显示性能基准热重绘计时）。 */
  onGridDrawn?: () => void;
}

type DragState =
  | { type: 'pan'; pointerId: number; lastClientX: number; lastClientY: number; movedPx: number }
  | { type: 'heading'; pointerId: number; kind: 'start' | 'goal' }
  | { type: 'window'; pointerId: number; startX: number; startY: number }
  | { type: 'obstacle-create'; pointerId: number; startX: number; startY: number }
  | { type: 'obstacle-move'; pointerId: number; id: string; lastWorldX: number; lastWorldY: number; movedPx: number }
  | null;

const CLICK_MOVE_TOLERANCE_PX = 4;
const POSE_HIT_RADIUS_PX = 12;

export function MapCanvas(props: MapCanvasProps) {
  const { snapshot, viewport, layers, overlays, gridLayer, gridOpacity, gridSelectedCell, planSceneLayer, mode } =
    props;
  const {
    onViewportChange,
    onSizeChange,
    onCursorWorld,
    onCanvasClick,
    onHeadingDrag,
    onWindowDrag,
    onObstacleCreate,
    onObstacleMove,
    onObstacleClick,
  } = props;
  const hostRef = useRef<HTMLDivElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const gridCacheRef = useRef<GridTextureCache | null>(null);
  const planSceneCacheRef = useRef<GridTextureCache | null>(null);
  const dragRef = useRef<DragState>(null);
  const [dragging, setDragging] = useState(false);

  if (gridCacheRef.current === null) gridCacheRef.current = new GridTextureCache();
  if (planSceneCacheRef.current === null) {
    planSceneCacheRef.current = new GridTextureCache(undefined, buildPlanSceneImageData);
  }

  // 尺寸测量（CSS 像素）。除 ResizeObserver 外增加 window resize 与延迟复测，
  // 覆盖“初始测量早于最终布局”或观察器未触发的边界情况（幂等，尺寸不变时去重）。
  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    const measure = () => {
      const rect = host.getBoundingClientRect();
      onSizeChange({
        width_px: Math.max(1, Math.round(rect.width)),
        height_px: Math.max(1, Math.round(rect.height)),
      });
    };
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(host);
    window.addEventListener('resize', measure);
    const raf = window.requestAnimationFrame(measure);
    const timer = window.setTimeout(measure, 300);
    return () => {
      observer.disconnect();
      window.removeEventListener('resize', measure);
      window.cancelAnimationFrame(raf);
      window.clearTimeout(timer);
    };
  }, [onSizeChange]);

  // 单次重绘：清屏 → 背景 → 基础命令 + W2 覆盖层命令；DPR 通过一次 setTransform 处理。
  useEffect(() => {
    const canvas = canvasRef.current;
    const host = hostRef.current;
    if (!canvas || !host) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const dpr = window.devicePixelRatio > 0 ? window.devicePixelRatio : 1;
    const rect = host.getBoundingClientRect();
    const widthPx = Math.max(1, Math.round(rect.width));
    const heightPx = Math.max(1, Math.round(rect.height));
    const effectiveViewport =
      viewport && (viewport.width_px !== widthPx || viewport.height_px !== heightPx)
        ? { ...viewport, width_px: widthPx, height_px: heightPx }
        : viewport;
    if (viewport && effectiveViewport !== viewport) {
      onSizeChange({ width_px: widthPx, height_px: heightPx });
    }
    canvas.width = Math.max(1, Math.round(widthPx * dpr));
    canvas.height = Math.max(1, Math.round(heightPx * dpr));
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, widthPx, heightPx);
    ctx.fillStyle = MAP_COLORS.background;
    ctx.fillRect(0, 0, widthPx, heightPx);
    if (effectiveViewport && gridLayer && gridOpacity > 0) {
      const cache = gridCacheRef.current;
      if (cache) {
        const rebuilt = drawGridLayer(ctx, gridLayer, effectiveViewport, gridOpacity, cache);
        if (rebuilt) {
          if (typeof performance !== 'undefined') performance.mark('grid:first-frame');
          props.onGridTextureRebuilt?.();
        }
      }
    }
    if (effectiveViewport && planSceneLayer) {
      const cache = planSceneCacheRef.current;
      if (cache) {
        const rebuilt = drawGridLayer(ctx, planSceneLayer, effectiveViewport, 1, cache);
        if (rebuilt) {
          if (typeof performance !== 'undefined') performance.mark('plan-scene:first-frame');
          props.onGridTextureRebuilt?.();
        }
      }
    }
    if (snapshot && effectiveViewport) {
      const commands = [
        ...buildDrawCommands(snapshot, effectiveViewport, layers),
        ...buildOverlayCommands(snapshot, effectiveViewport, overlays),
      ];
      executeDrawCommands(ctx, commands);
    }
    if (effectiveViewport && gridLayer && gridSelectedCell && gridOpacity > 0) {
      const corners = gridCellScreenCorners(
        gridLayer,
        effectiveViewport,
        gridSelectedCell.row,
        gridSelectedCell.col,
      );
      ctx.beginPath();
      ctx.moveTo(corners[0][0], corners[0][1]);
      for (let index = 1; index < corners.length; index += 1) {
        ctx.lineTo(corners[index][0], corners[index][1]);
      }
      ctx.closePath();
      ctx.strokeStyle = '#fde047';
      ctx.lineWidth = 2;
      ctx.stroke();
    }
    if ((gridLayer || planSceneLayer) && effectiveViewport) {
      props.onGridDrawn?.();
    }
  }, [snapshot, viewport, layers, overlays, gridLayer, gridOpacity, gridSelectedCell, planSceneLayer, onSizeChange]);

  // 滚轮缩放（以光标为中心）。原生监听以启用 preventDefault。
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const handleWheel = (event: WheelEvent) => {
      event.preventDefault();
      const current = viewport;
      if (!current) return;
      const rect = canvas.getBoundingClientRect();
      const anchorX = event.clientX - rect.left;
      const anchorY = event.clientY - rect.top;
      const delta =
        event.deltaMode === 1 ? event.deltaY * 16 : event.deltaMode === 2 ? event.deltaY * 100 : event.deltaY;
      const factor = Math.pow(1.0015, -delta);
      onViewportChange(zoomAt(current, factor, anchorX, anchorY));
    };
    canvas.addEventListener('wheel', handleWheel, { passive: false });
    return () => canvas.removeEventListener('wheel', handleWheel);
  }, [viewport, onViewportChange]);

  const eventToScreen = (event: ReactPointerEvent<HTMLCanvasElement>): [number, number] => {
    const rect = event.currentTarget.getBoundingClientRect();
    return [event.clientX - rect.left, event.clientY - rect.top];
  };

  const findPoseMarker = (sx: number, sy: number): 'start' | 'goal' | null => {
    if (!viewport) return null;
    const candidates: Array<['start' | 'goal', Pose2d | null | undefined]> = [
      ['start', overlays.start_pose],
      ['goal', overlays.goal_pose],
    ];
    for (const [kind, pose] of candidates) {
      if (!pose) continue;
      const [px, py] = worldToScreen(viewport, pose.x_m, pose.y_m);
      if (Math.hypot(px - sx, py - sy) <= POSE_HIT_RADIUS_PX) return kind;
    }
    return null;
  };

  const handlePointerDown = (event: ReactPointerEvent<HTMLCanvasElement>) => {
    if (event.button !== 0 || !viewport) return;
    event.currentTarget.setPointerCapture(event.pointerId);
    const [sx, sy] = eventToScreen(event);

    if (mode === 'window') {
      const [worldX, worldY] = screenToWorld(viewport, sx, sy);
      dragRef.current = { type: 'window', pointerId: event.pointerId, startX: worldX, startY: worldY };
      setDragging(true);
      return;
    }

    if (mode === 'obstacle') {
      const [worldX, worldY] = screenToWorld(viewport, sx, sy);
      const hit = hitTestObstacles(overlays.obstacles ?? [], worldX, worldY);
      if (hit) {
        dragRef.current = {
          type: 'obstacle-move',
          pointerId: event.pointerId,
          id: hit.id,
          lastWorldX: worldX,
          lastWorldY: worldY,
          movedPx: 0,
        };
      } else {
        dragRef.current = {
          type: 'obstacle-create',
          pointerId: event.pointerId,
          startX: worldX,
          startY: worldY,
        };
      }
      setDragging(true);
      return;
    }

    const marker = findPoseMarker(sx, sy);
    if (marker) {
      dragRef.current = { type: 'heading', pointerId: event.pointerId, kind: marker };
      setDragging(true);
      return;
    }

    dragRef.current = {
      type: 'pan',
      pointerId: event.pointerId,
      lastClientX: event.clientX,
      lastClientY: event.clientY,
      movedPx: 0,
    };
    setDragging(true);
  };

  const handlePointerMove = (event: ReactPointerEvent<HTMLCanvasElement>) => {
    if (!viewport) return;
    const [sx, sy] = eventToScreen(event);
    onCursorWorld(screenToWorld(viewport, sx, sy));

    const drag = dragRef.current;
    if (!drag || drag.pointerId !== event.pointerId) return;

    if (drag.type === 'heading') {
      const [worldX, worldY] = screenToWorld(viewport, sx, sy);
      const pose = drag.kind === 'start' ? overlays.start_pose : overlays.goal_pose;
      if (pose) {
        const yaw = yawFromDelta(worldX - pose.x_m, worldY - pose.y_m);
        onHeadingDrag(drag.kind, normalizeYaw(yaw));
      }
    } else if (drag.type === 'window') {
      const [worldX, worldY] = screenToWorld(viewport, sx, sy);
      onWindowDrag(normalizeBounds(drag.startX, drag.startY, worldX, worldY), false);
    } else if (drag.type === 'obstacle-create') {
      const [worldX, worldY] = screenToWorld(viewport, sx, sy);
      onObstacleCreate(normalizeBounds(drag.startX, drag.startY, worldX, worldY), false);
    } else if (drag.type === 'obstacle-move') {
      const [worldX, worldY] = screenToWorld(viewport, sx, sy);
      const dx = worldX - drag.lastWorldX;
      const dy = worldY - drag.lastWorldY;
      drag.lastWorldX = worldX;
      drag.lastWorldY = worldY;
      drag.movedPx += (Math.abs(dx) + Math.abs(dy)) * viewport.scale_px_per_m;
      onObstacleMove(drag.id, dx, dy);
    } else {
      const dx = event.clientX - drag.lastClientX;
      const dy = event.clientY - drag.lastClientY;
      drag.lastClientX = event.clientX;
      drag.lastClientY = event.clientY;
      drag.movedPx += Math.abs(dx) + Math.abs(dy);
      onViewportChange(panBy(viewport, dx, dy));
    }
  };

  const handlePointerUp = (event: ReactPointerEvent<HTMLCanvasElement>) => {
    const drag = dragRef.current;
    if (!drag || drag.pointerId !== event.pointerId) return;
    dragRef.current = null;
    setDragging(false);
    if (!viewport) return;

    if (drag.type === 'window') {
      const [sx, sy] = eventToScreen(event);
      const [worldX, worldY] = screenToWorld(viewport, sx, sy);
      onWindowDrag(normalizeBounds(drag.startX, drag.startY, worldX, worldY), true);
    } else if (drag.type === 'obstacle-create') {
      const [sx, sy] = eventToScreen(event);
      const [worldX, worldY] = screenToWorld(viewport, sx, sy);
      onObstacleCreate(normalizeBounds(drag.startX, drag.startY, worldX, worldY), true);
    } else if (drag.type === 'obstacle-move') {
      if (drag.movedPx <= CLICK_MOVE_TOLERANCE_PX) onObstacleClick(drag.id);
    } else if (drag.type === 'pan' && drag.movedPx <= CLICK_MOVE_TOLERANCE_PX) {
      const [sx, sy] = eventToScreen(event);
      onCanvasClick(screenToWorld(viewport, sx, sy));
    }
  };

  const handlePointerLeave = () => {
    onCursorWorld(null);
  };

  const cursorClass =
    mode === 'window' || mode === 'obstacle' ? 'map-canvas crosshair' : dragging ? 'map-canvas dragging' : 'map-canvas';

  return (
    <div className="canvas-host" ref={hostRef}>
      <canvas
        ref={canvasRef}
        className={cursorClass}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
        onPointerLeave={handlePointerLeave}
      />
    </div>
  );
}

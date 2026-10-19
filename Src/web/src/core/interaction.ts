/** W2/W3 画布交互模式。 */
export type InteractionMode = 'select' | 'set_start' | 'set_goal' | 'window' | 'obstacle';

export const INTERACTION_MODES: readonly InteractionMode[] = [
  'select',
  'set_start',
  'set_goal',
  'window',
  'obstacle',
];

export const INTERACTION_MODE_LABELS: Record<InteractionMode, string> = {
  select: '选择',
  set_start: '设起点',
  set_goal: '设终点',
  window: '窗口查询',
  obstacle: '障碍',
};

export const INTERACTION_MODE_HINTS: Record<InteractionMode, string> = {
  select: '单击选择 lanelet/区域；拖动起点/终点圆点可调整航向',
  set_start: '单击地图设置起点位姿（航向取命中 lanelet 中心线切向）',
  set_goal: '单击地图设置终点位姿（航向取命中 lanelet 中心线切向）',
  window: '拖拽矩形框查询局部窗口（服务在线时返回裁剪结果）',
  obstacle: '拖拽空白处新建矩形障碍；拖拽已有障碍移动；单击障碍选中后在面板编辑',
};

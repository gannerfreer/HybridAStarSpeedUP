// S-03 justinhj 适配器变体：内存池关（-DUSE_FSA_MEMORY=0 经守卫补丁）。
// 实现位于 justinhj_adapter_impl.ipp；此 TU 仅选择配置与命名空间。

#define USE_FSA_MEMORY 0
#define PLANNER_JUSTINHJ_NAMESPACE justinhj_nopool
#define PLANNER_JUSTINHJ_FACTORY make_justinhj_astar_nopool

#include "justinhj_adapter_impl.ipp"

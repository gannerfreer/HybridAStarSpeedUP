// S-03 justinhj 适配器变体：内存池开（上游默认；USE_FSA_MEMORY=1）。
// 实现位于 justinhj_adapter_impl.ipp；此 TU 仅选择配置与命名空间。

#define USE_FSA_MEMORY 1
#define PLANNER_JUSTINHJ_NAMESPACE justinhj_pool
#define PLANNER_JUSTINHJ_FACTORY make_justinhj_astar_pool

#include "justinhj_adapter_impl.ipp"

/**
 * @file jps_planner.h
 * @brief JPSPlanner
 */
#ifndef JPS_PLANNER_BASE_H
#define JPS_PLANNER_BASE_H

#include <functional>  // [grid_search 适配补丁] 终止检查钩子

#include <jps_basis/data_type.h>
#include <jps_planner/jps_planner/graph_search.h>
#include <jps_collision/map_util.h>

class GraphSearch;
/**
 * @brief Abstract base for planning
 */
template <int Dim>
class JPSPlanner
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable debug mode
     */
    JPSPlanner(bool verbose = false);

    ///Set map util for collistion checking
    void setMapUtil(const std::shared_ptr<JPS::MapUtil<Dim>>& map_util);
    /**
     * @brief Status of the planner
     *
     * 0 --- exit normally;
     * -1 --- no path found;
     * 1, 2 --- start or goal is not free.
     */
    int status();
    ///Get the modified path
    vec_Vecf<Dim> getPath();
    ///Get the raw path
    vec_Vecf<Dim> getRawPath();
    ///remove redundant points on the same line
    vec_Vecf<Dim> removeLinePts(const vec_Vecf<Dim> &path);
    ///Remove some corner waypoints
    vec_Vecf<Dim> removeCornerPts(const vec_Vecf<Dim> &path);
    ///Must be called before run the planning thread
    void updateMap();
    /**
     * @brief [grid_search 适配补丁 P4] 选择四邻域（仅普通 A* 模式）
     * @param four_connect true 时普通 A* 使用正交四步；JPS 模式必须保持八邻域
     * @details 上游固定八邻域；本补丁在 plan() 创建 GraphSearch 后应用。
     */
    void setFourConnect(bool four_connect);
    /**
     * @brief [grid_search 适配补丁] 设置协作取消/超时检查钩子
     * @param check 返回 true 时搜索循环尽快终止（取消与无路径由适配层区分）
     */
    void setTerminationCheck(std::function<bool()> check);
    ///Planning function
    bool plan(const Vecf<Dim> &start, const Vecf<Dim> &goal, decimal_t eps = 1, bool use_jps = true);
    ///Get the nodes in open set
    vec_Vecf<Dim> getOpenSet() const;
    ///Get the nodes in close set
    vec_Vecf<Dim> getCloseSet() const;
    ///Get all the nodes
    vec_Vecf<Dim> getAllSet() const;
  protected:
    ///Assume using 3D voxel map for all 2d and 3d planning
    std::shared_ptr<JPS::MapUtil<Dim>> map_util_;
    ///The planner
    std::shared_ptr<JPS::GraphSearch> graph_search_;
    ///Raw path from planner
    vec_Vecf<Dim> raw_path_;
    ///Modified path for future usage
    vec_Vecf<Dim> path_;
    ///Flag indicating the success of planning
    int status_ = 0;
    ///Enabled for printing info
    bool planner_verbose_;
    ///1-D map array
    std::vector<char> cmap_;
    // [grid_search 适配补丁] 四邻域开关与取消/超时钩子（默认保持上游行为）
    bool four_connect_ = false;
    std::function<bool()> term_check_;
};

///Planner for 2D OccMap
typedef JPSPlanner<2> JPSPlanner2D;

///Planner for 3D VoxelMap
typedef JPSPlanner<3> JPSPlanner3D;

#endif

#pragma once

// ===========================================================================
// PROVENANCE / 来源说明
// Adapted from ZJU-FAST-Lab/Dftpav (GPL-3.0), see Src/dftpav/PROVENANCE.md:
//   * src/Plan/traj_planner/include/plan_manage/traj_optimizer.h
// Pinned upstream commit: 87f63ad2f67506e090b36eac38f5d247c51a330e
// (dependencies.lock.json entry `dftpav`).
//
// Porting changes (all inside the `planner::dftpav::ported` namespace):
//   * removed ROS (`ros/ros.h`, publishers, `ros::Time`), protobuf
//     (`minco_config.pb.h`) and the HKUST `common/*` dependencies; the
//     protobuf `planning::minco::Config` is replaced by the plain
//     `OptimizerConfig` value struct below (defaults taken from upstream
//     config/minco_config.pb.txt);
//   * removed the dynamic-surround (`surround_trajs_`) feature and all
//     visualization helpers; only the static corridor, body-shape, dynamic
//     limit, gear-optimisation and time terms of the cost remain;
//   * removed `extractVs` (the only user of the ported geoutils2d helpers),
//     which is unused by the optimization itself;
//   * `setParam` now takes `OptimizerConfig` directly instead of a
//     `ros::NodeHandle` + protobuf message;
//   * `OptimizeTrajectory` keeps the upstream signature except that `now`
//     defaults to 0.0 instead of `ros::Time::now().toSec()`;
//   * added read-only accessors for the adapter/statistics
//     (`getIterations`, `getLbfgsResult`, `getFinalCost`);
//   * PORT EXTENSION (cooperative cancellation, see Src/dftpav/PROVENANCE.md):
//     the dormant upstream progress hook (`earlyExitCallback`) is wired into
//     `lbfgs_optimize` (its signature is aligned with this port's
//     `lbfgs_progress_t`) and an optional caller callback
//     (`setCancelCheck`/`wasCancelled`) is polled at the cost-function and
//     progress-callback boundaries. When it requests cancellation the solve
//     stops (LBFGS_CANCELED) and `OptimizeTrajectory` reports failure instead
//     of exposing the partial iterate. The optimization math is unchanged.
// ===========================================================================

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Eigen>

#include "planner/dftpav/vehicle_param.hpp"

#include "lbfgs.hpp"
#include "poly_traj.hpp"

namespace planner::dftpav::ported {

namespace plan_manage
{

  // Plain value configuration replacing upstream `planning::minco::Config`
  // (protobuf). Defaults follow upstream config/minco_config.pb.txt; fields
  // not relevant to the extracted static-corridor optimizer are omitted.
  struct OptimizerConfig
  {
    int traj_resolution = 16;       // constraint samples per inner piece
    int destraj_resolution = 32;    // constraint samples on first/last piece
    double wei_sta_obs = 1000.0;    // static obstacle/corridor weight
    double wei_feas = 2500.0;       // dynamic feasibility weight
    double wei_time = 500.0;        // total time weight
    double half_margin = 0.15;      // hull margin added to both sides [m]

    double max_forward_vel = 5.0;   // [m/s]
    double max_backward_vel = 2.0;  // [m/s]
    double max_forward_cur = 1.0;   // [1/m]
    double max_backward_cur = 1.0;  // [1/m]
    double max_forward_acc = 8.0;   // [m/s^2]
    double max_backward_acc = 4.0;  // [m/s^2]
    double max_latacc = 5.0;        // [m/s^2]
    double max_phidot = 10000.0;    // [rad/s]

    // Upstream hard-coded `non_sinv = 0.24` m/s junction speed for gear
    // changes. The adapter overrides it (default 0.0) so that the output
    // trajectory stops exactly at a direction change (plan section 7.1).
    double non_sinv = 0.24;

    // Port addition (not in upstream): quadratic anchor of the inner waypoints
    // and junction positions to their prefabricated initial values, in
    // cost-per-metre^2 units. Keeps the soft-constrained solution near the
    // input path (station consistency with the corridor parametrization);
    // 0 disables the term.
    double wei_anchor = 0.0;

    bool gearopt = true;
    int lbfgs_memsize = 256;
    int lbfgs_past = 3;
    double lbfgs_delta = 1.0e-4;
    double mini_t = 0.1;            // minimum duration of one piece [s]

    // Port additions for the isolated backend's status reporting.
    int lbfgs_max_iterations = 12000;   // upstream hard-coded 12000
    double cost_fail_threshold = 50000.0;  // upstream hard-coded 50000.0
  };

  class PolyTrajOptimizer
  {

  private:

    int traj_resolution_; // number of distinctive constrain points each piece
    int destraj_resolution_; //number of distinctive constrain points of the first and last piece (should be more dense!
    int variable_num_;     // optimization variables
    int iter_num_;         // iteration of the solver


    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_;                         // obstacle weight
    double wei_feas_;                        // feasibility weight
    double wei_time_;                        // time weight

    double half_margin;                        // safe margin
    ::planner::dftpav::VehicleParam veh_param_;
    double L_;
    std::vector<Eigen::Vector2d> lz_set_;
    std::vector<Eigen::Vector2d> vec_le_;


    /*new added param*/
    double max_forward_vel, max_backward_vel, max_forward_cur, max_backward_cur;
    double max_forward_acc, max_backward_acc, max_phidot_, max_latacc_;       // dynamic limits
    double non_sinv = 0.24;
    double wei_anchor_ = 0.0;
    bool GearOpt = true;
    /*lbfgs param*/
    int memsize;
    int past;
    double delta;
    double mini_T = 0.05;
    int lbfgs_max_iterations_;
    double cost_fail_threshold_;


    // Each col of cfgHs denotes a facet (outter_normal^T,point^T)^T
    Eigen::Matrix<double, 2, 2> B_h;
    double epis;
    std::vector<int> singul_container;
    std::vector<Eigen::MatrixXd> iniState_container;
    std::vector<Eigen::MatrixXd> finState_container;
    std::vector<std::vector<Eigen::MatrixXd>> cfgHs_container;
    std::vector<Eigen::MatrixXd> iniInnerPts_container_;
    std::vector<Eigen::Vector2d> iniGear_container_;
    int trajnum;
    std::vector<plan_utils::MinJerkOpt> jerkOpt_container;
    std::vector<int> piece_num_container;

    /* port additions: result reporting */
    int lbfgs_result_ = -1;
    double final_cost_ = 0.0;

    /* port addition: cooperative cancellation hook (see PROVENANCE.md).
     * `cancel_check_` is polled at the cost-function and progress-callback
     * boundaries; `cancelled_` records that a cancel was observed so that a
     * partial iterate is never reported as success. */
    bool (*cancel_check_)(void *) = nullptr;
    void *cancel_check_context_ = nullptr;
    bool cancelled_ = false;

    bool cancelRequested() const
    {
      return cancel_check_ != nullptr && cancel_check_(cancel_check_context_);
    }

  public:

    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    /* set variables */
    // Replaces upstream `void setParam(ros::NodeHandle nh, planning::minco::Config cfg_)`.
    void setParam(const OptimizerConfig& cfg_);

    // Port addition: upstream relied on the default-constructed HKUST
    // VehicleParam and never configured the geometry; the adapter sets the
    // whole-body rectangle and rear-axle offset explicitly here.
    void setVehicleParam(const ::planner::dftpav::VehicleParam& vp) { veh_param_ = vp; }

    /* helper functions */
    inline const std::vector<plan_utils::MinJerkOpt> *getMinJerkOptPtr(void) { return &jerkOpt_container; }
    inline int get_traj_resolution_() { return traj_resolution_; };
    inline int get_destraj_resolution_() { return destraj_resolution_; };
    inline int getIterations() const { return iter_num_; }
    inline int getLbfgsResult() const { return lbfgs_result_; }
    inline double getFinalCost() const { return final_cost_; }

    /* Port addition (cooperative cancellation): optional callback polled at
     * the L-BFGS iteration/cost-function boundaries. When it returns true the
     * solve stops and OptimizeTrajectory() returns false with
     * wasCancelled() == true. The callback must be non-blocking, must not
     * throw, and must stay valid until the OptimizeTrajectory() call returns
     * (the callback and its context are not copied). */
    void setCancelCheck(bool (*check)(void *), void *context)
    {
      cancel_check_ = check;
      cancel_check_context_ = context;
    }

    /* True when the last OptimizeTrajectory() call stopped because the
     * cancellation hook requested it (never combined with a success return). */
    inline bool wasCancelled() const { return cancelled_; }

    /* main planning API */
    bool OptimizeTrajectory(const std::vector<Eigen::MatrixXd> &iniStates, const std::vector<Eigen::MatrixXd> &finStates,
                            std::vector<Eigen::MatrixXd> &initInnerPts, const Eigen::VectorXd &initTs,
                            std::vector<std::vector<Eigen::MatrixXd>> &hPoly_container,std::vector<int> singuls,double now = 0.0,double help_eps = 1.0e-4);


  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad);

    /* Port extension: the upstream progress callback kept the liblbfgs-style
     * signature and was never wired into lbfgs_optimize(); the port aligns it
     * with this lbfgs.hpp's lbfgs_progress_t and wires it (see PROVENANCE.md). */
    static int earlyExitCallback(void *func_data, const Eigen::VectorXd &x,
                                 const Eigen::VectorXd &g, const double fx,
                                 const double step, const int k, const int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);
    void VirtualTGradCost(const double &RT, const double & VT, const double & gdRT, double & gdVT, double & costT);


    void addPVAGradCost2CT(Eigen::VectorXd &costs, const int trajid, const double trajtime);

    void getBoundPts(Eigen::Vector2d &position, double angle, std::vector<Eigen::Vector2d> &BoundVertices);
    void positiveSmoothedL1(const double &x, double &f, double &df);

  public:
    typedef std::unique_ptr<PolyTrajOptimizer> Ptr;

  };

} // namespace plan_manage

}  // namespace planner::dftpav::ported

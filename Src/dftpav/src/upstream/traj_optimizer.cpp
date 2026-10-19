// ===========================================================================
// PROVENANCE / 来源说明
// Adapted from ZJU-FAST-Lab/Dftpav (GPL-3.0), see Src/dftpav/PROVENANCE.md:
//   * src/Plan/traj_planner/src/traj_optimizer.cpp
// Pinned upstream commit: 87f63ad2f67506e090b36eac38f5d247c51a330e
// (dependencies.lock.json entry `dftpav`).
//
// Porting changes (all inside the `planner::dftpav::ported` namespace):
//   * removed ROS/protobuf/visualization: publishers, `ros::Time`,
//     `ROS_ERROR/ROS_WARN` messages (replaced by stderr diagnostics) and the
//     `minco_config.pb.h` reads (replaced by OptimizerConfig);
//   * removed the dynamic-surround avoidance feature
//     (surroundGradCostP / debugGradCheck / dynamicObsGradCostP /
//     dynamicObsCosCheck / log_sum_exp) and its cost hook; the static
//     corridor, body-hull, dynamic-limit, gear and time terms are unchanged;
//   * removed unused helpers positiveSmoothedL3 / getBoundPts and the debug
//     point collections;
//   * `OptimizeTrajectory` keeps the upstream math and signature with
//     `now = 0.0` and records `lbfgs_result_` / `final_cost_` for status
//     reporting; the iteration limit and the cost-failure threshold are
//     configurable (upstream hard-coded 12000 / 50000.0);
//   * PORT EXTENSION (cooperative cancellation, see PROVENANCE.md): the
//     upstream `earlyExitCallback` progress hook is wired into
//     `lbfgs_optimize` (its signature is aligned with this port's
//     `lbfgs_progress_t`) and an optional caller callback (`setCancelCheck`)
//     is polled in the cost callback and in the progress hook. A requested
//     cancellation stops the solve and makes `OptimizeTrajectory` report
//     failure (`wasCancelled() == true`) instead of exposing the partial
//     iterate as a usable result. The optimization math is unchanged
//     (upstream had the same progress hook; the port previously passed NULL).
// ===========================================================================
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "traj_optimizer.hpp"

namespace planner::dftpav::ported {


namespace plan_manage
{
  /* main planning API */
  bool PolyTrajOptimizer::OptimizeTrajectory(
      const std::vector<Eigen::MatrixXd> &iniStates, const std::vector<Eigen::MatrixXd> &finStates,
      std::vector<Eigen::MatrixXd> &initInnerPts, const Eigen::VectorXd &initTs,
      std::vector<std::vector<Eigen::MatrixXd>> &hPoly_container,std::vector<int> singuls,double now, double help_eps)
  {
    
    trajnum = initInnerPts.size();
    epis = help_eps;
    cancelled_ = false;  // port addition: reset the cancellation state per call
    cfgHs_container = hPoly_container;
    iniState_container = iniStates;
    finState_container = finStates;
    singul_container = singuls;
    // Port addition: keep the prefabricated initial waypoints and junction
    // positions for the anchor cost (see OptimizerConfig::wei_anchor).
    iniInnerPts_container_ = initInnerPts;
    iniGear_container_.clear();
    for (int i = 0; i < trajnum - 1; i++) {
      iniGear_container_.push_back(finState_container[i].col(0));
    }
    variable_num_ = 0;
    jerkOpt_container.clear();
    piece_num_container.clear();
    jerkOpt_container.resize(trajnum);
    piece_num_container.resize(trajnum);
    double final_cost;

    if(initTs.size()!=trajnum){
      std::fprintf(stderr, "[planner_dftpav] initTs.size() != trajnum\n");
      return false;
    }
    if(initTs.minCoeff() < mini_T){
      std::fprintf(stderr, "[planner_dftpav] mini segment T < mini_T\n");
      return false;
    }

    for(int i = 0; i < trajnum; i++){
      //check
      if(initInnerPts[i].cols()==0){
        std::fprintf(stderr, "[planner_dftpav] trajectory has no inner point\n");
        return false;
      }
      int piece_num_ = initInnerPts[i].cols() + 1;
      piece_num_container[i] = piece_num_;
      if(cfgHs_container[i].size()!=(piece_num_ - 2) * (traj_resolution_ + 1) + 2 * (destraj_resolution_ + 1)){
        std::fprintf(stderr, "[planner_dftpav] cfgHs size mismatch\n");
        return false;
      }
      for (int k = 0; k < (piece_num_ - 2) * (traj_resolution_ + 1) + 2 * (destraj_resolution_ + 1); k++)
      {
        cfgHs_container[i][k].topRows<2>().colwise().normalize(); // norm vector outside
      }

      //reset the start end max_vel_
      double max_vel,max_acc = 0.0;
      if(singuls[i] > 0){
        max_vel  = max_forward_vel;
        max_acc = max_forward_acc; 
      }
      else{
        max_vel = max_backward_vel;
        max_acc = max_backward_acc;
      }
      if(iniState_container[i].col(1).norm()>=max_vel){
        iniState_container[i].col(1) = iniState_container[i].col(1).normalized()*(max_vel-1.0e-2);
      }
      if(finState_container[i].col(1).norm()>=max_vel){
        finState_container[i].col(1) = finState_container[i].col(1).normalized()*(max_vel-1.0e-2);
      }

      if(iniState_container[i].col(2).norm()>=max_acc){
        iniState_container[i].col(2) = iniState_container[i].col(2).normalized()*(max_acc-1.0e-2);
      }
      if(finState_container[i].col(2).norm()>=max_acc){
        finState_container[i].col(2) = finState_container[i].col(2).normalized()*(max_acc-1.0e-2);
      }
      

      jerkOpt_container[i].reset(piece_num_);
      variable_num_ += 2 * (piece_num_ - 1);


    }  
    variable_num_ += trajnum;
    variable_num_ += 2 * (trajnum-1);
    variable_num_ += 1 * (trajnum-1);
    // variable_num_ += 2;
    //Waypoints + T + GearPosition + angle
    
    
      

    auto t0 = std::chrono::steady_clock::now();
    auto t1 = t0;
    auto t2 = t0;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_occ, flag_success;
    Eigen::VectorXd x;
    x.resize(variable_num_);
    int offset = 0;
    for(int i = 0; i<trajnum; i++){
      memcpy(x.data()+offset,initInnerPts[i].data(), initInnerPts[i].size() * sizeof(x[0]));
      offset += initInnerPts[i].size();
    }
    Eigen::Map<Eigen::VectorXd> Vt(x.data()+offset, initTs.size());
    RealT2VirtualT(initTs, Vt);

    offset += initTs.size();
    for(int i = 0; i < trajnum-1; i++){
      memcpy(x.data()+offset,finState_container[i].col(0).data(), 2 * sizeof(x[0]));
      offset += 2;
    }
    Eigen::Map<Eigen::VectorXd> angles(x.data()+offset, trajnum-1);
    for(int i = 0; i < trajnum - 1; i++){
      Eigen::Vector2d gearv = finState_container[i].col(1);
      angles[i] = std::atan2(gearv[1],gearv[0]);
    }

    /*for debug*/
    // offset +=  trajnum-1;
    // memcpy(x.data()+offset,finState_container[trajnum-1].col(1).data(), 2 * sizeof(x[0]));

    





    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = memsize;//128
    lbfgs_params.past = past; //3 
    lbfgs_params.g_epsilon = 1.0e-16;
    // lbfgs_params.g_epsilon = 0.1;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.delta = delta;
    lbfgs_params.max_iterations = lbfgs_max_iterations_;
    (void)now;  // upstream wall-clock input (ros::Time) is unused by the port











 
    /* ---------- prepare ---------- */
    iter_num_ = 0;
    flag_force_return = false;
    force_stop_type_ = DONT_STOP;
    flag_still_occ = false;
    flag_success = false;
    /* ---------- optimize ---------- */
    t1 = std::chrono::steady_clock::now();
    int result = lbfgs::lbfgs_optimize(
        x,
        final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        /* Port extension: wire the upstream progress hook (it was dormant in
         * the port) so that a cooperative cancellation can stop the solve. */
        PolyTrajOptimizer::earlyExitCallback,
        this,
        lbfgs_params);
    t2 = std::chrono::steady_clock::now();
    double time_ms = std::chrono::duration<double>(t2 - t1).count() * 1000.0;
    double total_time_ms = std::chrono::duration<double>(t2 - t0).count() * 1000.0;
  


    /* ---------- get result and check collision ---------- */
    if (result == lbfgs::LBFGS_CONVERGENCE ||
        result == lbfgs::LBFGS_CANCELED ||
        result == lbfgs::LBFGS_STOP||result == lbfgs::LBFGSERR_MAXIMUMITERATION)
    {
      flag_force_return = false;
      flag_success = true;

    } 
    else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
      // ROS_WARN("Lbfgs: The line-search routine reaches the maximum number of evaluations.");
      flag_force_return = false;
      flag_success = true;
    }
    else
    {
      std::fprintf(stderr, "[planner_dftpav] solver error. Return = %d, %s. Skip this planning.\n", result, lbfgs::lbfgs_strerror(result));
    }

    
    lbfgs_result_ = result;
    final_cost_ = final_cost;
    if (final_cost >= cost_fail_threshold_) {
      std::fprintf(stderr, "[planner_dftpav] optimization failed: cost %.3f >= %.3f\n", final_cost, cost_fail_threshold_);
      flag_success = false;
    }
    if (cancelled_) {
      /* Port extension: a cooperative cancellation must never be reported as
       * a usable result; the iterate at hand is partial. */
      std::fprintf(stderr, "[planner_dftpav] optimization cancelled by the caller\n");
      flag_success = false;
    }
    return flag_success;
  }


  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad)
  { 

    double total_smcost = 0.0, total_timecost = 0.0, penalty_cost = 0.0;
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    int offset = 0;

    /* Port extension: poll the cooperative cancellation hook before doing any
     * work. Returning a zero cost/gradient keeps the callback contract (the
     * solver consumes both) while `cancelled_` / `force_stop_type_` make the
     * next progress hook return LBFGS_CANCELED and OptimizeTrajectory report
     * failure; the partial iterate is never exposed as a result. */
    if (opt->cancelRequested()) {
      opt->cancelled_ = true;
      opt->force_stop_type_ = STOP_FOR_ERROR;
      grad.setZero();
      return 0.0;
    }

    std::vector<Eigen::Map<const Eigen::MatrixXd>> P_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradP_container;;

    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, opt->piece_num_container[trajid] - 1);
      Eigen::Map<Eigen::MatrixXd>gradP(grad.data()+offset, 2, opt->piece_num_container[trajid] - 1);
      offset += 2 * (opt->piece_num_container[trajid] - 1);
      gradP.setZero();
      P_container.push_back(P);
      gradP_container.push_back(gradP);
    }
    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, opt->trajnum);
    Eigen::Map<Eigen::VectorXd>gradt(grad.data()+offset, opt->trajnum);
    offset += opt -> trajnum;
    Eigen::VectorXd T(opt->trajnum);
    Eigen::VectorXd gradT(opt->trajnum); gradT.setZero();
    opt->VirtualT2RealT(t, T);
    std::vector<double> trajtimes; trajtimes.push_back(0.0);
    //T(i) is sum time of i-segment traj
    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      trajtimes.push_back(T[trajid]);
    }
    // Ini/Fin Gear Pos
    std::vector<Eigen::Map<const Eigen::MatrixXd>> Gear_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradGear_container;
    for(int trajid = 0; trajid < opt->trajnum - 1; trajid++){
      Eigen::Map<const Eigen::MatrixXd> Gear(x.data()+offset, 2, 1);
      Eigen::Map<Eigen::MatrixXd>gradGear(grad.data()+offset, 2, 1);
      offset += 2;
      gradGear.setZero();
      Gear_container.push_back(Gear);
      gradGear_container.push_back(gradGear);
    }
    // Port addition: anchor cost keeps the soft-constrained solution near the
    // prefabricated initial path (waypoints and junction positions).
    double anchor_cost = 0.0;
    if (opt->wei_anchor_ > 0.0) {
      for (int trajid = 0; trajid < opt->trajnum; trajid++) {
        const Eigen::MatrixXd &P0 = opt->iniInnerPts_container_[trajid];
        if (P_container[trajid].rows() == P0.rows() &&
            P_container[trajid].cols() == P0.cols()) {
          anchor_cost += opt->wei_anchor_ * (P_container[trajid] - P0).squaredNorm();
          gradP_container[trajid] += 2.0 * opt->wei_anchor_ * (P_container[trajid] - P0);
        }
      }
      for (int trajid = 0; trajid < opt->trajnum - 1; trajid++) {
        const Eigen::Vector2d delta =
            Gear_container[trajid] - opt->iniGear_container_[trajid];
        anchor_cost += opt->wei_anchor_ * delta.squaredNorm();
        gradGear_container[trajid] += 2.0 * opt->wei_anchor_ * delta;
      }
    }
    //
    Eigen::Map<const Eigen::VectorXd> Angles(x.data()+offset, opt->trajnum-1);
    Eigen::Map<Eigen::VectorXd>gradAngles(grad.data()+offset, opt->trajnum-1);
    gradAngles.setZero();
    
    /*for relax end v?*/
    // offset += opt->trajnum-1;
    // Eigen::Map<const Eigen::MatrixXd> endV (x.data()+offset, 2, 1);
    // Eigen::Map<Eigen::MatrixXd> gradendV (grad.data()+offset, 2, 1);
    // gradendV.setZero();
    


      



    // Eigen::VectorXd gradt;
    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      double smoo_cost;
      Eigen::VectorXd obs_surround_feas_qvar_costs(3);
      obs_surround_feas_qvar_costs.setZero();

      Eigen::MatrixXd IniS,FinS;
      IniS = opt->iniState_container[trajid];
      FinS = opt->finState_container[trajid];

      if(trajid > 0){
        double theta = Angles[trajid-1];
        IniS.col(0) = Gear_container[trajid-1];
        IniS.col(1) = Eigen::Vector2d(-opt->non_sinv*cos(theta), -opt->non_sinv*sin(theta)); 
      }
      if(trajid < opt->trajnum-1){
        double theta = Angles[trajid];
        FinS.col(0) = Gear_container[trajid];
        FinS.col(1) = Eigen::Vector2d(opt->non_sinv*cos(theta), opt->non_sinv*sin(theta));
      }
      //relax end v?
      // if(trajid == opt->trajnum - 1){
      //   FinS.col(1) = endV;
      // }

      opt->jerkOpt_container[trajid].generate(P_container[trajid],T[trajid] / opt->piece_num_container[trajid],IniS,FinS);
      opt->jerkOpt_container[trajid].initSmGradCost(); // Smoothness cost   
      smoo_cost = opt->jerkOpt_container[trajid].getTrajJerkCost();
      opt->addPVAGradCost2CT( obs_surround_feas_qvar_costs, trajid, trajtimes[trajid]); // Time int cost
      //Get gradT gradC
      total_smcost += smoo_cost;
      penalty_cost +=  obs_surround_feas_qvar_costs.sum();
      // std::cout<<"Trajid: "<<trajid<<" penalty: "<<obs_surround_feas_qvar_costs.transpose()<<std::endl;
    }


    for(int trajid = 0; trajid < opt->trajnum; trajid++){
      double time_cost = 0.0;
      Eigen::Matrix<double,2,3> gradIni, gradFin;
      opt->jerkOpt_container[trajid].calGrads_PT(); // gdt gdp gdhead gdtail
      //waypoint
      gradP_container[trajid] = opt->jerkOpt_container[trajid].get_gdP();
      //init Fin
      gradIni = opt->jerkOpt_container[trajid].get_gdHead();
      gradFin = opt->jerkOpt_container[trajid].get_gdTail();
      if(opt->GearOpt){
        if(trajid > 0){
          double theta = Angles[trajid-1];
          gradGear_container[trajid-1] += gradIni.col(0);
          gradAngles[trajid-1] += gradIni.col(1).transpose() * Eigen::Vector2d(opt->non_sinv * sin(theta), -opt->non_sinv*cos(theta));
        }
        if(trajid < opt->trajnum-1){
          double theta = Angles[trajid];
          gradGear_container[trajid] += gradFin.col(0);
          gradAngles[trajid] += gradFin.col(1).transpose() * Eigen::Vector2d(-opt->non_sinv * sin(theta), opt->non_sinv*cos(theta));
        }
      }

      // if(trajid ==  opt->trajnum-1){
      //   gradendV += gradFin.col(1);
      // }

      opt->VirtualTGradCost(T[trajid],t[trajid],opt->jerkOpt_container[trajid].get_gdT() / opt->piece_num_container[trajid],gradt[trajid],time_cost);
      
      // gradt[trajid] = 0.0;
      
      total_timecost += time_cost;
      // std::cout<<"gradp: \n"<<gradP_container[trajid] <<std::endl;
      // std::cout<<"gradeigen p: \n"<<opt->jerkOpt_container[trajid].get_gdP()<<std::endl;
    }

    opt->iter_num_ += 1;
    // std::cout << "grad angle : "<< gradAngles.transpose() << "\n";
    // std::cout << "angle : "<< Angles.transpose() << "\n";
    // std::cout <<"endV: " <<endV.transpose()<<"\n";
    
    




    return total_smcost + total_timecost + penalty_cost + anchor_cost;





  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const Eigen::VectorXd &, const Eigen::VectorXd &,
                                           const double, const double, const int, const int)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    /* Port extension: cooperative cancellation (see PROVENANCE.md). Checked at
     * every L-BFGS progress boundary; returning 1 makes lbfgs_optimize stop
     * with LBFGS_CANCELED. */
    if (opt->cancelRequested())
    {
      opt->cancelled_ = true;
      return 1;
    }

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 + mini_T 
              ? (sqrt(2.0 * RT(i) - 1.0 - 2 * mini_T) - 1.0)
              : (1.0 - sqrt(2.0 / (RT(i)-mini_T) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0) + mini_T
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0) + mini_T;
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }
  void PolyTrajOptimizer::VirtualTGradCost(const double &RT, const double &VT, const double &gdRT, double &gdVT, double& costT){
    double gdVT2Rt;
    if (VT > 0)
    {
      gdVT2Rt = VT + 1.0;
    }
    else
    {
      double denSqrt = (0.5 * VT - 1.0) * VT + 1.0;
      gdVT2Rt = (1.0 - VT) / (denSqrt * denSqrt);
    }

    gdVT = (gdRT + wei_time_) * gdVT2Rt;
    costT = RT * wei_time_;
  }


  void PolyTrajOptimizer::addPVAGradCost2CT(Eigen::VectorXd &costs,  const int trajid, const double trajtime)
  {

    // output gradT gradC 
    int N = piece_num_container[trajid];
    Eigen::Vector2d outerNormal;
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma, ddddsigma;
    double vel2_reci,vel2_reci_e,vel3_2_reci_e,acc2, cur2, cur, phi_dot;
    double latacc2;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 2> gradViolaPc, gradViolaVc, gradViolaAc,gradViolaLatAc, gradViolaKc,gradViolaKLc,gradViolaKRc,gradViolaPhidotLc,gradViolaPhidotRc;
    double gradViolaPt, gradViolaVt, gradViolaAt,gradViolaLatAt, gradViolaKt,gradViolaKLt,gradViolaKRt,gradViolaPhidotLt,gradViolaPhidotRt;
    double violaPos, violaVel, violaAcc, violaLatAcc, violaCur, violaCurL, violaCurR, violaDynamicObs, violaPhidotL, violaPhidotR;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaLatAccPenaD, violaCurPenaD, violaCurPenaDL, violaCurPenaDR,violaDynamicObsPenaD, violaPhidotPenaDL, violaPhidotPenaDR;
    double violaPosPena, violaVelPena, violaAccPena, violaLatAccPena, violaCurPena, violaCurPenaL, violaCurPenaR,violaDynamicObsPena, violaPhidotPenaL, violaPhidotPenaR;
    double phidot_denominator, phidot_nominator;

    double approxcur2, approxviolaCur,approxviolaCurPenaD,approxviolaCurPena;
    Eigen::Matrix<double, 6, 2> gradapproxViolaKc;
    double gradapproxViolaKt;
    
    std::vector<Eigen::MatrixXd> cfgHs = cfgHs_container[trajid];
    int singul_ = singul_container[trajid];
    double max_vel,max_cur,max_acc;
    if(singul_ > 0){
      max_vel = max_forward_vel;
      max_cur = max_forward_cur;
      max_acc = max_forward_acc;
    }
    else{
      max_vel = max_backward_vel;
      max_cur = max_backward_cur;
      max_acc = max_backward_acc;
    }


    double omg;
    int i_dp = 0; // the index of constrain points
    costs.setZero();
    double z_h0, z_h1, z_h2, z_h3, z_h4;
    double z1, z2, z3;
    double n1, n2, n3, n4, n5, n6;
    Eigen::Matrix2d ego_R;
    /* port: upstream visualization/debug collections removed */
    // int innerLoop;
    double t = 0;

    //debug
    double velcost=0.0;
    double acccost=0.0;
    double latacccost = 0.0;
    double curcost=0.0;
    double phidotcost = 0.0;
    int pointid = -1;




    for (int i = 0; i < N; ++i)
    {
      int K;
      if(i==0 || i==N-1){
        K = destraj_resolution_;
      }
      else{
        K = traj_resolution_;
      }
      const Eigen::Matrix<double, 6, 2> &c = jerkOpt_container[trajid].getCoeffs().block<6, 2>(i * 6, 0);
      step = jerkOpt_container[trajid].getDt() / K; // T_i /k
      s1 = 0.0;
      // innerLoop = K;
      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1;
        alpha = 1.0 / K * j;
        
        //update s1 for the next iteration
        s1 += step;
        pointid++;

        sigma = c.transpose() * beta0;
        dsigma = c.transpose() * beta1;
        ddsigma = c.transpose() * beta2;
        dddsigma = c.transpose() * beta3;
        ddddsigma = c.transpose() * beta4;
         
        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        // some help values
        
        z_h0 = dsigma.norm();
        z_h1 = ddsigma.transpose() * dsigma;
        z_h2 = dddsigma.transpose() * dsigma;
        z_h3 = ddsigma.transpose() * B_h * dsigma;

        n1 = z_h0;
        n2 = n1 * n1;
        n3 = n2 * n1;
        n4 = n2 * n2;
        n5 = n3 * n2;
        n6 = n3 * n3;
      
        z1 = dddsigma.transpose() * B_h * dsigma;
        z2 = ddsigma.transpose() * B_h * dsigma;
        z3 = dsigma.transpose() * ddsigma;  
        
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }


        // add cost z_h0 = ||v||
        if ( z_h0 < 1e-4 || (j==0&&i==0) || (i==N-1&&j==K))
        {
          continue;
        }
        //avoid siguality

        vel2_reci = 1.0 / (z_h0 * z_h0);
        vel2_reci_e = 1.0 / (z_h0 * z_h0+epis);
        vel3_2_reci_e = vel2_reci_e * sqrt(vel2_reci_e);
        z_h0 = 1.0 / z_h0;

        z_h4 = z_h1 * vel2_reci;
        violaVel = 1.0 / vel2_reci - max_vel * max_vel;
        acc2 = z_h1 * z_h1 * vel2_reci;
        latacc2 = z_h3 * z_h3 * vel2_reci;
        cur2 = z_h3 * z_h3 * (vel2_reci_e * vel2_reci_e * vel2_reci_e);
        cur = z_h3 * vel3_2_reci_e;
        violaAcc = acc2 - max_acc * max_acc;
        violaLatAcc = latacc2 - max_latacc_ * max_latacc_;

        phidot_denominator = n6 + L_ * L_ * z2 * z2;
        phidot_nominator = L_ * (n3 * z1 - 3 * z2 * z3 * n1);
        phi_dot = phidot_nominator / phidot_denominator; // S/M

        //@hzc: add feasibility with curvature
        violaCur = cur2 - max_cur * max_cur;
        violaCurL = cur-max_cur;
        violaCurR = -cur-max_cur;
        violaPhidotL = phi_dot - max_phidot_;
        violaPhidotR = -phi_dot - max_phidot_;

        ego_R << dsigma(0), -dsigma(1),
                 dsigma(1), dsigma(0);
        ego_R = singul_ * ego_R * z_h0;

        Eigen::Matrix2d temp_a, temp_v;
        temp_a << ddsigma(0), -ddsigma(1),
                  ddsigma(1), ddsigma(0);
        temp_v << dsigma(0), -dsigma(1),
                  dsigma(1), dsigma(0);
        Eigen::Matrix2d R_dot = singul_ * (temp_a * z_h0 - temp_v * vel2_reci * z_h0 * z_h1);

        for(auto le : vec_le_)
        {
          Eigen::Vector2d bpt = sigma + ego_R * le;

          Eigen::Matrix2d temp_l_Bl;
          temp_l_Bl << le(0), -le(1),
                       le(1), le(0);          

          int corr_k = cfgHs[pointid].cols();

          for(int k = 0; k < corr_k; k++)
          {
            outerNormal = cfgHs[pointid].col(k).head<2>();
            violaPos = outerNormal.dot(bpt - cfgHs[pointid].col(k).tail<2>());

            if(violaPos > 0)
            {
              positiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);
              
              gradViolaPc = beta0 * outerNormal.transpose() + 
                            beta1 * outerNormal.transpose() * (singul_ * temp_l_Bl * z_h0 - ego_R * le * dsigma.transpose() * vel2_reci);
              
              gradViolaPt = alpha * outerNormal.transpose() * (dsigma + R_dot * le);

              jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_obs_ * violaPosPenaD * gradViolaPc;
              jerkOpt_container[trajid].get_gdT() += omg * wei_obs_ * (violaPosPenaD * gradViolaPt * step + violaPosPena / K);

              costs(0) += omg * step * wei_obs_ * violaPosPena; // cost is the same
            }
          }
        }
        
        // Dynamic-surround avoidance (upstream `surround_trajs_`) is not part
        // of this port; costs(1) stays zero.
        
        
        
        if (violaVel > 0.0)
        {
          positiveSmoothedL1(violaVel, violaVelPena, violaVelPenaD);

          gradViolaVc = 2.0 * beta1 * dsigma.transpose(); // 6*2
          gradViolaVt = 2.0 * alpha * z_h1;               // 1*1
          jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * violaVelPenaD * gradViolaVc;
          jerkOpt_container[trajid].get_gdT() += omg * wei_feas_ * (violaVelPenaD * gradViolaVt * step + violaVelPena / K);
          costs(2) += omg * step * wei_feas_ * violaVelPena;
          velcost+=omg * step * wei_feas_ * violaVelPena;

        }
        
        if (violaAcc > 0.0)
        {
          positiveSmoothedL1(violaAcc, violaAccPena, violaAccPenaD);
          gradViolaAc = 2.0 * beta1 * (z_h4 * ddsigma.transpose() - z_h4 * z_h4 * dsigma.transpose()) +
                        2.0 * beta2 * z_h4 * dsigma.transpose(); // 6*2
          gradViolaAt = 2.0 * alpha * (z_h4 * (ddsigma.squaredNorm() + z_h2) - z_h4 * z_h4 * z_h1);
          jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * violaAccPenaD * gradViolaAc;
          jerkOpt_container[trajid].get_gdT()  += omg * wei_feas_ * (violaAccPenaD * gradViolaAt * step + violaAccPena / K);
          costs(2) += omg * step * wei_feas_ * violaAccPena;
          acccost += omg * step * wei_feas_ * violaAccPena;
        } 
        // if (violaLatAcc > 0.0)
        // {
        //   positiveSmoothedL1(violaLatAcc, violaLatAccPena, violaLatAccPenaD);
        //   gradViolaLatAc = 2.0 * beta1 * (z_h3 * vel2_reci * ddsigma.transpose() * B_h - z_h3 * vel2_reci  * z_h3 * vel2_reci  * dsigma.transpose()) +
        //                 2.0 * beta2 * z_h3  * vel2_reci * dsigma.transpose() * B_h.transpose(); // 6*2
        //   gradViolaLatAt = 2.0 * alpha * (z_h3  * vel2_reci * z1
        //                     -z_h3 * vel2_reci  * z_h3 * vel2_reci * dddsigma.transpose()*B_h*dsigma);

        //   jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * violaLatAccPenaD * gradViolaLatAc;
        //   jerkOpt_container[trajid].get_gdT()  += omg * wei_feas_ * (violaLatAccPenaD * gradViolaLatAt * step + violaLatAccPena / K);
        //   costs(2) += omg * step * wei_feas_ * violaLatAccPena;
        //   latacccost += omg * step * wei_feas_ * violaLatAccPena;
        // } 

       
        /*violaCurL = cur-max_cur_;
        violaCurR = -cur-max_cur_;*/

        if(violaCurL > 0.0){
          positiveSmoothedL1(violaCurL, violaCurPenaL, violaCurPenaDL);
          //@hzc
          gradViolaKLc = beta1 * (vel3_2_reci_e * ddsigma.transpose()*B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) 
                         + beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose(); // 6*2
          gradViolaKLt  = alpha*vel3_2_reci_e*(dddsigma.transpose()*B_h*dsigma-3*vel2_reci_e*z_h3*z_h1);
          jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * 10.0 * violaCurPenaDL * gradViolaKLc;
          jerkOpt_container[trajid].get_gdT() += omg * wei_feas_ * 10.0 * (violaCurPenaDL * gradViolaKLt * step + violaCurPenaL / K);
          costs(2) += omg * step * wei_feas_ * 10.0 * violaCurPenaL;
          curcost+=omg * step * wei_feas_ * 10.0 * violaCurPenaL;
        }
        if(violaCurR > 0.0){
          positiveSmoothedL1(violaCurR, violaCurPenaR, violaCurPenaDR);
          //@hzc
          gradViolaKRc = -(beta1 * (vel3_2_reci_e * ddsigma.transpose()*B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) 
                         + beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose()); // 6*2
          gradViolaKRt  = -(alpha*vel3_2_reci_e*(dddsigma.transpose()*B_h*dsigma-3*vel2_reci_e*z_h3*z_h1));
          jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * 10.0 * violaCurPenaDR * gradViolaKRc;
          jerkOpt_container[trajid].get_gdT()  += omg * wei_feas_ * 10.0 * (violaCurPenaDR * gradViolaKRt * step + violaCurPenaR / K);
          costs(2) += omg * step * wei_feas_ * 10.0 * violaCurPenaR;
          curcost+=omg * step * wei_feas_ * 10.0 * violaCurPenaR;
        }

        // if(violaPhidotL > 0.0)
        // {
        //   positiveSmoothedL1(violaPhidotL, violaPhidotPenaL, violaPhidotPenaDL);
        //   Eigen::Vector2d partial_S_over_partial_dsigma
        //       = L_ * (n3 * B_h.transpose() * dddsigma + 3 * z1 * n1 * dsigma - 3 * B_h.transpose() * ddsigma * z3 * n1 - 3 * z2 * ddsigma - 3 * z2 * z3 * dsigma / z1);
        //   Eigen::Vector2d partial_M_over_partial_dsigma 
        //       = 6 * n4 * dsigma + 2 * L_ * L_ * z2 * B_h.transpose() * ddsigma;
        //   Eigen::Vector2d partial_S_over_partial_ddsigma
        //       = -3 * L_ * n1 * (B_h * dsigma * z3 + z2 * dsigma);
        //   Eigen::Vector2d partial_M_over_partial_ddsigma
        //       = 2 * L_ * L_ * z2 * B_h * dsigma;
        //   Eigen::Vector2d partial_S_over_partial_dddsigma
        //       = L_ * n3 * B_h * dsigma;

        //   Eigen::Vector2d partial_phi_dot_over_partial_dsigma
        //       = (partial_S_over_partial_dsigma * phidot_denominator - partial_M_over_partial_dsigma * phidot_nominator) / pow(phidot_denominator, 2);
        //   Eigen::Vector2d partial_phi_dot_over_partial_ddsigma
        //       = (partial_S_over_partial_ddsigma * phidot_denominator - partial_M_over_partial_ddsigma * phidot_nominator) / pow(phidot_denominator, 2);
        //   Eigen::Vector2d partial_phi_dot_over_partial_dddsigma
        //       = partial_S_over_partial_dddsigma / phidot_denominator;

        //   gradViolaPhidotLc = beta1 * partial_phi_dot_over_partial_dsigma.transpose()
        //                     + beta2 * partial_phi_dot_over_partial_ddsigma.transpose()
        //                     + beta3 * partial_phi_dot_over_partial_dddsigma.transpose();
        //   gradViolaPhidotLt = alpha * (partial_phi_dot_over_partial_dsigma.transpose() * ddsigma
        //                               +partial_phi_dot_over_partial_ddsigma.transpose() * dddsigma
        //                               +partial_phi_dot_over_partial_dddsigma.transpose() * ddddsigma)(0, 0);

        //   jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * 1.0 * violaPhidotPenaDL * gradViolaPhidotLc;
        //   gdTs[trajid](i) += omg * wei_feas_ * 1.0 * (violaPhidotPenaDL * gradViolaPhidotLt * step + violaPhidotPenaL / K);
        //   costs(2) += omg * step * wei_feas_ * 1.0 * violaPhidotPenaL;
        //   phidotcost+=omg * step * wei_feas_ * 1.0 * violaPhidotPenaL;
        // }

        // if(violaPhidotR > 0.0)
        // {
        //   positiveSmoothedL1(violaPhidotR, violaPhidotPenaR, violaPhidotPenaDR);
        //   Eigen::Vector2d partial_S_over_partial_dsigma
        //       = L_ * (n3 * B_h.transpose() * dddsigma + 3 * z1 * n1 * dsigma - 3 * B_h.transpose() * ddsigma * z3 * n1 - 3 * z2 * ddsigma * n1 - 3 * z2 * z3 * dsigma / n1);
        //   Eigen::Vector2d partial_M_over_partial_dsigma 
        //       = 6 * n4 * dsigma + 2 * L_ * L_ * z2 * B_h.transpose() * ddsigma;
        //   Eigen::Vector2d partial_S_over_partial_ddsigma
        //       = -3 * L_ * n1 * (B_h * dsigma * z3 + z2 * dsigma);
        //   Eigen::Vector2d partial_M_over_partial_ddsigma
        //       = 2 * L_ * L_ * z2 * B_h * dsigma;
        //   Eigen::Vector2d partial_S_over_partial_dddsigma
        //       = L_ * n3 * B_h * dsigma;

        //   Eigen::Vector2d partial_phi_dot_over_partial_dsigma
        //       = (partial_S_over_partial_dsigma * phidot_denominator - partial_M_over_partial_dsigma * phidot_nominator) / pow(phidot_denominator, 2);
        //   Eigen::Vector2d partial_phi_dot_over_partial_ddsigma
        //       = (partial_S_over_partial_ddsigma * phidot_denominator - partial_M_over_partial_ddsigma * phidot_nominator) / pow(phidot_denominator, 2);
        //   Eigen::Vector2d partial_phi_dot_over_partial_dddsigma
        //       = partial_S_over_partial_dddsigma / phidot_denominator;

        //   gradViolaPhidotRc = -beta1 * partial_phi_dot_over_partial_dsigma.transpose()
        //                     - beta2 * partial_phi_dot_over_partial_ddsigma.transpose()
        //                     - beta3 * partial_phi_dot_over_partial_dddsigma.transpose();
        //   gradViolaPhidotRt = -alpha * (partial_phi_dot_over_partial_dsigma.transpose() * ddsigma
        //                               +partial_phi_dot_over_partial_ddsigma.transpose() * dddsigma
        //                               +partial_phi_dot_over_partial_dddsigma.transpose() * ddddsigma)(0, 0);

        //   jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * 1.0 * violaPhidotPenaDR * gradViolaPhidotRc;
        //   gdTs[trajid](i) += omg * wei_feas_ * 1.0 * (violaPhidotPenaDR * gradViolaPhidotRt * step + violaPhidotPenaR / K);
        //   costs(2) += omg * step * wei_feas_ * 1.0 * violaPhidotPenaR;
        //   phidotcost+=omg * step * wei_feas_ * 1.0 * violaPhidotPenaR;
        // }
      }
      t += jerkOpt_container[trajid].getDt();
    }


  }


  ///viola*, viola*Pena, viola*PenaD
  void PolyTrajOptimizer::positiveSmoothedL1(const double &x, double &f, double &df)
  {
        const double pe = 1.0e-4;
        const double half = 0.5 * pe;
        const double f3c = 1.0 / (pe * pe);
        const double f4c = -0.5 * f3c / pe;
        const double d2c = 3.0 * f3c;
        const double d3c = 4.0 * f4c;

        if (x < pe)
        {
            f = (f4c * x + f3c) * x * x * x;
            df = (d3c * x + d2c) * x * x;
        }
        else
        {
            f = x - half;
            df = 1.0;
        }
    //       df = x * x;
    // f = df *x;
    // df *= 3.0;
        return;
  }
  /* helper functions */
  // Port replacement for upstream
  //   setParam(ros::NodeHandle nh, planning::minco::Config cfg_)
  // The ROS publishers and protobuf reads are gone; all parameters come from
  // the plain OptimizerConfig value. The vehicle geometry must be configured
  // through setVehicleParam() before calling this function.
  void PolyTrajOptimizer::setParam(const OptimizerConfig& cfg)
  {
    traj_resolution_ = cfg.traj_resolution;
    destraj_resolution_ = cfg.destraj_resolution;
    wei_obs_ = cfg.wei_sta_obs;
    wei_feas_ = cfg.wei_feas;
    wei_time_ = cfg.wei_time;
    half_margin = cfg.half_margin;
    max_phidot_ = cfg.max_phidot;
    max_forward_vel = cfg.max_forward_vel;
    max_backward_vel = cfg.max_backward_vel;
    max_forward_cur = cfg.max_forward_cur;
    max_backward_cur = cfg.max_backward_cur;
    max_forward_acc = cfg.max_forward_acc;
    max_backward_acc = cfg.max_backward_acc;
    max_latacc_ = cfg.max_latacc;
    GearOpt = cfg.gearopt;
    memsize = cfg.lbfgs_memsize;
    past = cfg.lbfgs_past;
    delta = cfg.lbfgs_delta;
    mini_T = cfg.mini_t;
    non_sinv = cfg.non_sinv;
    wei_anchor_ = cfg.wei_anchor;
    lbfgs_max_iterations_ = cfg.lbfgs_max_iterations;
    cost_fail_threshold_ = cfg.cost_fail_threshold;

    B_h << 0, -1,
           1, 0;

    // upstream inflated the vehicle rectangle by 2 * half_margin
    veh_param_.set_width(veh_param_.width() + 2 * half_margin);
    veh_param_.set_length(veh_param_.length() + 2 * half_margin);

    double half_wid = 0.5 * veh_param_.width();
    double half_len = 0.5 * veh_param_.length();

    L_ = veh_param_.wheel_base();
    lz_set_.clear();
    lz_set_.push_back(Eigen::Vector2d(  half_len,  half_wid));
    lz_set_.push_back(Eigen::Vector2d(  half_len, -half_wid));
    lz_set_.push_back(Eigen::Vector2d( -half_len, -half_wid));
    lz_set_.push_back(Eigen::Vector2d( -half_len,  half_wid));

    Eigen::Vector2d le_1, le_2, le_3, le_4;  // vertices of the ego car in the body frame
    vec_le_.clear();
    le_1 << veh_param_.d_cr() + veh_param_.length() / 2.0, veh_param_.width() / 2.0;
    le_2 << veh_param_.d_cr() + veh_param_.length() / 2.0, -veh_param_.width() / 2.0;
    le_3 << veh_param_.d_cr() - veh_param_.length() / 2.0, -veh_param_.width() / 2.0;
    le_4 << veh_param_.d_cr() - veh_param_.length() / 2.0, veh_param_.width() / 2.0;

    // attention here! These vectors store one more of the vertices! The
    // vertices are stored Clockwise!
    vec_le_.push_back(le_1);
    vec_le_.push_back(le_2);
    vec_le_.push_back(le_3);
    vec_le_.push_back(le_4);
    vec_le_.push_back(le_1);
  }

} // namespace plan_manage

}  // namespace planner::dftpav::ported

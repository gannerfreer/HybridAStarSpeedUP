#include "user_astar.hpp"
#include "hybrid_astar.h"
#include "motion.hpp"
#include "reeds_shepp.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
namespace {
using namespace planner;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
void run() {
 astar::PlannerConfig config;astar::HybridAStar core(config);astar::HybridSearchResult raw;
 require(!core.Search({0,0,0},{1,0,0},&raw) && raw.message.find("hook")!=std::string::npos,"missing hooks must reject search");
 core.SetPoseCollisionHook([](const astar::Pose2D&){return false;});
 core.SetGoalHook([](const astar::Pose2D&){return true;});
 require(!core.Search({0,0,0},{1,0,0},&raw),"missing motion check must reject search");
 core.SetMotionCollisionHook([](const astar::Pose2D&,const astar::Pose2D&){return false;});
 core.SetAbortCallback([]{return true;});
 require(!core.Search({0,0,0},{1,0,0},&raw) && raw.message=="aborted","core cancellation");
 common::VehicleModel vehicle;vehicle.wheelbase_m=2.5;vehicle.width_m=1.8;vehicle.front_overhang_m=0.8;vehicle.rear_overhang_m=0.8;vehicle.max_steer_rad=0.6;
 collision::Scene scene;auto& grid=scene.grid;grid.map_id="regression";grid.map_version="1";grid.width=80;grid.height=80;grid.resolution_m=0.5;grid.cells.assign(6400,0);
 collision::CollisionChecker checker(vehicle,scene);hybrid_astar::UserAstar planner;
 common::PlanRequest request;request.map_id=grid.map_id;request.map_version=grid.map_version;request.vehicle=vehicle;request.start={10,10,0};request.goal={6,10,0};
 auto result=planner.plan(request,grid,checker);
 require(result.status==common::Status::kSuccess && checker.check_path(result.path).is_clear(),"reverse planning");
 bool reverse=false;for(const auto& segment:result.path)for(const auto& p:segment.points)reverse|=p.direction==common::Direction::kReverse;
 require(reverse,"reverse motion required");
 request.cancelled=true;require(planner.plan(request,grid,checker).status==common::Status::kCancelled,"request cancellation");request.cancelled=false;
 planner.set_cancel_callback([]{return true;});require(planner.plan(request,grid,checker).status==common::Status::kCancelled,"callback cancellation");planner.set_cancel_callback({});
 request.goal={30,10,0};request.node_budget=1;require(planner.plan(request,grid,checker).status==common::Status::kTimeout,"node budget");request.node_budget.reset();
 request.time_budget_s=1e-12;require(planner.plan(request,grid,checker).status==common::Status::kTimeout,"time budget");request.time_budget_s.reset();
 request.start={30,10,common::kPi};request.goal={10,10,common::kPi};
 auto plus=planner.plan(request,grid,checker);request.start.yaw_rad=-common::kPi;request.goal.yaw_rad=-common::kPi;
 auto minus=planner.plan(request,grid,checker);
 require(plus.status==common::Status::kSuccess && minus.status==common::Status::kSuccess,"pi boundary planning");
 require(checker.check_path(plus.path).is_clear() && checker.check_path(minus.path).is_clear(),"pi boundary collision");
 require(std::abs(plus.path.back().points.back().s_m-minus.path.back().points.back().s_m)<1e-9,"pi paths must agree");
 // 对外 RS 积分应使用共享内核；包括微小曲率与倒车。
 rs::RsSegment segment;segment.length_m=2;segment.curvature_inv_m=1e-12;segment.direction=common::Direction::kReverse;
 const auto sampled=rs::integrate_segment({2,3,0.7},segment,2);
 const auto expected=math::integrate_arc({2,3,0.7},-2,1e-12);
 require(std::hypot(sampled.x_m-expected.x_m,sampled.y_m-expected.y_m)<1e-12,"RS integration must share math");
}
}
int main(){try{run();std::cout<<"PASS: search hooks, reverse, cancellation, budgets, pi boundary and RS integration\n";return EXIT_SUCCESS;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return EXIT_FAILURE;}}

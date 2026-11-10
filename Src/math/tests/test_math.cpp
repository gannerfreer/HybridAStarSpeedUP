#include "geometry.hpp"
#include "geometry_internal.hpp"
#include "motion.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

namespace {
using namespace planner;
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
void near(double a, double b, double tolerance = 1e-11) {
  require(std::isfinite(a) && std::isfinite(b) && std::abs(a-b) <= tolerance, "numeric mismatch");
}
void angles() {
  near(math::normalize_angle_rad(common::kPi), -common::kPi, 0);
  near(math::normalize_angle_rad(-common::kPi), -common::kPi, 0);
  for (double a : {0., -0., 1., -1., 100., -100., 1e12, -1e12}) {
    const double out = math::normalize_angle_rad(a);
    require(out >= -common::kPi && out < common::kPi, "angle range");
    near(math::normalize_angle_rad(out), out, 0);
  }
  require(std::isnan(math::normalize_angle_rad(std::numeric_limits<double>::infinity())), "nonfinite angle");
}
void arcs() {
  std::mt19937 engine(42);
  std::uniform_real_distribution<double> distribution(-3., 3.);
  for (int i=0; i<600; ++i) {
    common::Pose2d start{distribution(engine), distribution(engine), distribution(engine)};
    const double s=distribution(engine), k=i%3==0 ? 1e-12 : (i%3==1 ? 0. : distribution(engine));
    const auto out=math::integrate_arc(start,s,k);
    const auto cached=math::integrate_arc_trig(start,s,k,std::cos(start.yaw_rad),std::sin(start.yaw_rad));
    near(out.x_m,cached.x_m,0);near(out.y_m,cached.y_m,0);near(out.yaw_rad,cached.yaw_rad,0);
    // 独立的长双精度中点公式，避免差分正余弦在近直线时相消。
    const long double h=static_cast<long double>(k)*s/2;
    const long double scale=h==0 ? 1 : std::sin(h)/h;
    near(out.x_m,static_cast<double>(start.x_m+s*scale*std::cos(start.yaw_rad+h)));
    near(out.y_m,static_cast<double>(start.y_m+s*scale*std::sin(start.yaw_rad+h)));
    const auto back=math::integrate_arc(out,-s,k);
    near(back.x_m,start.x_m);near(back.y_m,start.y_m);
    near(math::normalize_angle_rad(back.yaw_rad-start.yaw_rad),0.);
  }
  const auto straight=math::integrate_arc({0,0,0},-5,0);
  near(straight.x_m,-5);near(straight.y_m,0);
}
void geometry() {
  const auto polygon=math::rectangle_polygon(0,0,2,2);
  require(math::convex_polygon_valid(polygon),"valid rectangle");
  require(!math::convex_polygon_valid({{0,0},{1,0},{2,0}}),"degenerate polygon");
  require(math::convex_polygon_intersects_aabb(polygon,{2,0,3,1}),"touch counts as intersection");
  near(math::point_polygon_distance_sq({3,3},polygon),2.);
  near(math::point_segment_distance_sq({2,2},{0,0},{0,0}),8.);
  near(math::aabb_distance({0,0,1,1},{2,2,3,3}),std::sqrt(2.));
  const common::Pose2d pose{3,-2,0.7};const common::Point2d p{1,4};
  const auto restored=math::to_body_frame(pose,math::transform_point(pose,p));
  near(restored.x_m,p.x_m);near(restored.y_m,p.y_m);
  std::mt19937 engine(17);std::uniform_real_distribution<double> distribution(-5,5);
  for (int i=0;i<500;++i) {
    auto rotated=polygon;
    const common::Pose2d transform{distribution(engine),distribution(engine),distribution(engine)};
    for(auto& point:rotated)point=math::transform_point(transform,point);
    const double x=distribution(engine),y=distribution(engine);
    const math::Aabb2d box{x,y,x+1,y+1};
    math::internal::PolygonSatCache8 cache;cache.build(rotated.data(),rotated.size());
    const bool reference=math::internal::convex_polygon_intersects_aabb_normalized(rotated.data(),rotated.size(),box,1e-10);
    require(math::convex_polygon_intersects_aabb(rotated,box,1e-10)==reference,"fast/reference SAT mismatch");
    require(cache.maybe_intersects(box,1e-10)==reference,"cached/reference SAT mismatch");
    const double distance=math::point_polygon_distance({x,y},rotated);
    near(distance*distance,math::point_polygon_distance_sq({x,y},rotated),1e-10);
  }
  // 超出平方域快速路径的坐标仍需安全回退，保持相交/分离判定。
  const auto huge=math::rectangle_polygon(0,0,1e80,1e80);
  math::internal::PolygonSatCache8 cache;cache.build(huge.data(),huge.size());
  require(!cache.fast_domain,"must select numerical fallback");
  require(cache.maybe_intersects({5e79,5e79,2e80,2e80},0),"fallback intersection");
  require(!cache.maybe_intersects({2e80,2e80,3e80,3e80},0),"fallback separation");
  std::array<common::Point2d,8> input{{{0,0},{1,0},{1,1},{0,1},{0,0},{0.5,0.5},{1,0},{0,1}}},out;
  require(math::convex_hull_small(input,8,out)==4,"small hull duplicates");
  std::vector<common::Point2d> many(input.begin(),input.end());
  many.push_back({0.2,0.2});many.push_back({0.7,0.7});
  require(math::convex_hull(many).size()==4,"generic hull duplicates");
  require(math::convex_hull({}).empty(),"empty hull");
}
}
int main() {
 try {angles();arcs();geometry();std::cout<<"PASS: angles, stable arcs, geometry and SAT fallback\n";return EXIT_SUCCESS;}
 catch(const std::exception& e){std::cerr<<e.what()<<'\n';return EXIT_FAILURE;}
}

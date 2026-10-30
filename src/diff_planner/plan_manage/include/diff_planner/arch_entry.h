#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace diff_planner {
namespace arch_entry {

struct Config {
  double width = 1.50, height = 1.50;
  double range = 4.0, min_progress = .55, max_lateral = 1.40;
  double half_fov = 50.0 * M_PI / 180.0;
  double min_structure_z = .20, max_z = 1.90;
  double goal_z = .60, min_goal_z = .45, max_goal_z = .78;
  double body_radius = .25, body_below = .20, body_above = .10;
  double tolerance = .11;
};

struct Result {
  bool found = false;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double yaw = 0.0, radius = 0.0, spring_z = 0.0;
  double score = -std::numeric_limits<double>::infinity();
  int support = 0;
};

// 同一门平面的机体通行窗口，禁止把被板材/箱子堵住的拱形轮廓当成洞口。
inline bool openingClear(const std::vector<Eigen::Vector3d>& cloud,
                         const Eigen::Vector3d& center, double yaw, const Config& c) {
  const Eigen::Vector3d forward(std::cos(yaw),std::sin(yaw),0);
  const Eigen::Vector3d side(-forward.y(),forward.x(),0);
  for (const auto& p : cloud) {
    if (!p.allFinite()) continue;
    const Eigen::Vector3d delta=p-center;
    if (std::fabs(delta.dot(forward))<=.15 && std::fabs(delta.dot(side))<=c.body_radius &&
        delta.z()>=-c.body_below && delta.z()<=c.body_above) return false;
  }
  return true;
}

// 独立的拱形截面匹配：下方两根竖边 + 同一截面的左右弧肩及弧顶。
// 只用有点云支持的轮廓，不把两个相邻拱门之间的空白当成入口。
inline Result detect(const std::vector<Eigen::Vector3d>& cloud,
                     const Eigen::Vector3d& origin, double heading, const Config& c) {
  Result best;
  constexpr double res = .10;
  const double base = c.goal_z-.5*c.height;
  const double spring = c.goal_z+.5*c.height-.5*c.width;
  const double visible_post_bottom = std::max(base+.10,c.min_structure_z+.05);
  const double lateral_limit = c.max_lateral+c.width;
  const int cols = static_cast<int>(std::ceil(2*lateral_limit/res))+1;
  const int heights = static_cast<int>(std::ceil((c.max_z-base)/res))+1;
  if (heights < 4 || c.width <= 0 || c.height < .5*c.width) return best;
  const int rows = static_cast<int>(std::ceil(c.range/res))+1;
  for (double offset = -c.half_fov; offset <= c.half_fov+1e-6; offset += 10*M_PI/180) {
    const double yaw = heading+offset;
    const Eigen::Vector3d forward(std::cos(yaw),std::sin(yaw),0);
    const Eigen::Vector3d side(-forward.y(),forward.x(),0);
    std::vector<std::vector<Eigen::Vector3d>> slices(rows);
    for (const auto& p : cloud) {
      if (!p.allFinite() || p.z() < c.min_structure_z || p.z() > c.max_z) continue;
      const double d = (p-origin).dot(forward), y = (p-origin).dot(side);
      if (d < 0 || d > c.range || std::fabs(y) > lateral_limit) continue;
      const int row = static_cast<int>(std::round(d/res));
      // 一层相邻体素补齐薄框的深度离散，不混入更深的另一道门。
      for (int r = std::max(0,row-1); r <= std::min(rows-1,row+1); ++r)
        slices[r].emplace_back(d,y,p.z());
    }
    for (int row = static_cast<int>(std::ceil(c.min_progress/res)); row < rows; ++row) {
      if (slices[row].size() < 12) continue;
      std::vector<unsigned char> occupied(cols*heights,0);
      for (const auto& p : slices[row]) {
        const int col = static_cast<int>(std::round((p.y()+lateral_limit)/res));
        const int z = static_cast<int>(std::round((p.z()-base)/res));
        if (col >= 0 && col < cols && z >= 0 && z < heights) occupied[col*heights+z] = 1;
      }
      const auto distance = [&](double y, double z) {
        const int col = static_cast<int>(std::round((y+lateral_limit)/res));
        const int iz = static_cast<int>(std::round((z-base)/res));
        double nearest = std::numeric_limits<double>::infinity();
        for (int dc=-1; dc<=1; ++dc) for (int dz=-1; dz<=1; ++dz) {
          const int x=col+dc, h=iz+dz;
          if (x<0 || x>=cols || h<0 || h>=heights || !occupied[x*heights+h]) continue;
          nearest=std::min(nearest,std::hypot(x*res-lateral_limit-y,base+h*res-z));
        }
        return nearest;
      };
      std::vector<double> posts;
      for (int col=1; col<cols-1; ++col) {
        const double y=col*res-lateral_limit;
        int hits=0;
        for (double z=visible_post_bottom; z<=spring+1e-6; z+=res)
          if (distance(y,z)<=c.tolerance) ++hits;
        if (hits>=3) posts.push_back(y);
      }
      for (size_t a=0; a<posts.size(); ++a) for (size_t b=a+1; b<posts.size(); ++b) {
        const double observed_width=posts[b]-posts[a], center=.5*(posts[a]+posts[b]), radius=.5*c.width;
        if (std::fabs(observed_width-c.width)>2*c.tolerance || std::fabs(center)>c.max_lateral) continue;
        {
          int arc_hits=0, sectors[3]={0,0,0}; double error=0;
          bool left_shoulder=false, right_shoulder=false;
          for (int i=1; i<12; ++i) {
            const double angle=i*M_PI/12;
            const double e=distance(center+radius*std::cos(angle),spring+radius*std::sin(angle));
            if (e<=c.tolerance) {
              ++arc_hits; ++sectors[i<=3?0:(i>=9?2:1)]; error+=e;
              if (i==3) right_shoulder=true;
              if (i==9) left_shoulder=true;
            }
          }
          // 45度弧肩必须有证据，避免竖柱端部加平横梁在容差内冒充半圆。
          if (arc_hits<8 || sectors[0]<2 || sectors[1]<3 || sectors[2]<2 ||
              !left_shoulder || !right_shoulder) continue;
          int post_hits=0, post_total=0;
          for (double z=visible_post_bottom; z<=spring+1e-6; z+=res) {
            ++post_total;
            if (distance(center-radius,z)<=c.tolerance && distance(center+radius,z)<=c.tolerance) ++post_hits;
          }
          if (post_hits<3 || post_hits<.75*post_total || radius<=c.body_radius) continue;
          const double roof=spring+std::sqrt(radius*radius-c.body_radius*c.body_radius);
          const double z=std::min(std::max(c.goal_z,c.min_goal_z),
                                  std::min(c.max_goal_z,roof-c.body_above-c.tolerance));
          if (z<c.min_goal_z || z-c.body_below<base) continue;
          // 切片厚度只用于容纳体素误差，入口点深度取实测轮廓的中位数，而非切片前沿。
          std::vector<double> depths;
          for (const auto& p : slices[row]) {
            const double y=p.y()-center;
            const bool post=p.z()>=visible_post_bottom-c.tolerance && p.z()<=spring &&
                std::fabs(std::fabs(y)-radius)<=c.tolerance;
            const bool arc=p.z()>=spring &&
                std::fabs(std::hypot(y,p.z()-spring)-radius)<=c.tolerance;
            if (post || arc) depths.push_back(p.x());
          }
          if (depths.empty()) continue;
          auto middle=depths.begin()+depths.size()/2;
          std::nth_element(depths.begin(),middle,depths.end());
          Eigen::Vector3d target=origin+(*middle)*forward+center*side; target.z()=z;
          // 同一门平面优先更完整且拟合更好的轮廓，再优先较近的入口。
          const double score=arc_hits-error*10-.5*std::fabs(center)-.2*row*res;
          const double range=(target-origin).head<2>().norm();
          const double best_range=(best.center-origin).head<2>().norm();
          if (best.found && (range>best_range+.30 ||
              (range>=best_range-.30 && score<=best.score))) continue;
          if (!openingClear(cloud,target,yaw,c)) continue;
          best.found=true; best.center=target; best.yaw=yaw; best.radius=radius;
          best.spring_z=spring; best.score=score; best.support=arc_hits+2*post_hits;
        }
      }
    }
  }
  return best;
}

}  // namespace arch_entry
}  // namespace diff_planner

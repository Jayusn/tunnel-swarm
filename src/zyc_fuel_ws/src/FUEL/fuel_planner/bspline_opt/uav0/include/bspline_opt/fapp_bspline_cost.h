#pragma once
#include <bspline_opt/fapp_dynamic_obstacle.h>
#include <vector>
#include <algorithm>
#include <cmath>
namespace fuel_fapp {
inline double bsplineCost(const std::vector<Eigen::Vector3d>& q, double dt,
    const std::vector<DynamicObstacleState>& obstacles, const DynamicObstacleConfig& config,
    double start, double reference, std::vector<Eigen::Vector3d>& gradient, double& time_gradient) {
  gradient.assign(q.size(), Eigen::Vector3d::Zero()); time_gradient = 0.0;
  if (!config.enabled || q.size()<4 || dt<=0.0) return 0.0;
  double cost=0.0;
  // Fixed normalized samples keep the objective and its time derivative consistent.
  for (size_t k=0; k+3<q.size(); ++k) for (int sample=0; sample<=10; ++sample) {
    const double u=sample/10.0, u2=u*u, u3=u2*u;
    const double w[4]={(1-3*u+3*u2-u3)/6, (4-6*u2+3*u3)/6,
                       (1+3*u+3*u2-3*u3)/6, u3/6};
    Eigen::Vector3d point=Eigen::Vector3d::Zero();
    for (int j=0;j<4;++j) point+=w[j]*q[k+j];
    for (const auto& obstacle:obstacles) {
      double clearance; Eigen::Vector3d normal,velocity;
      if (!evaluateDynamicObstacle(point,obstacle,start+(k+u)*dt,reference,
                                   config,&clearance,&normal,nullptr,&velocity)) continue;
      // A small soft clearance outside the padded body avoids boundary tangency.
      const double error=0.05-clearance;
      if (error<=0.0) continue;
      cost+=error*error/10.0;
      for (int j=0;j<4;++j) gradient[k+j]-=0.2*error*w[j]*normal;
      time_gradient+=0.2*error*normal.dot(velocity)*(k+u);
    }
  }
  return cost;
}
}

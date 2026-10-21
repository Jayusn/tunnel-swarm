#undef NDEBUG
#include <cassert>
#include <use-ikfom.hpp>

using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
namespace {
int calls = 0;
int valid_calls = 100;
bool constant_residual = false;
void measurement(state_ikfom& s, esekfom::dyn_share_datastruct<double>& d) {
  ++calls;
  d.valid = calls <= valid_calls;
  if (!d.valid) return;
  d.h_x = Eigen::MatrixXd::Zero(30, 12);
  d.h = Eigen::VectorXd::Zero(30);
  for (int i=0;i<30;++i) {
    d.h_x(i,i%3)=1.;
    d.h(i)=constant_residual ? 1. : -s.pos[i%3];
  }
}
void init(Filter& f, int iterations=3) {
  double limits[23]; std::fill(limits,limits+23,1e-3);
  f.init_dyn_share(get_f,df_dx,df_dw,measurement,iterations,limits);
  auto s=f.get_x(); s.pos=Eigen::Vector3d(.2,-.3,.5); s.vel=Eigen::Vector3d(.1,.2,.3);
  f.change_x(s);
  Filter::cov p=Filter::cov::Identity();
  p(2,14)=p(14,2)=.15; p(14,20)=p(20,14)=.1;
  f.change_P(p);
}
}
void ObservationDoesNotChangeStateOrCovariance() {
  Filter plain,observed;init(plain);init(observed);
  valid_calls=100;constant_residual=false;
  double solve1=0,solve2=0;calls=0;
  plain.update_iterated_dyn_share_modified(.001,solve1);
  esekfom::UpdateDiagnostics d;calls=0;
  observed.update_iterated_dyn_share_modified(.001,solve2,&d);
  Eigen::Matrix<double,23,1> delta;
  observed.get_x().boxminus(delta,plain.get_x());
  assert((delta.norm())<(1e-14));
  assert((observed.get_P().array()==plain.get_P().array()).all());
  assert((d.iterations)==(static_cast<unsigned>(calls)));
  assert((d.iterations)==(d.valid_iterations));
  assert(d.final_measurement_valid);
  assert(d.covariance_updated);
  assert(d.last_increment_converged);
}
void InvalidMeasurementsResetCountersAndLeaveCovarianceAlone() {
  Filter f;init(f); const auto p=f.get_P(); const auto s=f.get_x();
  valid_calls=0;calls=0;constant_residual=false;
  esekfom::UpdateDiagnostics d;d.valid_iterations=90;d.covariance_updated=true;
  double solve=0;f.update_iterated_dyn_share_modified(.001,solve,&d);
  assert((d.iterations)==(4U));assert((d.valid_iterations)==(0U));
  assert(!(d.final_measurement_valid));assert(!(d.covariance_updated));
  assert(!(d.last_increment_converged));
  assert((p.array()==f.get_P().array()).all());
  Eigen::Matrix<double,23,1> delta;f.get_x().boxminus(delta,s);
  assert((delta.norm())==(0.));
}
void InvalidFinalMeasurementIsNotReportedAsCompletedUpdate() {
  Filter f;init(f);valid_calls=1;calls=0;constant_residual=false;
  double solve=0;esekfom::UpdateDiagnostics d;
  f.update_iterated_dyn_share_modified(.001,solve,&d);
  assert((d.iterations)==(4U));assert((d.valid_iterations)==(1U));
  assert(!(d.final_measurement_valid));assert(!(d.covariance_updated));
}
void ForcedRematchingIsNotNumericalConvergence() {
  Filter f;init(f,1);valid_calls=100;calls=0;constant_residual=true;
  double solve=0;esekfom::UpdateDiagnostics d;
  f.update_iterated_dyn_share_modified(.001,solve,&d);
  assert((d.iterations)==(2U));assert(d.covariance_updated);
  assert(!(d.last_increment_converged));
}
int main() {
  ObservationDoesNotChangeStateOrCovariance();
  InvalidMeasurementsResetCountersAndLeaveCovarianceAlone();
  InvalidFinalMeasurementIsNotReportedAsCompletedUpdate();
  ForcedRematchingIsNotNumericalConvergence();
}

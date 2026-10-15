#!/usr/bin/env python3
"""Compile the production propagation functions and test delayed scan handoff."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ImuRepropagationTest(unittest.TestCase):
    def test_delayed_correction_replays_history_and_reseeds_boundary(self):
        source = (ROOT / 'src/laserMapping.cpp').read_text()
        functions = source[source.index('void predictIMUState('):
                           source.index('void fastPredictIMU(')]
        preamble = r'''
#include <Eigen/Geometry>
#include <so3_math.h>
#include <deque>
#include <memory>
#include <cassert>
#include <cmath>
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
constexpr double G_m_s2 = 9.81;
struct Stamp { double value; double toSec() const { return value; } };
struct Vector { double x=0, y=0, z=0; };
struct Imu { struct { Stamp stamp; } header; Vector linear_acceleration, angular_velocity; };
using ImuPtr = std::shared_ptr<Imu>;
struct { std::deque<ImuPtr> imu; } Measures;
std::deque<ImuPtr> imu_buffer;
struct { V3D pos, vel, ba, bg, grav; M3D rot; } state_point;
struct ImuProcess { double GetMeanAccNorm() const { return 1.1; } } process;
ImuProcess *p_imu = &process;
double latest_time, lidar_end_time;
V3D latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
M3D latest_Q;
ImuPtr sample(double t) {
    auto m=std::make_shared<Imu>(); m->header.stamp.value=t;
    m->linear_acceleration.z=(G_m_s2+2.0+3.0*t+0.2)*1.1/G_m_s2;
    m->angular_velocity.z=std::sin(45*t)+0.07;
    return m;
}
'''
        checks = r'''
int main() {
    // A 152.5ms backlog with changing angular rate, nonzero biases, and a
    // correction timestamp between two IMU samples (not at an IMU boundary).
    lidar_end_time=0.0025;
    state_point.pos=V3D(1,2,3); state_point.vel=V3D(0,0,0.4);
    state_point.rot=M3D::Identity(); state_point.ba=V3D(0,0,0.2);
    state_point.bg=V3D(0,0,0.07); state_point.grav=V3D(0,0,-G_m_s2);
    Measures.imu.push_back(sample(0));
    for(int i=1;i<=31;++i) imu_buffer.push_back(sample(i*.005));
    // Deliberately poison the old predictor's endpoint IMU: reset must not use it.
    latest_acc_0=V3D(100,100,100); latest_gyr_0=V3D(20,20,20);
    updateLatestStates();
    double yaw=0, previous_t=lidar_end_time;
    double previous_w=0.5*std::sin(45*.005);
    for(int i=1;i<=31;++i) {
        double t=i*.005, w=std::sin(45*t);
        yaw+=.5*(previous_w+w)*(t-previous_t);
        previous_t=t; previous_w=w;
    }
    const M3D expected=Eigen::AngleAxisd(yaw,V3D::UnitZ()).toRotationMatrix();
    assert((latest_Q-expected).norm()<1e-12);
    const double end=.155, start=.0025;
    const double vz=.4+2*(end-start)+1.5*(end*end-start*start);
    const double pz=3+.4*(end-start)+(1+1.5*start)*(end-start)*(end-start)
                    +.5*std::pow(end-start,3);
    assert(std::abs(latest_time-end)<1e-12);
    assert(std::abs(latest_V.z()-vz)<1e-12);
    assert(std::abs(latest_P.z()-pz)<3e-6);
    assert(imu_buffer.size()==31); // the mapping EKF still needs its IMU history
    auto next=sample(.160);
    predictIMUState(.160,V3D(0,0,next->linear_acceleration.z),V3D(0,0,next->angular_velocity.z));
    yaw+=.5*(std::sin(45*.155)+std::sin(45*.160))*.005;
    assert((latest_Q-Eigen::AngleAxisd(yaw,V3D::UnitZ()).toRotationMatrix()).norm()<1e-12);
    const M3D saved=latest_Q; const V3D saved_p=latest_P;
    predictIMUState(.155,V3D(99,99,99),V3D(99,99,99));
    assert((latest_Q-saved).norm()==0 && (latest_P-saved_p).norm()==0);
    // No backlog, exact scan boundary: seed from the scan's final IMU.
    imu_buffer.clear(); Measures.imu.clear(); Measures.imu.push_back(sample(.2));
    lidar_end_time=.2; latest_acc_0.setConstant(99); latest_gyr_0.setConstant(99);
    updateLatestStates();
    assert(std::abs(latest_time-.2)<1e-12);
    assert(std::abs(latest_gyr_0.z()-sample(.2)->angular_velocity.z)<1e-12);
    assert(std::abs(latest_acc_0.z()-(G_m_s2+2+3*.2+.2))<1e-12);
    // Exact boundary also works with future buffered samples.
    imu_buffer.push_back(sample(.205)); updateLatestStates();
    assert(std::abs(latest_time-.205)<1e-12);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            cpp = Path(tmp) / 'test.cpp'
            exe = Path(tmp) / 'test'
            cpp.write_text(preamble + functions + checks)
            subprocess.run(['c++', '-std=c++14', '-O2', '-I/usr/include/eigen3',
                            '-I' + str(ROOT / 'include'), str(cpp), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()

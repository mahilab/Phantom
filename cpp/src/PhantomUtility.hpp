#pragma once
#include <Eigen/Dense>
#include <Mahi/Util.hpp>

namespace Phantom {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using mahi::util::DEG2RAD;

/// Integrator
template <typename T>
class Integrator {
public:
    Integrator(T initial) : val(initial) { }
    inline T integrate(T dval_dt, double dt) {
        val += dt * 0.5 * (m_prev_dval_dt + dval_dt);
        m_prev_dval_dt = dval_dt;
        return val;
    }
    T val;
private:
    T m_prev_dval_dt;
};

/// Cartesian point
struct Point {
    double x,y,z;
};

/// Inertia tensor
struct Tensor {
    double xx, xy, xz, yy, yz, zz;
};

} // namespace Phantom
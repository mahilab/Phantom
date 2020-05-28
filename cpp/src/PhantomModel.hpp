#pragma once
#include "PhantomUtility.hpp"

namespace Phantom {
namespace Model {

//=============================================================================
// CONSTANTS
//=============================================================================

/// Rigidbody masses [kg]
constexpr double m_a  = 0.026220; 
constexpr double m_c  = 0.026146; 
constexpr double m_be = 0.267840;
constexpr double m_df = 0.215420;
constexpr double m_g  = 0.361590;

/// Rigidbody inertia tensors [kg*m^2]
constexpr Tensor Ic_a  = { 6.029e-05,         0,          0,    5.7e-07,          0, 6.004e-05 };
constexpr Tensor Ic_c  = {0.00013177,         0,          0,    8.9e-07,          0, 0.00013138};
constexpr Tensor Ic_be = { 0.0001133,   4.1e-07, -2.296e-05, 0.00091667,    4.8e-07, 0.00082222};
constexpr Tensor Ic_df = { 0.0001214,  -3.2e-07,  -7.45e-06, 0.00026133,   3.23e-06, 0.00015823};
constexpr Tensor Ic_g  = { 0.0015202, 5.838e-05, 0.00018147,  0.0012112, 0.00023832, 0.00093887};

/// Rigidbody centers of mass [m]
constexpr Point Pc_a  = { 0.000024230000000, -0.052378920000000,                  0};
constexpr Point Pc_c  = {                 0,  0.104777390000000,                  0};
constexpr Point Pc_be = {-0.035188750000000, -0.000062800000000,  0.002324160000000};
constexpr Point Pc_df = {-0.051535410000000,  0.000609030000000,  0.004028730000000};
constexpr Point Pc_g  = {-0.010485630000000,  0.013963870000000, -0.053808270000000};

/// Relevant link lengths (see diagram) [m]
constexpr double l1 = 0.209550;
constexpr double l2 = 0.169545;
constexpr double l3 = 0.031750;

/// Joint limits [rad]
constexpr double Q_min[3] = {-90*DEG2RAD, -124*DEG2RAD, -29*DEG2RAD}; 
constexpr double Q_max[3] = { 90*DEG2RAD,  119*DEG2RAD, 214*DEG2RAD};
constexpr double Q23_max  = 65 * DEG2RAD;
constexpr double Q32_max  = 55 * DEG2RAD;

/// Damping coefficients [Nm*s/rad]
constexpr double B_coef[3] = {0.0005, 0.0005, 0.0005};

/// Kinetic friction [Nm]
constexpr double Fk_coef[3] = {0.01, 0.01, 0.01};

/// Transmission Ratios [unitless]
constexpr double Eta[3] = {13.3, 11.2, 11.2};

/// Motor Nominal Torque [Nm]
constexpr double Tau_mot_nom = 0.0286;

/// Motor Max Torque [Nm]
constexpr double Tau_mot_max = 0.129;

/// Joint Max Torque [Nm]
constexpr double Tau_nom[3] = {Tau_mot_nom * Eta[0], Tau_mot_nom * Eta[1], Tau_mot_nom * Eta[2]};

/// Joint Max Torque [Nm]
constexpr double Tau_max[3] = {Tau_mot_max * Eta[0], Tau_mot_max * Eta[1], Tau_mot_max * Eta[2]};

/// Motor Rotor Inertia [kg-m^2]
constexpr double J_mot = 1.08e-06;

/// Motor Torque Constant [Nm/A]
constexpr double Kt = 0.0234;

/// Gravity
constexpr double g = 9.80665;

//=============================================================================
// KINEMATICS
//=============================================================================

/// Validates a set of Phantom joint angles.
bool validate_angles(const Vector3d& Q);

/// Returns a cartesian position in frame {0} given a set of valid joint angles.
Vector3d forward_kinematics(const Vector3d& Q);

/// Returns the joint angles given a cartesian position in frame {0}
Vector3d inverse_kinematics(const Vector3d& P, const Vector3d Q_prev);

//=============================================================================
// JACOBIAN
//=============================================================================

/// Returns the Phantom Jacobian evaluated at the joint angles Q.
Matrix3d J(const Vector3d& Q);

/// Computes torques required to generate forces at end effector: Tau = J(Q)' * F
Vector3d forces_to_torques(const Vector3d& F, const Vector3d& Q);

//=============================================================================
// DYNAMICS
//=============================================================================

/// Phantom dynamic state
struct State {
    State() { Q=Qd=Qdd=Tau=Vector3d::Zero(); }
    Vector3d Q;   // joint angles                [rad]
    Vector3d Qd;  // joint angular velocities    [rad/s]
    Vector3d Qdd; // joint angular accelerations [rad/s^2]
    Vector3d Tau; // joint torques               [Nm]
};

/// Compute Phantom mass matrix
Matrix3d M(const Vector3d& Q);

/// Computes Phantom coriolis vector
Vector3d V(const Vector3d& Q, const Vector3d& Qd);

/// Computes Phantom gravity vector
Vector3d G(const Vector3d& Q);

/// Computes Phantom kinetic energies
Eigen::Matrix<double,5,1> K(const Vector3d& Q, const Vector3d& Qd);

/// Computes Phantom potential energies
Eigen::Matrix<double,5,1> U(const Vector3d& Q);

/// Steps Phantom dynamics forward in time (includes reflected motor inertia, damping, and friction)
void step_dynamics(State& s, double dt);

/// Steps Phantom dynamics forward in time according to Cavusoglu et al. 2001
void step_dynamics_cavusoglu(State& s, double dt);

} // namespace Model
} // namespace Phantom
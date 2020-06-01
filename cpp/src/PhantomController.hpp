#pragma once
#include "PhantomInterface.hpp"
#include <thread>
#include <mutex>
#include <atomic>

namespace Phantom {

//=============================================================================
// CONTROL LAW
//=============================================================================

struct ControlLaw {
    virtual ~ControlLaw() { }
    /// Return joint torques given joint positions and velocities and delta time
    virtual Vector3d control(Vector3d Q, Vector3d , double dt) = 0;
};

struct JointSpaceTorque : public ControlLaw {
    JointSpaceTorque() { Tau.fill(0); }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) override {
        return Tau;
    }
    Vector3d Tau;
};

struct JointSpacePD : public ControlLaw {
    JointSpacePD() { Kp.fill(10); Kd.fill(0.5f); Q_ref.fill(0); }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) override {
        return Kp.cwiseProduct(Q_ref - Q) - Kd.cwiseProduct(Qd);               
    }
    Vector3d Kp;
    Vector3d Kd;
    Vector3d Q_ref;    
};

struct TaskSpaceForce : public ControlLaw {
    TaskSpaceForce() { gcomp = true; F.fill(0); }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) {
        Vector3d G = gcomp ? Model::G(Q) : Vector3d::Zero();
        return G + Model::forces_to_torques(F,Q);
    }
    bool gcomp;
    Vector3d F;
};

struct TaskSpacePD : public ControlLaw {
    TaskSpacePD() { gcomp = true; Kp.fill(100); Kd.fill(5); P_ref.fill(0); }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) {
        Vector3d G = gcomp ? Model::G(Q) : Vector3d::Zero();
        Matrix3d Kp_diag = Kp.asDiagonal();
        Matrix3d Kd_diag = Kd.asDiagonal();
        Matrix3d Kpq = Model::J(Q).transpose() * Kp_diag * Model::J(Q);
        Matrix3d Kdq = Model::J(Q).transpose() * Kd_diag * Model::J(Q);
        Vector3d Q_ref = Model::inverse_kinematics(P_ref, Q);
        return G + Kpq * (Q_ref - Q) - Kdq * (Qd);       
    }
    bool gcomp;
    Vector3d Kp;
    Vector3d Kd;
    Vector3d P_ref;
};

struct TaskSpaceEulerPD : public ControlLaw {
    TaskSpaceEulerPD() { gcomp = true; Kp.fill(100); Kd.fill(5); P_ref.fill(0); EulerAngles.fill(0);}
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) {
        Vector3d G = gcomp ? Model::G(Q) : Vector3d::Zero();
        Matrix3d Rx, Ry, Rz;
        Rx << 1, 0, 0, 0, mahi::util::cos(EulerAngles(0)), -mahi::util::sin(EulerAngles(0)), 0, mahi::util::sin(EulerAngles(0)), mahi::util::cos(EulerAngles(0));
        Ry << mahi::util::cos(EulerAngles(1)), 0, mahi::util::sin(EulerAngles(1)), 0, 1, 0, -mahi::util::sin(EulerAngles(1)), 0, mahi::util::cos(EulerAngles(1));
        Rz << mahi::util::cos(EulerAngles(2)), -mahi::util::sin(EulerAngles(2)), 0, mahi::util::sin(EulerAngles(2)), mahi::util::cos(EulerAngles(2)), 0, 0, 0, 1;
        auto R = Rx*Ry*Rz;
        auto J_rot = R*Model::J(Q);
        Matrix3d Kp_diag = Kp.asDiagonal();
        Matrix3d Kd_diag = Kd.asDiagonal();
        Matrix3d Kpq = J_rot.transpose() * Kp_diag * J_rot;
        Matrix3d Kdq = J_rot.transpose() * Kd_diag * J_rot;
        Vector3d Q_ref = Model::inverse_kinematics(P_ref, Q);
        return G + Kpq * (Q_ref - Q) - Kdq * (Qd);       
    }
    bool gcomp;
    Vector3d EulerAngles;
    Vector3d Kp;
    Vector3d Kd;
    Vector3d P_ref;
};

//=============================================================================
// CONTROLLER
//=============================================================================

class Controller {
public:
    Controller(Ptr<Interface> interface, 
                      Ptr<ControlLaw> law = nullptr,
                      Frequency rate = 1000_Hz) 
        : m_interface(std::move(interface)), m_law(std::move(law)), m_rate(rate), m_running(false)
    { }
    /// Start the controller
    void start() {
        if (!m_running) {
            m_running = true;
            m_thread = std::thread(&Phantom::Controller::thread_func, this);
        }
    }
    /// Stop the controller
    void stop() {
        m_running = false;
        if (m_thread.joinable()) {
            m_thread.join();
        }
    }
    /// Change the control law
    void set_law(Ptr<ControlLaw> law) {
        Lock lock(m_mtx);
        m_law = std::move(law);
    }
     /// Gets the Phantom joint angles [rad]
    virtual Vector3d get_positions() const {
        Lock lock(m_mtx);
        return m_interface->get_positions();
    }
    /// Gets the Phantom joint velocities [rad/s]
    virtual Vector3d get_velocities() const {
        Lock lock(m_mtx);
        return m_interface->get_velocities();
    }  
    /// Gets the Phantom joint velocities [rad/s]
    virtual Vector3d get_torques() const {
        Lock lock(m_mtx);
        return m_interface->get_torques();
    }  
    /// Acquire controller thread mutex 
    inline Lock get_lock() {
        return Lock(m_mtx);
    }
private:
    void thread_func() {
        m_interface->start();
        Timer timer(m_rate);
        Clock clk;
        while (m_running) {
            {
                Lock lock(m_mtx);
                auto Q = m_interface->get_positions();
                auto Qd = m_interface->get_velocities();
                double dt = clk.restart().as_seconds();
                Vector3d Tau = m_law ? m_law->control(Q,Qd,dt) : Vector3d::Zero();
                m_interface->set_torques(Tau);
            }
            timer.wait();
        }
        m_interface->set_torques(Vector3d::Zero());
        m_interface->stop();
    }
private:
    Ptr<Interface>  m_interface;
    Ptr<ControlLaw> m_law;
    Frequency m_rate;
    std::thread m_thread;
    std::atomic_bool m_running;
    mutable std::mutex m_mtx;
};

} // namespace Phantom

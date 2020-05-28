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
    TaskSpacePD() { gcomp = true; Kp.fill(10); Kd.fill(0.5f); P_ref.fill(0); }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) {
        Vector3d G = gcomp ? Model::G(Q) : Vector3d::Zero();
        auto P = Model::forward_kinematics(Q);
        auto V = Model::J(Q) * Qd;
        auto F = Kp.cwiseProduct(P_ref - P) - Kd.cwiseProduct(V);               
        return G + Model::forces_to_torques(F,Q);
    }
    bool gcomp;
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

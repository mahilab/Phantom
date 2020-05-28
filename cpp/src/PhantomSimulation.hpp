#pragma once
#include "PhantomInterface.hpp"


namespace Phantom {

class Simulation : public Interface {
public:
    /// Constructor
    Simulation() : m_running(false) { }
    /// Destructor
    ~Simulation() { }
    /// Starts running the Phantom simulation
    virtual void start() override {
        if (!m_running) {
            m_state = Model::State();
            m_running = true;
            m_thread = std::thread(&Phantom::Simulation::thread_func, this);
        }
    }
    /// Stops running the Phantom simulation
    virtual void stop() override {
        m_running = false;
        if (m_thread.joinable())
            m_thread.join();
    }
    /// Sets the Phantom joint torques [Nm]
    virtual void set_torques(const Vector3d& Tau) override {
        Lock lock(m_mtx);
        m_state.Tau = Tau;
    }
    /// Gets the Phantom joint angles [rad]
    virtual Vector3d get_positions() override {
        Lock lock(m_mtx);
        return m_state.Q;
    }
    /// Gets the Phantom joint velocities [rad/s]
    virtual Vector3d get_velocities() override {
        Lock lock(m_mtx);
        return m_state.Qd;
    }
    /// Gets the Phantom joint Torques [Nm]
    virtual Vector3d get_torques() override {
        Lock lock(m_mtx);
        return m_state.Tau;
    }
private:
    /// Simulation thread implementation
    virtual void thread_func() {
        Timer timer(mahi::util::hertz(1000), Timer::Hybrid);
        Clock clk;
        while (m_running) {
            {
                Lock lock(m_mtx);
                Phantom::Model::step_dynamics(m_state, clk.restart().as_seconds());
            }
            timer.wait();
        }
    }
private:
    Model::State m_state;
    std::thread m_thread;
    std::atomic_bool m_running;
    mutable std::mutex   m_mtx;
};

} // namespace Phantom

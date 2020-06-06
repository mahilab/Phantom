#pragma once
#include "PhantomControlLaws.hpp"
#include <thread>
#include <mutex>
#include <atomic>

namespace Phantom {

using mahi::util::RingBuffer;
using mahi::util::Severity;

/// Indices for Phantom control data
enum DataIndex : std::size_t {
    Data_Time = 0, 
    Data_Q0,   Data_Q1,   Data_Q2, 
    Data_Qd0,   Data_Qd1,  Data_Qd2,
    Data_Tau0, Data_Tau1, Data_Tau2, 
    Data_X, Data_Y, Data_Z,
    Data_COUNT
};

/// The number of samples to store for each DataIndex
constexpr std::size_t DATA_BUFFER_SIZE = 20000;

class Controller {
public:

    Controller(Ptr<Interface> phantom = nullptr, 
                      Ptr<ControlLaw> law = nullptr,
                      Frequency rate = 2000_Hz) 
        : m_phantom(std::move(phantom)), m_law(std::move(law)), 
          m_rate(rate), m_running(false), m_data(Data_COUNT, RingBuffer<double>(DATA_BUFFER_SIZE)), m_timer(rate)
    { }
    ~Controller() {
        stop();
    }
    /// Start the controller (thread safe)
    bool start() {
        if (!m_running && m_phantom != nullptr) {
            m_running = true;
            m_thread = std::thread(&Phantom::Controller::thread_func, this);
            return true;
        }
        return false;
    }
    /// Stop the controller (thread safe)
    bool stop() {
        if (m_running) { 
            m_running = false;
            if (m_thread.joinable()) {
                m_thread.join();
            }
            clear_data();
            return true;
        }
        return false;
    }
    /// Set the Phantom interface to be controlled (thread safe)
    bool set_phantom(Ptr<Phantom::Interface> phantom) {
        if (!m_running) {
            m_phantom = phantom;
            return true;
        }
        return false;
    }
    /// Change the control law (threaf safe)
    void set_law(Ptr<ControlLaw> law) {
        Lock lock(m_mtx);
        m_law = std::move(law);
    }
     /// Gets the Phantom joint angles [rad] (thread safe)
    virtual Vector3d get_positions() const {
        if (!m_running)
            return Vector3d::Zero();
        Lock lock(m_mtx);
        return m_phantom->get_positions();
    }
    /// Gets the Phantom joint velocities [rad/s] (thread safe)
    virtual Vector3d get_velocities() const {
        if (!m_running)
            return Vector3d::Zero();
        Lock lock(m_mtx);
        return m_phantom->get_velocities();
    }  
    /// Gets the Phantom joint velocities [rad/s] (thread safe)
    virtual Vector3d get_torques() const {
        if (!m_running)
            return Vector3d::Zero();
        Lock lock(m_mtx);
        return m_phantom->get_torques();
    }  
    /// Return true if the Controller is running (thread safe)
    bool running() {
        return m_running;
    }
    /// Copy the last n samples of data up to DATA_BUFFER_SIZE. Return number samples copied. (thread safe)
    int get_data(DataIndex idx, double* buffer, std::size_t n) {
        if (!m_running)
            return 0;
        Lock lock(m_mtx);
        std::size_t avail = m_data[idx].size();
        std::size_t start =  n > avail ? 0 : avail - n;
        int j = 0;
        for (std::size_t i = start; i < avail; ++i) 
            buffer[j++] = m_data[idx][i];        
        return j;
    }
    /// Clears the data buffers
    void clear_data() {
        Lock lock(m_mtx);
        for (auto& d : m_data)
            d.clear();
    }
    /// Acquire controller thread mutex (thread safe)
    inline Lock get_lock() {
        return Lock(m_mtx);
    }
    /// Get a pointer to the Phantom this controller manages (use with get_lock)
    Ptr<Phantom::Interface> get_phantom() {
        return m_phantom;
    }
private:
    void thread_func() {
        m_phantom->start();
        m_timer = Timer(m_rate);
        Clock clk;
        double t = 0;
        while (m_running) {
            {
                Lock lock(m_mtx);
                auto Q = m_phantom->get_positions();
                auto Qd = m_phantom->get_velocities();
                Vector3d XYZ = Phantom::Model::forward_kinematics(Q);
                double dt = clk.restart().as_seconds();
                Vector3d Tau = m_law ? m_law->control(Q,Qd,dt) : Vector3d::Zero();
                m_phantom->set_torques(Tau);
                m_data[Data_Time].push_back(t);
                m_data[Data_Q0].push_back(Q[0]);
                m_data[Data_Q1].push_back(Q[1]);
                m_data[Data_Q2].push_back(Q[2]);
                m_data[Data_Qd0].push_back(Qd[0]);
                m_data[Data_Qd1].push_back(Qd[1]);
                m_data[Data_Qd2].push_back(Qd[2]);
                m_data[Data_Tau0].push_back(Tau[0]);
                m_data[Data_Tau1].push_back(Tau[1]);
                m_data[Data_Tau2].push_back(Tau[2]);
                m_data[Data_X].push_back(XYZ[0]);
                m_data[Data_Y].push_back(XYZ[1]);
                m_data[Data_Z].push_back(XYZ[2]);
            }
            m_timer.wait();
            t = m_timer.get_elapsed_time_ideal().as_seconds();
            m_timer.get_misses();
        }
        m_phantom->set_torques(Vector3d::Zero());
        m_phantom->stop();
        LOG(Severity::Info) << "[Phantom] Controller timer misses:     " << m_timer.get_misses();
        LOG(Severity::Info) << "[Phantom] Controller timer wait ratio: " << m_timer.get_wait_ratio();
    }
private:
    std::vector<RingBuffer<double>> m_data;
    Ptr<Interface>  m_phantom;
    Ptr<ControlLaw> m_law;
    Frequency m_rate;
    std::thread m_thread;
    std::atomic_bool m_running;
    mutable std::mutex m_mtx;
    Timer m_timer;
};

} // namespace Phantom

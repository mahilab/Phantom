#pragma once
#include "PhantomInterface.hpp"
#include <Mahi/Daq.hpp>

using namespace mahi::daq;

namespace Phantom {

class Hardware : public Interface {
public:

    Hardware() : m_q8(false) { }
    ~Hardware() { 
        stop(); 
    }
    /// Starts running the Phantom
    virtual void start() {
        QuanserOptions opts;
        m_q8.open();
        for (int i = 0; i < 3; ++i) {
            opts.encX_dir[i] = QuanserOptions::EncoderDirection::Reversed;
            m_q8.encoder.modes.write(i, QuadMode::X4);
            m_q8.encoder.units.set(i, 2 * PI / 1024);
            m_q8.DO.enable_values[i] = TTL_HIGH;   
            m_q8.DO.disable_values[i] = TTL_LOW;         
        }    
        m_q8.set_options(opts);   
        m_q8.enable();
        m_q8.encoder.zero();
    }
    /// Stops running the Phantom
    virtual void stop() {
        m_q8.disable();
        m_q8.close();
    }
    /// Sets the Phantom joint torques [Nm]
    virtual void set_torques(const Vector3d& Tau) {
        Vector3d Tau_clamp = Tau;
        Model::clamp_torques_nom(Tau_clamp);
        for (int i = 0; i < 3; ++i)
            m_q8.AO[i] = ((Tau_clamp[i] / Model::Eta[i]) / Model::Kt) * m_command_gain * m_directions[i];
        m_q8.AO.write();
    }
    /// Gets the Phantom joint angles [rad]
    virtual Vector3d get_positions() {
        m_q8.encoder.read();
        Vector3d Q;
        for (int i = 0; i < 3; ++i)
            Q[i] = m_q8.encoder.positions[i] / Model::Eta[i];
        return Q;
    }
    /// Gets the Phantom joint velocities [rad/s]
    virtual Vector3d get_velocities() {
        m_q8.read_all();
        Vector3d Qd;   
        for (int i = 0; i < 3; ++i) 
            Qd[i] = m_q8.velocity.velocities[i] / Model::Eta[i];
        return Qd;
    }
    /// Gets the Phantom joint Torques [rad/s]
    virtual Vector3d get_torques() {
        Vector3d Tau;   
        for (int i = 0; i < 3; ++i) 
            Tau[i] = m_q8.AO[i];
        return Tau;
    }
private:
    Q8Usb m_q8;
    static constexpr double m_command_gain  = 10.0 / 1.5f; // [V/A]
    static constexpr double m_directions[3] = {1,-1,-1};
};

}

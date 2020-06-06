#pragma once
#include "PhantomInterface.hpp"
#include <Mahi/Daq.hpp>
#include "Iir.h"

using namespace mahi::daq;

namespace Phantom {

// template <int T>
// class StupidFilter  {
// public:
//     StupidFilter() { 
//         for (int i = 0; i < T; ++i)
//             s[i] = 0;
//     }
//     double filter(double in) {
//         s[T-1] = in;
//         double sum = s[0];
//         for (int i = 1; i < T; ++i) {
//             sum += s[i];
//             s[i-1] = s[i];
//         }
//         return sum / T;
//     }
// private:
//     double s[T];
// };

template <typename TDaq>
class Hardware : public Interface {
public:

    Hardware() : m_daq(false) { 
        for (int i = 0; i < 3; ++i) {
            m_butts[i].configure(2, hertz(m_cutoff), 2000_Hz);
            m_iir[i].setup(2000, m_cutoff);
        }
    }
    ~Hardware() { 

        stop(); 
    }
    /// Starts running the Phantom
    virtual void start() {
        QuanserOptions opts;
        m_daq.open();
        m_daq.DO.set_channels({0,1,2});
        for (int i = 0; i < 3; ++i) {
            opts.encX_dir[i] = QuanserOptions::EncoderDirection::Reversed;
            m_daq.encoder.modes.write(i, QuadMode::X4);
            m_daq.encoder.units.set(i, 2 * PI / 1024);
            m_daq.DO.enable_values[i] = TTL_HIGH;   
            m_daq.DO.disable_values[i] = TTL_LOW;         
        }    
        m_daq.set_options(opts);   
        m_daq.enable();
        m_daq.encoder.zero();
    }
    /// Stops running the Phantom
    virtual void stop() {
        m_daq.disable();
        m_daq.close();
    }
    /// Sets the Phantom joint torques [Nm]
    virtual void set_torques(const Vector3d& Tau) {
        Vector3d Tau_clamp = Tau;
        Model::clamp_torques_nom(Tau_clamp);
        for (int i = 0; i < 3; ++i)
            m_daq.AO[i] = ((Tau_clamp[i] / Model::Eta[i]) / Model::Kt) * m_command_gain * m_directions[i];
        m_daq.AO.write();
    }
    /// Gets the Phantom joint angles [rad]
    virtual Vector3d get_positions() {
        static mahi::util::Clock clk;
        m_daq.encoder.read();
        Vector3d Q;
        for (int i = 0; i < 3; ++i) {
            Q[i] = m_daq.encoder.positions[i] / Model::Eta[i];
            m_diff[i].update(Q[i], clk.get_elapsed_time());            
        }
        return Q;
    }
    /// Gets the Phantom joint velocities [rad/s]
    virtual Vector3d get_velocities() {
        if (m_use_hw_vel)
            m_daq.velocity.read();
        Vector3d Qd;   
        for (int i = 0; i < 3; ++i) {
            double v = m_diff[i].get_value();
            Qd[i] = m_use_hw_vel ? m_daq.velocity.velocities[i] / Model::Eta[i] : m_filter ? Qd[i] = m_iir[i].filter(v) : v;
        }
        return Qd;
    }
    /// Gets the Phantom joint torques [Nm]
    virtual Vector3d get_torques() {
        Vector3d Tau;   
        for (int i = 0; i < 3; ++i) 
            Tau[i] = m_daq.AO[i]; // TODO: conversion
        return Tau;
    }
    /// GUI
    virtual void imgui() override {
        ImGui::Checkbox("Use Hardware Velocity", &m_use_hw_vel);
        if (!m_use_hw_vel) {
            ImGui::Checkbox("Filter Velocity", &m_filter);
            if (ImGui::DragDouble("Filter Cutoff", &m_cutoff, 1, 10, 999)) {
                for (int i = 0; i < 3; ++i)
                    m_iir[i].setup(2000,m_cutoff);
            }
        }
    }
private:
    TDaq m_daq;
    static constexpr double m_command_gain  = 10.0 / 1.5f; // [V/A]
    static constexpr double m_directions[3] = {1,-1,-1};
    mahi::util::Differentiator m_diff[3];
    // filters 
    mahi::util::Butterworth    m_butts[3];
    Iir::Butterworth::LowPass<2> m_iir[3];
    double m_cutoff = 100;
    bool m_filter    = true;
    bool m_use_hw_vel = false;
    // StupidFilter<16> m_stupid[3];
};

}

#include "Phantom.hpp"
#include <Mahi/Util.hpp>
#include <thread>
#include <mutex>
#include <atomic>

using mahi::util::Timer;
using mahi::util::Clock;

#define EXPORT extern "C" __declspec(dllexport)

Phantom g_phantom;
std::thread g_thread;
std::mutex g_mtx;
std::atomic_bool g_stop;

void simulation() {
    Timer timer(mahi::util::hertz(1000), Timer::Hybrid);
    Clock clk;
    while (!g_stop) {
        {
            std::lock_guard<std::mutex> lock(g_mtx);
            g_phantom.update(clk.restart().as_seconds());
        }
        timer.wait();
    }
}

EXPORT void stop() {
    g_stop = true;
    if (g_thread.joinable())
        g_thread.join();
}

EXPORT void start() {
    stop();
    g_phantom = Phantom();
    g_stop = false;
    g_thread = std::thread(simulation);
}

EXPORT void set_torques(double* Tau) {
    std::lock_guard<std::mutex> lock(g_mtx);
    for (int i = 0; i < 3; ++i)
        g_phantom.Tau[i] = Tau[i];
}

EXPORT void get_positions(double* Q) {
    std::lock_guard<std::mutex> lock(g_mtx);
    for (int i = 0; i < 3; ++i)
        Q[i] = g_phantom.Q[i];
}

EXPORT void get_fk(double* ee_pos) {
    std::lock_guard<std::mutex> lock(g_mtx);
    std::vector<double> ee_pos_vec = g_phantom.fk(g_phantom.Q);
    for (int i = 0; i < 3; ++i)
        ee_pos[i] = ee_pos_vec[i];
}
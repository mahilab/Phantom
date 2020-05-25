#include "Phantom.hpp"
#include <Mahi/Util.hpp>
#include <Mahi/Gui.hpp>
#include <thread>
#include <mutex>
#include <atomic>

using namespace mahi::gui;
using mahi::util::Timer;
using mahi::util::Clock;

#define EXPORT extern "C" __declspec(dllexport)

Phantom g_phantom;
std::thread g_thread;
std::mutex g_mtx;
std::atomic_bool g_stop_sim;

void simulation_thread() {
    Timer timer(mahi::util::hertz(1000), Timer::Hybrid);
    Clock clk;
    while (!g_stop_sim) {
        {
            std::lock_guard<std::mutex> lock(g_mtx);
            g_phantom.update2(clk.restart().as_seconds());
        }
        timer.wait();
    }
}

EXPORT void stop() {
    g_stop_sim = true;
    if (g_thread.joinable())
        g_thread.join();
}

EXPORT void start() {
    stop();
    g_phantom = Phantom();
    g_stop_sim = false;
    g_thread = std::thread(simulation_thread);
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
    std::copy(ee_pos_vec.begin(),ee_pos_vec.end(),ee_pos);
}

EXPORT void get_ik(const double* ee_pos, double* theta_d) {
    Point ee_point{ee_pos[0], ee_pos[1], ee_pos[2]};
    std::lock_guard<std::mutex> lock(g_mtx);
    std::vector<double> theta_d_ = g_phantom.ik(ee_point);
    std::copy(theta_d_.begin(),theta_d_.end(),theta_d);
}

class Tuner : public Application {
public:
    Tuner() : Application() { }
    void update() override {
        static bool keep_open = true;
        ImGui::Begin("Test",&keep_open);
        {
            std::lock_guard<std::mutex> lock(g_mtx);
            ImGui::DragDouble("K Hardstop", &g_phantom.Khard, 1, 0, 100);
            ImGui::DragDouble("B Hardstop", &g_phantom.Bhard, 1, 0, 100);

        }
        ImGui::End();
        if (!keep_open)
            quit();
    }
};

std::unique_ptr<Tuner> g_tuner = nullptr;

void tuner_thread() {
    g_tuner = std::make_unique<Tuner>();
    g_tuner->run();
    g_tuner = nullptr;
}

EXPORT bool open_tuner() 
{
    if (g_tuner == nullptr) {
        auto thrd = std::thread(tuner_thread);
        thrd.detach();
        return true;
    }
    return false;
}
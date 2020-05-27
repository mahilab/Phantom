#include "PhantomModel.hpp"
#include <Mahi/Util.hpp>
#include <Mahi/Gui.hpp>
#include <thread>
#include <mutex>
#include <atomic>

using namespace Phantom;
using namespace mahi::gui;
using mahi::util::Timer;
using mahi::util::Clock;

#define EXPORT extern "C" __declspec(dllexport)

Model::State g_phantom;
std::thread g_thread;
std::mutex g_mtx;
std::atomic_bool g_stop_sim;

bool cavusoglu  = false;
bool control = false;
bool jacobian = false;
Vector3d q_d = Vector3d::Zero();
std::vector<double> input = {0,0,0};
std::vector<double> kps = { 10, 10, 10};
std::vector<double> kds = {0.5,0.5,0.5};
std::vector<double> torques = {0,0,0};

void simulation_thread() {
    Timer timer(mahi::util::hertz(1000), Timer::Hybrid);
    Clock clk;
    while (!g_stop_sim) {
        {
            std::lock_guard<std::mutex> lock(g_mtx);
            if(control){
                Point ee_point{input[0],input[1],input[2]};
                Vector3d q_curr = {g_phantom.Q[0],g_phantom.Q[1],g_phantom.Q[2]};
                q_d = Phantom::Model::inverse_kinematics(ee_point,q_curr);
                for(size_t i = 0; i < 3; ++i){
                    g_phantom.Tau[i] = kps[i]*(q_d[i] - g_phantom.Q[i]) - kds[i]*g_phantom.Qd[i];
                }
            }
            else if (jacobian) {
                Vector3d F(&input[0]);
                g_phantom.Tau = Phantom::Model::G(g_phantom.Q) + Phantom::Model::forces_to_torques(F, g_phantom.Q);
            } 
            if (cavusoglu)
                Model::step_dynamics_cavusoglu(g_phantom, clk.restart().as_seconds());
            else
                Model::step_dynamics(g_phantom, clk.restart().as_seconds());
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
    g_phantom = Model::State();
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
    auto ee_pos_vec = Model::forward_kinematics(g_phantom.Q);
    ee_pos[0] = ee_pos_vec.x;
    ee_pos[1] = ee_pos_vec.x;
    ee_pos[2] = ee_pos_vec.x;
}

// EXPORT void get_ik(const double* ee_pos, double* theta_d, double* curr_angles) {
//     Point ee_point{ee_pos[0], ee_pos[1], ee_pos[2]};
//     std::vector<double> curr_angles_vec(curr_angles,curr_angles+sizeof(curr_angles)/sizeof(curr_angles[0]));
//     std::lock_guard<std::mutex> lock(g_mtx);
//     std::vector<double> theta_d_ = Model::inverse_kinematics(ee_point, curr_angles_vec);
//     std::copy(theta_d_.begin(),theta_d_.end(),theta_d);
// }

// EXPORT void get_input(double* input_) {
//     std::copy(input.begin(),input.end(),input_);
// }

class Tuner : public Application {
public:
    Tuner() : Application() { }
    void update() override {
        ImGui::Begin("Test",&keep_open);
        {
            std::lock_guard<std::mutex> lock(g_mtx);
            ImGui::Checkbox("Cavusoglu",&cavusoglu);
            ImGui::Checkbox("Control",&control);
            ImGui::Checkbox("Jacobian Test",&jacobian);
            // ImGui::DragDouble("K Hardstop", &g_phantom.Khard, 1, 0, 100);
            // ImGui::DragDouble("B Hardstop", &g_phantom.Bhard, 1, 0, 100);
            ImGui::DragDouble("Kp 1", &kps[0], 0.1, 0, 100);
            ImGui::DragDouble("Kp 2", &kps[1], 0.1, 0, 100);
            ImGui::DragDouble("Kp 3", &kps[2], 0.1, 0, 100);
            ImGui::DragDouble("Kd 1", &kds[0], 0.01, 0, 10);
            ImGui::DragDouble("Kd 2", &kds[1], 0.01, 0, 10);
            ImGui::DragDouble("Kd 3", &kds[2], 0.01, 0, 10);
            ImGui::DragDouble("X", &input[0], 0.001, -1, 1);
            ImGui::DragDouble("Y", &input[1], 0.001, -1, 1);
            ImGui::DragDouble("Z", &input[2], 0.001, -1, 1);
        }
        ImGui::End();
        if (!keep_open)
            quit();
    }
    bool keep_open = true;
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
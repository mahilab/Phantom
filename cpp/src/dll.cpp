#include "PhantomSimulation.hpp"
#include "PhantomController.hpp"
#include <Mahi/Util.hpp>
#include <Mahi/Gui.hpp>

using namespace Phantom;
using namespace mahi::gui;
using mahi::util::Timer;
using mahi::util::Clock;

#define EXPORT extern "C" __declspec(dllexport)

Controller g_sim = Controller(std::make_shared<Simulation>());

EXPORT void start() {
    g_sim.start();
}

EXPORT void stop() {
    g_sim.stop();
}

EXPORT void set_torques(double* Tau) {

}

EXPORT void get_positions(double* Q) {
    auto Q_sim = g_sim.get_positions();
    std::copy(Q_sim.begin(), Q_sim.end(), Q);
}

class Tuner : public Application {
public:
    Tuner() : Application() { }
    void update() override {
        ImGui::Begin("Test",&keep_open);
        {
            // auto lock = g_sim.get_lock();
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
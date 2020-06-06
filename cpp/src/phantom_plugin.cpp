#include "PhantomGui.hpp"

#define EXPORT extern "C" __declspec(dllexport)

using Phantom::Ptr;
using mahi::util::Severity;

std::mutex g_mtx;

Ptr<Phantom::Controller> g_controller = nullptr;
Ptr<Phantom::Gui>        g_gui        = nullptr;

void gui_thread() {
    {
        LOG(Severity::Info) << "Initalizing plugin objects";
        Phantom::Lock lock(g_mtx);
        g_controller = std::make_shared<Phantom::Controller>();
        g_gui = std::make_shared<Phantom::Gui>(g_controller);
        LOG(Severity::Info) << "Initialized plugin objects";
    }
    LOG(Severity::Info) << "Running Phantom GUI";
    g_gui->run();
    LOG(Severity::Info) << "Terminating Phantom GUI";
    {
        LOG(Severity::Info) << "Destroying plugin objects";
        Phantom::Lock lock(g_mtx);
        g_controller = nullptr;
        g_gui        = nullptr;
        LOG(Severity::Info) << "Destroyed plugin objects";
    }
}

EXPORT bool open_gui() {
    if (g_gui == nullptr) {
        auto thrd = std::thread(gui_thread);
        thrd.detach();
        return true;
    }
    return false;
}

EXPORT void get_positions(double* Q_out) {
    Phantom::Lock lock(g_mtx);
    if (g_controller != nullptr && g_controller->running()) {
        auto Q = g_controller->get_positions();
        std::copy(Q.begin(), Q.end(), Q_out);
    }
}
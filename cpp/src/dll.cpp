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
Vector3d g_target;
std::mutex g_unity_mtx;

EXPORT void start() {
    g_sim.start();
}

EXPORT void stop() {
    g_sim.stop();
}

EXPORT void get_positions(double* Q) {
    auto Q_sim = g_sim.get_positions();
    std::copy(Q_sim.begin(), Q_sim.end(), Q);
}

EXPORT void set_target(double x, double y, double z) {
    Lock lock(g_unity_mtx);
    g_target.x() = x;
    g_target.y() = y;
    g_target.z() = z;
}

class Debugger : public Application {
public:

    enum Mode { None = 0, JST, JSPD, TSF, TSPD, TSEPD};

    bool keep_open = true;
    Ptr<JointSpaceTorque> jst;
    Ptr<JointSpacePD> jspd;
    Ptr<TaskSpaceForce> tsf;
    Ptr<TaskSpacePD> tspd;
    Ptr<TaskSpaceEulerPD> tsepd;

    Debugger() : Application() { 
        jst  = std::make_shared<JointSpaceTorque>();
        jspd = std::make_shared<JointSpacePD>();
        tsf  = std::make_shared<TaskSpaceForce>();
        tspd = std::make_shared<TaskSpacePD>();
        tsepd = std::make_shared<TaskSpaceEulerPD>();
    }

    void update() override {
        ImGui::Begin("Test",&keep_open);
        {
            Lock unity_lock(g_unity_mtx);
            static int mode = 0;
            static bool track_target = false;
            if (ImGui::ModeSelector(&mode, {"None", "JS-T", "JS-PD","TS-F","TS-PD","TSE-PD"})) {
                if (mode == None)
                    g_sim.set_law(nullptr);
                if (mode == JST)
                    g_sim.set_law(jst);
                else if (mode == JSPD) 
                    g_sim.set_law(jspd);                
                else if (mode == TSF)
                    g_sim.set_law(tsf);
                else if (mode == TSPD)
                    g_sim.set_law(tspd);
                else if (mode == TSEPD)
                    g_sim.set_law(tsepd);
            }
            ImGui::Separator();
            auto Q = g_sim.get_positions();
            auto P = Model::forward_kinematics(Q);
            auto T = g_sim.get_torques();
            auto lock = g_sim.get_lock();
            if (mode == JST) {
                ImGui::DragDouble3("Torque [Nm]", jst->Tau.data(), 0.001f, -1, 1);               
            }
            if (mode == JSPD) {
                static bool ik = false;
                ImGui::Checkbox("IK", &ik);
                if (ik)
                    ImGui::Checkbox("Track Target", &track_target);
                if (ik) {
                    static Vector3d EE;
                    if (track_target)
                        EE = g_target;
                    else
                        ImGui::DragDouble3("Position [m]", EE.data(), 0.001f, -1, 1);
                    jspd->Q_ref = Model::inverse_kinematics(EE, jspd->Q_ref);
                }
                else
                    ImGui::DragDouble3("Theta [rad]", jspd->Q_ref.data(), 0.001f, -1, 1);                
            }
            else if (mode == TSF) {
                ImGui::Checkbox("G Comp", &tsf->gcomp);
                ImGui::DragDouble3("Forece [N]", tsf->F.data(), 0.001f, -1, 1);
            }
            else if (mode == TSPD) {
                ImGui::Checkbox("Track Target", &track_target);
                ImGui::Checkbox("G Comp", &tspd->gcomp);
                if (track_target)
                    tspd->P_ref = g_target;
                else
                    ImGui::DragDouble3("Position [m]", tspd->P_ref.data(), 0.001f, -1, 1);
                ImGui::DragDouble3("Kp [N/m]", tspd->Kp.data(), 0.001f, 0, 100);
                ImGui::DragDouble3("Kd [Ns/m", tspd->Kd.data(), 0.001f, 0, 10);
            }
            else if (mode == TSEPD) {
                ImGui::Checkbox("Track Target", &track_target);
                ImGui::Checkbox("G Comp", &tsepd->gcomp);
                if (track_target)
                    tsepd->P_ref = g_target;
                else
                    ImGui::DragDouble3("Position [m]", tsepd->P_ref.data(), 0.001f, -1, 1);
                ImGui::DragDouble3("Euler Angles [rad]", tsepd->EulerAngles.data(), .001f, -mahi::util::PI/2,mahi::util::PI/2);
                ImGui::DragDouble3("Kp [N/m]", tsepd->Kp.data(), 0.001f, 0, 100);
                ImGui::DragDouble3("Kd [Ns/m", tsepd->Kd.data(), 0.001f, 0, 10);
            }
            ImGui::Separator();
            ImGui::Text("Q: %.2f, %.2f, %.2f", Q[0], Q[1], Q[2]);
            ImGui::Text("P: %.2f, %.2f, %.2f", P[0], P[1], P[2]);
            ImGui::Text("T: %.2f, %.2f, %.2f", T[0], T[1], T[2]);

        }
        ImGui::End();
        if (!keep_open)
            quit();
    }
};

std::unique_ptr<Debugger> g_debugger = nullptr;

void tuner_thread() {
    g_debugger = std::make_unique<Debugger>();
    g_debugger->run();
    g_debugger = nullptr;
}

EXPORT bool open_tuner() 
{
    if (g_debugger == nullptr) {
        auto thrd = std::thread(tuner_thread);
        thrd.detach();
        return true;
    }
    return false;
}
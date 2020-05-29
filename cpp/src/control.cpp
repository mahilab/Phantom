#include "PhantomHardware.hpp"
#include "PhantomController.hpp"
#include <Mahi/Gui.hpp>

using namespace mahi::gui;
using namespace mahi::util;


struct ScrollingData {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingData() { 
        MaxSize = 1000;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};


class ControlGui : public Application {
public:

    enum Mode { None = 0, JST, JSPD, TSF, TSPD };


    ControlGui() : 
        Application(500,500,"Phantom Control"),
        m_phantom(std::make_shared<Phantom::Hardware>()),
        m_jst(std::make_shared<Phantom::JointSpaceTorque>()),
        m_jspd(std::make_shared<Phantom::JointSpacePD>()),
        m_controller(m_phantom, nullptr, 500_Hz)
     { 
         m_jspd->Kp = {1,1,1};
         m_jspd->Kd = {0.02, 0.02, 0.02};
     }

    ~ControlGui() {
        
    }

    void update() override {
        static ScrollingData dataP;
        static ScrollingData dataV;
        double t = time().as_seconds();

        // if (m_controller.running()) {
        //     auto Q = m_controller.get_positions();
        //     auto Qd = m_controller.get_velocities();
        //     dataP.AddPoint(t, Q[0]);
        //     dataV.AddPoint(t, Qd[0]);
        // }

        ImGui::Begin("Phantom Control");
        static int mode = 0;
        if (ImGui::ModeSelector(&mode, {"None", "JS-T", "JS-PD","TS-F","TS-PD"})) {
            if (mode == None)
                m_controller.set_law(nullptr);
            if (mode == JST)
                m_controller.set_law(m_jst);
            else if (mode == JSPD) 
                m_controller.set_law(m_jspd);                
            else if (mode == TSF)
                m_controller.set_law(nullptr);
            else if (mode == TSPD)
                m_controller.set_law(nullptr);
        }

        if (ImGui::Button("Start"))
            m_controller.start();
        if (ImGui::Button("Stop"))
            m_controller.stop();
        {
            auto lock = m_controller.get_lock();
            if (mode == JST) 
                ImGui::DragDouble3("Tau",m_jst->Tau.data(),0.001f,-0.4,0.4);
            else if (mode == JSPD) {
                static bool ik = false;
                ImGui::Checkbox("IK", &ik);
                if (ik) {
                    static Phantom::Vector3d EE = Phantom::Model::forward_kinematics(Phantom::Vector3d::Zero());
                    ImGui::DragDouble3("Position [m]", EE.data(), 0.001f, -1, 1);
                    m_jspd->Q_ref = Phantom::Model::inverse_kinematics(EE, m_jspd->Q_ref);
                }
                else {
                    ImGui::DragDouble3("Q",m_jspd->Q_ref.data(), 0.001, -1,1);
                }
                ImGui::DragDouble3("Kp [N/m]", m_jspd->Kp.data(), 0.001f, 0, 100);
                ImGui::DragDouble3("Kd [Ns/m", m_jspd->Kd.data(), 0.001f, 0, 10);
            }
        }
        // ImPlot::SetNextPlotLimitsX(t-10,t, ImGuiCond_Always);
        // if (ImPlot::BeginPlot("Plot", NULL, NULL, ImVec2(400,300), ImPlotFlags_Default | ImPlotFlags_YAxis2)) {
        //     ImPlot::SetPlotYAxis(0);
        //     ImPlot::PlotLine("Q[0]", &dataP.Data[0], dataP.Data.size(), dataP.Offset);
        //     ImPlot::SetPlotYAxis(1);
        //     ImPlot::PlotLine("Qd[0]", &dataV.Data[0], dataV.Data.size(), dataV.Offset);
        //     ImPlot::EndPlot();
        // }



        ImGui::End();
    }

private:
    std::shared_ptr<Phantom::Hardware> m_phantom;
    std::shared_ptr<Phantom::JointSpaceTorque> m_jst;
    std::shared_ptr<Phantom::JointSpacePD> m_jspd;
    Phantom::Controller m_controller;
};

int main(int argc, char const *argv[])
{    
    ControlGui gui;
    gui.run();    
    return 0;
}

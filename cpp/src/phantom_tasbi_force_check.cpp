#include "PhantomHardware.hpp"
#include "PhantomSimulation.hpp"
#include "PhantomController.hpp"
#include <Mahi/Robo/Mechatronics/AtiSensor.hpp>

using Phantom::Ptr;
using mahi::util::Severity;
using mahi::robo::AtiSensor;

namespace Phantom { 

using namespace mahi::gui;
using namespace mahi::util;

struct ScrollingData {
    int MaxSize;
    int Offset;
    ImVector<ImPlotPoint> Data;
    ScrollingData() {
        MaxSize = 2000;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(double x, double y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImPlotPoint(x,y));
        else {
            Data[Offset] = ImPlotPoint(x,y);
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

class TasbiGui : public Application {
public:

    TasbiGui() : 
        Application(),
        m_controller(std::make_shared<Phantom::Controller>()),
        m_phantom(std::make_shared<Phantom::Hardware<QPid>>()),
        m_law(std::make_shared<TasbiButton>())
     { 
        m_controller->set_phantom(m_phantom);
        m_controller->set_law(m_law);

        m_nano17.load_calibration("FT26062.cal");
        m_nano17.set_channels(&m_q8.AI[0], &m_q8.AI[1], &m_q8.AI[2], &m_q8.AI[3], &m_q8.AI[4], &m_q8.AI[5]);
     }

    /// Add a control law that can be used from the gui
 

private:
    void update() override {
        ImGui::Begin("Phantom-Tasbi GUI",&m_open);

        bool run = m_controller->running();
        if (ImGui::ButtonColored(run ? "Stop" : "Start", run ? mahi::gui::Reds::FireBrick : ImGui::GetStyleColorVec4(ImGuiCol_Button), ImVec2(100,0)))
            run ? m_controller->stop() : m_controller->start();

       if (m_phantom != nullptr) {
            ImGui::Separator();
            auto lock = m_controller->get_lock();
            m_phantom->imgui();
        }

        double phantom_force;

        ImGui::Separator(); 
        {
            auto lock = m_controller->get_lock();
            m_law->imgui();  
            phantom_force = m_law->F[2];
        }

        ImGui::Separator();
        if (ImGui::Button("Zero Nano17"))
            m_nano17.zero();

        m_q8.read_all();
        float nano17_force = (float)-m_nano17.get_force(mahi::robo::Axis::AxisZ);
        ImGui::Value("Force [N]",nano17_force);

        static mahi::util::Clock clk;
        double t = clk.get_elapsed_time().as_seconds();

        static ScrollingData nano17_data;
        static ScrollingData phantom_data;

        nano17_data.AddPoint(t, nano17_force);
        phantom_data.AddPoint(t, phantom_force);

        ImPlot::SetNextPlotLimitsX(t - 10, t, ImGuiCond_Always);
        if (ImPlot::BeginPlot("Force",NULL,"Force [N]", ImVec2(-1,-1))) {
            ImPlot::PlotLine("Phantom", &phantom_data.Data[0].x, &phantom_data.Data[0].y, phantom_data.Data.size(), phantom_data.Offset, 2 * sizeof(double));
            ImPlot::PlotLine("Nano17", &nano17_data.Data[0].x, &nano17_data.Data[0].y, nano17_data.Data.size(), nano17_data.Offset, 2 * sizeof(double));
            ImPlot::EndPlot();
        }

        ImGui::End();

        if (!m_open) 
            quit();
    }

public:
    Ptr<Controller>    m_controller;
    Ptr<Interface>     m_phantom;
    Ptr<TasbiButton>   m_law;

public: 
    bool m_open = true;
    bool m_pause_plot = false;
    int  m_samples = 0;
    AtiSensor m_nano17;
    mahi::daq::Q8Usb m_q8;
};

}

int main(int argc, char const *argv[])
{

    Phantom::Vector3d tau; 
    tau[0] = Phantom::Model::Tau_max[0];
    tau[1] = Phantom::Model::Tau_max[1];
    tau[2] = Phantom::Model::Tau_max[2];

    std::cout << Phantom::Model::torques_to_forces(tau, Phantom::Vector3d::Zero());

    Phantom::TasbiGui gui;
    gui.run();
    return 0;
}

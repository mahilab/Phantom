#include "PhantomHardware.hpp"
#include "PhantomSimulation.hpp"
#include "PhantomController.hpp"

namespace Phantom { 

using namespace mahi::gui;
using namespace mahi::util;

class Gui : public Application {
public:

    Gui(Ptr<Controller> controller, Ptr<Interface> phantom = nullptr) : 
        Application(),
        m_controller(controller),
        m_phantom(phantom)
     { 
        add_control_law("None", nullptr);
        add_control_law("Joint Space Torque", std::make_shared<JointSpaceTorque>());
        add_control_law("Joint Space PD", std::make_shared<JointSpacePD>());
        add_control_law("Joint Space PD (IK)", std::make_shared<JointSpaceIK>());
        add_control_law("Task Space Force", std::make_shared<TaskSpaceForce>());
        add_control_law("Task Space PD", std::make_shared<TaskSpacePD>());
        add_control_law("Task Space PD (Euler)", std::make_shared<TaskSpaceEulerPD>());
        add_control_law("Sin Tracker", std::make_shared<SinTracker>());
        add_control_law("Circle Tracker", std::make_shared<CircleTracker>());
        add_control_law("Python Script", std::make_shared<PythonScript>());
        add_control_law("Tasbi Button", std::make_shared<TasbiButton>());
     }

    /// Add a control law that can be used from the gui
    void add_control_law(const std::string& name, Ptr<ControlLaw> law) {
        m_laws.push_back({name, law});
    }

private:
    void update() override {
        ImGui::Begin("Phantom GUI",&m_open);

        // phantom interface selection
        ImGui::BeginDisabled(m_phantom != nullptr && m_controller->running());

        static const char* phantom_names[] = {"None", "Simulation", "Hardware (QPIDe)", "Hardware (Q8-USB)"};
        auto swap_phantom = [&](int i) {            
            m_current_phantom = i;
            if (i == 0)           
                m_controller->set_phantom(m_phantom = nullptr);
            else if (i == 1)
                m_controller->set_phantom(m_phantom = std::make_shared<Phantom::Simulation>());
            else if (i == 2)
                m_controller->set_phantom(m_phantom = std::make_shared<Phantom::Hardware<QPid>>());
            else if (i == 3)
                m_controller->set_phantom(m_phantom = std::make_shared<Phantom::Hardware<Q8Usb>>());
        };
        
        ImGui::SetNextItemWidth(400);
        if (ImGui::BeginCombo("Phantom", phantom_names[m_current_phantom])) {
            for (int i = 0; i < 4; ++i)
            if (ImGui::Selectable(phantom_names[i], m_current_phantom == i))
                swap_phantom(i);
            ImGui::EndCombo();
        }
        ImGui::EndDisabled();

        ImGui::BeginDisabled(m_phantom == nullptr);

        bool run = m_controller->running();
        if (ImGui::ButtonColored(run ? "Stop" : "Start", run ? mahi::gui::Reds::FireBrick : ImGui::GetStyleColorVec4(ImGuiCol_Button), ImVec2(100,0)))
            run ? m_controller->stop() : m_controller->start();
        ImGui::SameLine();
        ImGui::SetNextItemWidth(295);
        if (ImGui::BeginCombo("Control Law", m_laws[m_current_law].name.c_str())) {
            for (int i = 0; i < m_laws.size(); ++i) {
                if (ImGui::Selectable(m_laws[i].name.c_str(), m_current_law == i)) {
                    m_controller->set_law(m_laws[i].law);
                    m_current_law = i;
                }
            }
            ImGui::EndCombo();
        }
        if (m_phantom != nullptr) {
            ImGui::Separator();
            auto lock = m_controller->get_lock();
            m_phantom->imgui();
        }


        if (m_laws[m_current_law].law != nullptr) {
            ImGui::Separator();
            auto lock = m_controller->get_lock();
            m_laws[m_current_law].law->imgui();
        }

        ImGui::Separator();
        static constexpr int k_samples = 10000;
        static double data[Data_COUNT][k_samples];

        ImGui::Checkbox("Pause Plot",&m_pause_plot);
        
        if (!m_pause_plot) {
            m_samples = m_controller->get_data(Data_Time, data[Data_Time], k_samples);
            for (int i = 1; i < (int)Data_COUNT; ++i) {
                m_controller->get_data((Phantom::DataIndex)i, data[i], k_samples);
                Timer timer(100_us); // dont want to block the control thread (need better design here) THIS CAUSES PHASE DELAY IN PLOT
                timer.wait();
            }
        }       

        static bool show_velocities = true;
        static bool show_ee   = false;

        ImGui::SameLine();
        ImGui::Checkbox("Plot Velocities", &show_velocities);
        ImGui::SameLine();
        ImGui::Checkbox("Plot End Effector", &show_ee);

        double tmin = data[Data_Time][0];
        double tmax = std::max(k_samples/2000.0, data[Data_Time][m_samples-1]);
        ImPlot::SetNextPlotLimitsX(tmin, tmax, m_pause_plot ? ImGuiCond_Once : ImGuiCond_Always);
        ImPlot::SetNextPlotLimitsY(-2,0.5,ImGuiCond_Once,0);
        ImPlot::SetNextPlotLimitsY(-1.5,5,ImGuiCond_Once,1);
        if (ImPlot::BeginPlot("##Plot", NULL, NULL, ImVec2(-1,-1), ImPlotFlags_YAxis2 | ImPlotFlags_YAxis3)) {            
            
            // ImPlot::SetColormap(ImPlotColormap_Paired);
            
            ImPlot::SetPlotYAxis(0);
            ImPlot::PlotLine("Q0", data[Data_Time], data[Data_Q0], m_samples); 
            if (show_velocities) { 
                ImPlot::SetPlotYAxis(1);
                ImPlot::PlotLine("Qd0", data[Data_Time], data[Data_Qd0], m_samples);
            }

            ImPlot::SetPlotYAxis(0);
            ImPlot::PlotLine("Q1", data[Data_Time], data[Data_Q1], m_samples); 
            if (show_velocities) {
                ImPlot::SetPlotYAxis(1);
                ImPlot::PlotLine("Qd1", data[Data_Time], data[Data_Qd1], m_samples);
            }
            
            ImPlot::SetPlotYAxis(0);
            ImPlot::PlotLine("Q2", data[Data_Time], data[Data_Q2], m_samples); 
            if (show_velocities) {
                ImPlot::SetPlotYAxis(1);
                ImPlot::PlotLine("Qd2", data[Data_Time], data[Data_Qd2], m_samples);
            }
            if (show_ee) {
                // ImPlot::SetColormap(ImPlotColormap_Cool);
                ImPlot::SetPlotYAxis(2);
                ImPlot::PlotLine("X", data[Data_Time], data[Data_X], m_samples);
                ImPlot::PlotLine("Y", data[Data_Time], data[Data_Y], m_samples);
                ImPlot::PlotLine("Z", data[Data_Time], data[Data_Z], m_samples);
            }
            ImPlot::EndPlot();
        }

        ImGui::EndDisabled();
        ImGui::End();

        if (!m_open) 
            quit();
    }

    struct ControlLawHolder {
        std::string name;
        Ptr<ControlLaw> law;
    };

public: // fuck it
    bool m_open = true;
    Ptr<Controller>    m_controller;
    Ptr<Interface>     m_phantom;
    std::vector<ControlLawHolder>  m_laws;
    int  m_current_phantom = 0;
    int  m_current_law = 0;
    bool m_pause_plot = false;
    int  m_samples = 0;
};


} // namespace Phantom
#include "PhantomHardware.hpp"
#include "PhantomSimulation.hpp"
#include "PhantomController.hpp"

#define EXPORT extern "C" __declspec(dllexport)

using Phantom::Ptr;
using mahi::util::Severity;

namespace Phantom { 

using namespace mahi::gui;
using namespace mahi::util;

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

        ImGui::Separator();
        auto lock = m_controller->get_lock();
        m_law->imgui();        

        /*

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
        if (ImPlot::BeginPlot("##Plot", NULL, NULL, ImVec2(-1,-1), ImPlotFlags_Default | ImPlotFlags_YAxis2 | ImPlotFlags_YAxis3)) {            
            
            ImPlot::SetColormap(ImPlotColormap_Paired);
            
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
                ImPlot::SetColormap(ImPlotColormap_Cool);
                ImPlot::SetPlotYAxis(2);
                ImPlot::PlotLine("X", data[Data_Time], data[Data_X], m_samples);
                ImPlot::PlotLine("Y", data[Data_Time], data[Data_Y], m_samples);
                ImPlot::PlotLine("Z", data[Data_Time], data[Data_Z], m_samples);
            }
            ImPlot::EndPlot();
        }
    */

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
};

}

std::mutex g_mtx;

Ptr<Phantom::TasbiGui>   g_gui        = nullptr;

void gui_thread() {
    {
        LOG(Severity::Info) << "Initalizing plugin objects";
        Phantom::Lock lock(g_mtx);
        g_gui = std::make_shared<Phantom::TasbiGui>();
        LOG(Severity::Info) << "Initialized plugin objects";
    }
    LOG(Severity::Info) << "Running Phantom GUI";
    g_gui->run();
    LOG(Severity::Info) << "Terminating Phantom GUI";
    {
        LOG(Severity::Info) << "Destroying plugin objects";
        Phantom::Lock lock(g_mtx);
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
    if (g_gui != nullptr && g_gui->m_controller->running()) {
        auto Q = g_gui->m_controller->get_positions();
        std::copy(Q.begin(), Q.end(), Q_out);
    }
}

EXPORT bool start() {
    Phantom::Lock lock(g_mtx);
    if (g_gui != nullptr && !g_gui->m_controller->running()) {
        return g_gui->m_controller->start();
    }
    return false;
}

EXPORT bool stop() {
    Phantom::Lock lock(g_mtx);
    if (g_gui != nullptr && g_gui->m_controller->running()) {
        return g_gui->m_controller->stop();
    }
    return false;
}

EXPORT bool zero() {
    Phantom::Lock lock(g_mtx);    
    if (g_gui != nullptr) {
        return g_gui->m_controller->zero();
    }
    return false;    
}

EXPORT bool set_stiff_max(double Kb_max) {
    if (g_gui != nullptr) {
        auto lock = g_gui->m_controller->get_lock();
        g_gui->m_law->Kb_max = Kb_max;
        return true;
    }
    return false;
}

EXPORT bool set_stiff_min(double Kb_min) {
    if (g_gui != nullptr) {
        auto lock = g_gui->m_controller->get_lock();
        g_gui->m_law->Kb_min = Kb_min;
        return true;
    }
    return false;
}

EXPORT bool set_stiffness(double Kb) {
    if (g_gui != nullptr) {
        auto lock = g_gui->m_controller->get_lock();
        g_gui->m_law->Kb = Kb;
        return true;
    }
    return false;
}

EXPORT bool modify_stiffness(double delta) {
    if (g_gui != nullptr) {
        auto lock = g_gui->m_controller->get_lock();
        g_gui->m_law->Kb += delta;
        return true;
    }
    return false;
}

EXPORT double get_stiffness() {
    if (g_gui != nullptr) {
        auto lock = g_gui->m_controller->get_lock();
        return g_gui->m_law->Kb;
    }
    return -1; 
}

EXPORT double get_force() {
    if (g_gui != nullptr) {
        auto lock = g_gui->m_controller->get_lock();
        return g_gui->m_law->F[2];
    }
    return -1; 
}

EXPORT double get_height() {
    if (g_gui != nullptr) {
        auto lock = g_gui->m_controller->get_lock();
        return g_gui->m_law->h;
    }
    return -1; 
}

EXPORT double get_velocity() {
    if (g_gui != nullptr) {
        auto lock = g_gui->m_controller->get_lock();
        return g_gui->m_law->dhdt;
    }
    return -1; 
}
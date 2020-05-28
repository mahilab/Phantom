#include "PhantomHardware.hpp"
#include "PhantomController.hpp"
#include <Mahi/Gui.hpp>

using namespace mahi::gui;
using namespace mahi::util;

class ControlGui : public Application {
public:
    ControlGui() : 
        Application(500,500,"Phantom Control"),
        m_phantom(std::make_shared<Phantom::Hardware>()),
        m_jst(std::make_shared<Phantom::JointSpaceTorque>()),
        m_controller(m_phantom, m_jst)
     { }

    ~ControlGui() {
        
    }

    void update() override {
        ImGui::Begin("Phantom Control");
        if (ImGui::Button("Start"))
            m_controller.start();
        if (ImGui::Button("Stop"))
            m_controller.stop();
        {
            auto lock = m_controller.get_lock();
            ImGui::DragDouble3("Tau",m_jst->Tau.data(),0.001f,-0.2,0.2);
        }
        ImGui::End();
    }

private:
    std::shared_ptr<Phantom::Hardware> m_phantom;
    std::shared_ptr<Phantom::JointSpaceTorque> m_jst;
    Phantom::Controller m_controller;
};

int main(int argc, char const *argv[])
{    
    ControlGui gui;
    gui.run();    
    return 0;
}

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
        m_controller(m_phantom)
     { }

    void update() override {
        ImGui::Begin("Phantom Control");
        if (ImGui::Button("Start"))
            m_controller.start();
        if (ImGui::Button("Stop"))
            m_controller.stop();
        ImGui::End();
    }

private:
    std::shared_ptr<Phantom::Hardware> m_phantom;
    Phantom::Controller m_controller;
};

int main(int argc, char const *argv[])
{    
    // ControlGui gui;
    // gui.run();    
    Q8Usb q8;
    q8.enable();
    q8.velocity.read();
    print("{}",q8.velocity[0]);
    return 0;
}

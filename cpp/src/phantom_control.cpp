#include "PhantomGui.hpp"

int main(int argc, char const *argv[])
{  
    auto controller = std::make_shared<Phantom::Controller>();
    Phantom::Gui gui(controller);
    gui.run();  
    return 0;
}

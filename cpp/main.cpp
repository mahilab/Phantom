#include <iostream>
#include "Phantom.hpp"

int main(int argc, char const *argv[])
{
    Phantom phantom;
    phantom.reset();

    phantom.update(0.01);
    for (int i = 0; i < 10000; ++i) {
        Vector3d tau;
        phantom.update(0.001);
        std::cout << phantom.Q.transpose() << std::endl;
    }
    return 0;
}

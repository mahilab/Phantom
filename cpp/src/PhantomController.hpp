#pragma once
#include "PhantomInterface.hpp"

namespace Phantom {

struct Controller {
    virtual ~Controller() { }
    virtual void control(Interface& phantom) = 0;
};

struct JointSpacePD : public Controller {
    virtual void control(Interface& phantom) override {
        auto Q = phantom.get_positions();
        auto Qd = phantom.get_velocities();
        Vector3d Tau = Kp.cwiseProduct(Q_ref - Q) - Kd.cwiseProduct(Qd);
        phantom.set_torques(Tau);        
    }
    Vector3d Kp;
    Vector3d Kd;
    Vector3d Q_ref;    
};

struct TaskSpacePD : public Controller {

};

} // namespace Phantom

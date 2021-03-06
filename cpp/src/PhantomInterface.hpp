#pragma once
#include "PhantomModel.hpp"

namespace Phantom {

//=============================================================================
// INTERFACE
//=============================================================================

/// Base class for Phantom interfaces (simulations and hardware)
class Interface {
public:
    /// Virtual destructor
    virtual ~Interface() { }
    /// Starts running the Phantom
    virtual void start() = 0;
    /// Stops running the Phantom
    virtual void stop() = 0;
    /// Sets the Phantom joint torques [Nm]
    virtual void set_torques(const Vector3d& Tau) = 0;
    /// Gets the Phantom joint angles [rad]
    virtual Vector3d get_positions() = 0;
    /// Gets the Phantom joint velocities [rad/s]
    virtual Vector3d get_velocities() = 0;    
    /// Gets the Phantom joint Torques [Nm]
    virtual Vector3d get_torques() = 0;  
    /// Zeros the Phantom joint angles
    virtual bool zero() = 0;
    /// Implement to draw GUI 
    virtual void imgui() { };
};

} // namespace Phantom
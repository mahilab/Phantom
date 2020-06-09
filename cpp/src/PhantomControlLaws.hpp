#pragma once
#include "PhantomInterface.hpp"
#include <Mahi/Gui.hpp>
#include "Python.h"
#include <algorithm>

namespace Phantom {

struct ControlLaw {
    virtual ~ControlLaw() { }
    /// Return joint torques given joint positions and velocities and delta time
    virtual Vector3d control(Vector3d Q, Vector3d , double dt) = 0;
    virtual void imgui() = 0;
};

struct JointSpaceTorque : public ControlLaw {
    JointSpaceTorque() { Tau.fill(0); }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) override {
        return Tau;
    }
    virtual void imgui() override {
        ImGui::DragDouble3("Torque [Nm]",Tau.data(),0.001f,-0.4,0.4);
    }
    Vector3d Tau;
};

struct JointSpacePD : public ControlLaw {
    JointSpacePD() { Kp.fill(5); Kd.fill(0.1f); Q_ref.fill(0); }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) override {
        return Kp.cwiseProduct(Q_ref - Q) - Kd.cwiseProduct(Qd);               
    }
    virtual void imgui() override {
        ImGui::DragDouble3("Joint Angles [rad]",Q_ref.data(), 0.001f, -1,1);
        ImGui::DragDouble3("Kp [N/m]",  Kp.data(), 0.001f, 0, 100);
        ImGui::DragDouble3("Kd [Ns/m]", Kd.data(), 0.001f, 0, 10);
    }
    Vector3d Kp, Kd, Q_ref;
};

struct JointSpaceIK : public ControlLaw {
    JointSpaceIK() { 
        Kp.fill(5); 
        Kd.fill(0.1f); 
        P_ref = Phantom::Model::forward_kinematics(Vector3d::Zero());
    }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) override {
        Vector3d Q_ref = Phantom::Model::inverse_kinematics(P_ref, Q);
        return Kp.cwiseProduct(Q_ref - Q) - Kd.cwiseProduct(Qd);               
    }
    virtual void imgui() override {
        ImGui::DragDouble3("Position [m]", P_ref.data(), 0.001f, -1, 1);
        ImGui::DragDouble3("Kp [N/m]",  Kp.data(), 0.001f, 0, 100);
        ImGui::DragDouble3("Kd [Ns/m]", Kd.data(), 0.001f, 0, 10);
    }
    Vector3d Kp, Kd, P_ref;
};

struct TaskSpaceForce : public ControlLaw {
    TaskSpaceForce() { gcomp = true; F.fill(0); }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) {
        Vector3d G = gcomp ? Model::G(Q) : Vector3d::Zero();
        return G + Model::forces_to_torques(F,Q);
    }
    virtual void imgui() override {
        ImGui::DragDouble3("Force [N]",F.data(),0.001f,-0.5,0.5);
    }
    bool gcomp;
    Vector3d F;
};

struct TaskSpacePD : public ControlLaw {
    TaskSpacePD() { 
        gcomp = false; 
        Kp.fill(50); 
        Kd.fill(1); 
        P_ref = Phantom::Model::forward_kinematics(Phantom::Vector3d::Zero()); 
    }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) {
        Vector3d G = gcomp ? Model::G(Q) : Vector3d::Zero();
        Matrix3d Kp_diag = Kp.asDiagonal();
        Matrix3d Kd_diag = Kd.asDiagonal();
        Matrix3d Kpq = Model::J(Q).transpose() * Kp_diag * Model::J(Q);
        Matrix3d Kdq = Model::J(Q).transpose() * Kd_diag * Model::J(Q);
        Vector3d Q_ref = Model::inverse_kinematics(P_ref, Q);
        Vector3d tau = G + Kpq * (Q_ref - Q) - Kdq * (Qd);   
        return tau;    
    }
    virtual void imgui() override {
        ImGui::Checkbox("Gravity Compensation", &gcomp);
        ImGui::DragDouble3("Position [m]", P_ref.data(), 0.001f, -1, 1);
        ImGui::DragDouble3("Kp [N/m]", Kp.data(), 0.001f, 0, 100);
        ImGui::DragDouble3("Kd [Ns/m]", Kd.data(), 0.001f, 0, 10);
    }
    bool gcomp;
    Vector3d Kp, Kd, P_ref;
};

struct TaskSpaceEulerPD : public ControlLaw {
    TaskSpaceEulerPD() { gcomp = true; Kp.fill(100); Kd.fill(5); P_ref.fill(0); EulerAngles.fill(0);}
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) {
        Vector3d G = gcomp ? Model::G(Q) : Vector3d::Zero();
        Matrix3d Rx, Ry, Rz;
        Rx << 1, 0, 0, 0, mahi::util::cos(EulerAngles(0)), -mahi::util::sin(EulerAngles(0)), 0, mahi::util::sin(EulerAngles(0)), mahi::util::cos(EulerAngles(0));
        Ry << mahi::util::cos(EulerAngles(1)), 0, mahi::util::sin(EulerAngles(1)), 0, 1, 0, -mahi::util::sin(EulerAngles(1)), 0, mahi::util::cos(EulerAngles(1));
        Rz << mahi::util::cos(EulerAngles(2)), -mahi::util::sin(EulerAngles(2)), 0, mahi::util::sin(EulerAngles(2)), mahi::util::cos(EulerAngles(2)), 0, 0, 0, 1;
        auto R = Rx*Ry*Rz;
        auto J_rot = R*Model::J(Q);
        Matrix3d Kp_diag = Kp.asDiagonal();
        Matrix3d Kd_diag = Kd.asDiagonal();
        Matrix3d Kpq = J_rot.transpose() * Kp_diag * J_rot;
        Matrix3d Kdq = J_rot.transpose() * Kd_diag * J_rot;
        Vector3d Q_ref = Model::inverse_kinematics(P_ref, Q);
        return G + Kpq * (Q_ref - Q) - Kdq * (Qd);       
    }
    virtual void imgui() override {
        // TODO: Nathan, implement me!
    }
    bool gcomp;
    Vector3d EulerAngles;
    Vector3d Kp;
    Vector3d Kd;
    Vector3d P_ref;
};

struct SinTracker : public JointSpacePD {
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) override {
        for (int i = 0; i < 3; ++i)
            Q_ref[i] = amp * std::sin(2*PI*freq*t);
        t += dt;
        return JointSpacePD::control(Q, Qd, dt);
    }
    virtual void imgui() override {
        ImGui::DragDouble("Amplitude [rad]", &amp, 0.001f, 0, 0.4);
        ImGui::DragDouble("Frequency [Hz]", &freq, 0.001f, 0, 1);
        ImGui::DragDouble3("Kp [N/m]",  Kp.data(), 0.001f, 0, 100);
        ImGui::DragDouble3("Kd [Ns/m]", Kd.data(), 0.001f, 0, 10);
    }
    double amp = 0.1;
    double freq = 0.25;
    double t = 0;
};

struct CircleTracker : public JointSpaceIK {
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) override {
        P_ref[0] = 0.210 + radius * std::cos(2*PI*freq*t);
        P_ref[1] = radius * std::sin(2*PI*freq*t);
        P_ref[2] = z;
        t += dt;
        return JointSpaceIK::control(Q, Qd, dt);
    }

    virtual void imgui() override {
        ImGui::DragDouble("Radius [m]", &radius, 0.001f, 0, 0.06);
        ImGui::DragDouble("Frequency [Hz] ", &freq, 0.001f, 0, 1);
        ImGui::DragDouble("Z [m]",&z, 0.001f, -0.2, 0);
        ImGui::DragDouble3("Kp [N/m]",  Kp.data(), 0.001f, 0, 100);
        ImGui::DragDouble3("Kd [Ns/m]", Kd.data(), 0.001f, 0, 10);
    }

    double t      = 0;
    double radius = 0.0;
    double z      = -0.169;
    double freq   = 0.25;
};

struct PythonScript : public ControlLaw {
    PythonScript(){
        Py_Initialize();
    }
    ~PythonScript(){
        // Clean up
        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        Py_DECREF(pDict);
        Py_Finalize();
    }
    virtual Vector3d control(Vector3d Q, Vector3d Qd, double dt) override {
        Clock control_clock;
        if (py_enabled){
            PyObject *pResult, *pArgQ, *pArgQd, *pArgQref, *pArgs, *pValue, *pTau;
            pArgQ    = PyTuple_New(3);
            pArgQd   = PyTuple_New(3);
            pArgQref = PyTuple_New(3);
            pArgs    = PyTuple_New(3);
            for (size_t i = 0; i < 3; i++){
                // convert all values to PyFloats and write them to appropriate tuples
                pValue = PyFloat_FromDouble(Q[i]);
                PyTuple_SetItem(pArgQ, i, pValue);
                pValue = PyFloat_FromDouble(Qd[i]);
                PyTuple_SetItem(pArgQd, i, pValue);
                pValue = PyFloat_FromDouble(Q_ref[i]);
                PyTuple_SetItem(pArgQref, i, pValue);
            }
        
            // construct tuple of tuples as the arguments
            PyTuple_SetItem(pArgs, 0, pArgQ);
            PyTuple_SetItem(pArgs, 1, pArgQd);
            PyTuple_SetItem(pArgs, 2, pArgQref);

            pResult = PyObject_CallObject(pFunc, pArgs);
            
            Vector3d Tau = Vector3d::Zero();
            for (size_t i = 0; i < 3; i++){
                // iterate through output tuple and get the result
                pTau = PyTuple_GetItem(pResult, i);
                Tau[i] = PyFloat_AsDouble(pTau);
            }
            
            if(PyErr_Occurred()){
                py_err = true;
            }

            Py_DECREF(pResult);
            Py_DECREF(pArgQ);
            Py_DECREF(pArgQd);
            Py_DECREF(pArgQref);
            Py_DECREF(pArgs);
            Py_DECREF(pValue);
            Py_DECREF(pTau);
            control_time = control_clock.get_elapsed_time().as_microseconds();
            return py_err ? Vector3d::Zero() : Tau;
        } 
        else{
            control_time = control_clock.get_elapsed_time().as_microseconds();
            return Vector3d::Zero();
        } 
    }
    virtual void imgui() override {
        if (ImGui::Button("Open File")) {
            auto func = [this]() {
                std::string tmp;
                if (mahi::gui::open_dialog(tmp, {{"Python", "py"}}) == mahi::gui::DialogResult::DialogOkay) {
                    std::replace( tmp.begin(), tmp.end(), '\\', '/');
                    Lock lock(m_mtx);
                    size_t pathindex = tmp.find_last_of("/");
                    size_t lastindex = tmp.find_last_of(".");
                    std::string path = tmp.substr(0,pathindex);
                    m_file = tmp.substr(pathindex+1,lastindex-pathindex-1);

                    PySys_SetPath(&path[0]);
                    pModule = PyImport_ImportModule(m_file.c_str());
                    pDict = PyModule_GetDict(pModule);
                    pFunc = PyDict_GetItemString(pDict, "calc_torques");
                    py_enabled = PyCallable_Check(pFunc) ? true : false;
                }               
            };
            std::thread thrd(func); thrd.detach();
        }
        {
            Lock lock(m_mtx);
            ImGui::SameLine();
            ImGui::Text(("Python File: " + m_file).c_str());
        }
        if(py_enabled){
            ImGui::DragDouble3("Joint Angles [rad]",Q_ref.data(), 0.001f, -1,1);
            ImGui::InputInt("Control Time", &control_time);
            if(py_err) ImGui::TextColored(mahi::gui::Reds::Red, "Error in Python Code");
        }
    }
private:
    std::mutex m_mtx;
    std::string m_file;
    Vector3d Q_ref;
    int control_time;
    bool py_enabled = false;
    bool py_err = false;
    PyObject *pFunc, *pModule, *pDict;
};

} // namespace Phantom

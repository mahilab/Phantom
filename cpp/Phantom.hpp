#include <Eigen/Dense>
#include <Mahi/Util.hpp>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using mahi::util::DEG2RAD;

template <typename T>
class Integrator {
public:
    Integrator(T initial) : val(initial) { }
    inline T integrate(T dval_dt, double dt) {
        val += dt * 0.5 * (m_prev_dval_dt + dval_dt);
        m_prev_dval_dt = dval_dt;
        return val;
    }
    T val;
private:
    T m_prev_dval_dt;
};

class Phantom {
public:

    Phantom() : Qdd_Qd(Vector3d::Zero()), Qd_Q(Vector3d::Zero()) { 
        reset();
    }

    inline double hardstop(double q, double qd, double qmin, double qmax, double K, double B) {
        if (q < qmin)
            return K * (qmin - q) - B * qd;
        else if (q > qmax)
            return K * (qmax - q) - B * qd;
        else
            return 0;
    }

    inline void update(double dt) {

        M(0,0) = (0.125*(4*Iayy + 4*Iazz + 8*Ibaseyy + 4*Ibeyy + 4*Ibezz + 4*Icyy 
                 + 4*Iczz + 4*Idfyy + 4*Idfzz + 4*l1*l1*ma + l2*l2*ma + l1*l1*mc + 4*l3*l3*mc) 
                 + 0.125*(4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz 
                 + l1 *l1 *(4*ma + mc))*std::cos(2*Q[1]) 
                 + 0.125*(4*Iayy - 4*Iazz + 4*Idfyy - 4*Idfzz 
                 - l2*l2*ma - 4*l3*l3*mc)*std::cos(2*Q[2]) + l1*(l2*ma + l3*mc)*std::cos(Q[1])*std::sin(Q[2]));
        M(0,1) = 0;
        M(0,2) = 0;
        M(1,0) = 0;  
        M(1,1) = 0.25*(4*(Ibexx + Icxx + l1*l1*ma) + l1*l1*mc);        
        M(1,2) = -0.5*l1*(l2*ma + l3*mc)*std::sin(Q[1]-Q[2]);   
        M(2,0) = 0;     
        M(2,1) = -0.5*l1*(l2*ma + l3*mc)*std::sin(Q[1]-Q[2]);        
        M(2,2) = 0.25*(4*Iaxx + 4*Idfxx + l2*l2*ma + 4*l3*l3*mc);

        V(0,0) = 0.125*(-2*std::sin(Q[1])*((4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz
                 + 4*l1*l1*ma + l1*l1*mc)*std::cos(Q[1]) + 2*l1*(l2*ma + l3*mc)*std::sin(Q[2]))*Qd[1]
                 + 2*std::cos(Q[2])*(2*l1*(l2*ma+l3*mc)*std::cos(Q[1])
                 + (-4*Iayy + 4*Iazz - 4*Idfyy + 4*Idfzz + l2*l2*ma
                 + 4*l3*l3*mc)*std::sin(Q[2]))*Qd[2]);        
        V(0,1) = -0.125*((4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz + l1*l1*(4*ma
                 + mc))*std::sin(2*Q[1]) + 4*l1*(l2*ma + l3*mc)*std::sin(Q[1])*std::sin(Q[2]))*Qd[0];        
        V(0,2) = -0.125*(-4*l1*(l2*ma + l3*mc)*std::cos(Q[1])*std::cos(Q[2]) -(-4*Iayy
                 + 4*Iazz - 4*Idfyy + 4*Idfzz + l2*l2*ma + 4*l3*l3*mc)*std::sin(2*Q[2]))*Qd[0];                
        V(1,0) = -V(0,1);
        V(1,1) = 0;
        V(1,2) = 0.5*l1*(l2*ma+l3*mc)*std::cos(Q[1]-Q[2])*Qd[2];
        V(2,0) = -V(0,2);
        V(2,1) = 0.5*l1*(l2*ma+l3*mc)*std::cos(Q[1]-Q[2])*Qd[1];
        V(2,2) = 0;

        G(0)   = 0;
        G(1)   = 0.5*g*(2*l1*ma + 2*l5*mbe + l1*mc)*std::cos(Q[1]);
        G(2)   = 0.5*g*(l2*ma + 2*l3*mc - 2*l6*mdf)*std::sin(Q[2]);

        // hardstops and self collisions
        Vector3d Q_min;
        Vector3d Q_max;
        Q_min[0] = -90 * DEG2RAD;
        Q_max[0] =  90 * DEG2RAD;
        Q_min[1] = std::max(-124 * DEG2RAD, Q[2] - 55 * DEG2RAD);
        Q_max[1] = std::min( 119 * DEG2RAD, Q[2] + 65 * DEG2RAD);
        Q_min[2] = std::max( -29 * DEG2RAD, Q[1] - 65 * DEG2RAD);  
        Q_max[2] = std::min( 214 * DEG2RAD, Q[1] + 55 * DEG2RAD);

        Vector3d Tau_prime;
        for (int i = 0; i < 3; ++i)
            Tau_prime[i] = Tau[i] + hardstop(Q[i], Qd[i], Q_min[i], Q_max[i], Khard, Bhard);

        Qdd = M.householderQr().solve(Tau_prime - V*Qd - G);
        Qd  = Qdd_Qd.integrate(Qdd, dt);
        Q   = Qd_Q.integrate(Qd, dt);
    }

    void reset() {
        Qdd_Qd.val = Qd_Q.val = Qdd = Qd = Q = Tau = Vector3d::Zero();
    }

    std::vector<double> fk(Vector3d q){
        // std::vector<double> ee_pos = {cos(q(0))*(l2*sin(q(1) + q(2)) + l1*cos(q(1))),
        //                               sin(q(0))*(l2*sin(q(1) + q(2)) + l1*cos(q(1))),
        //                                         l1*sin(q(1)) - l2*cos(q(1) + q(2))};

        std::vector<double> ee_pos = {cos(q(0))*(l1*cos(q(1))+l2*sin(q(2)-mahi::util::PI/2)),
                                      sin(q(0))*(l1*cos(q(1))+l2*sin(q(2)-mahi::util::PI/2)),
                                                  l1*sin(q(1))+l2*cos(q(2)-mahi::util::PI/2)};
        return ee_pos; 
    }

public:

    Matrix3d M, V;
    Vector3d Q, Qd, Qdd, G, Tau;
    Integrator<Vector3d> Qdd_Qd, Qd_Q;

    static constexpr double g = 9.80665;

    static constexpr double ma = 0.0202;
    static constexpr double Iaxx = 0.4864e-4;
    static constexpr double Iayy = 0.001843e-4;
    static constexpr double Iazz = 0.4864e-4;

    static constexpr double mc = 0.0249;
    static constexpr double Icxx = 0.959e-4;
    static constexpr double Icyy = 0.959e-4;
    static constexpr double Iczz = 0.0051e-4;

    static constexpr double mbe = 0.2359;
    static constexpr double Ibexx = 11.09e-4;
    static constexpr double Ibeyy = 10.06e-4;
    static constexpr double Ibezz = 0.591e-4;

    static constexpr double mdf = 0.1906;
    static constexpr double Idfxx = 7.11e-4;
    static constexpr double Idfyy = 0.629e-4;
    static constexpr double Idfzz = 6.246e-4;  

    static constexpr double Ibaseyy = 11.87e-4;

    static constexpr double l1 = 0.215;
    static constexpr double l2 = 0.170;
    static constexpr double l3 = 0.0325;
    static constexpr double l5 = -0.0368;
    static constexpr double l6 = 0.0527;

    static constexpr double Khard = 50; 
    static constexpr double Bhard = 0.5;
};
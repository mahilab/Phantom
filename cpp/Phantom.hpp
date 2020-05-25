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

struct Point {
    double x,y,z;
};

struct Tensor {
    double xx, xy, xz, yy, yz, zz;
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

    inline void update1(double dt) {

        constexpr double ma = 0.0202;
        constexpr double Iaxx = 0.4864e-4;
        constexpr double Iayy = 0.001843e-4;
        constexpr double Iazz = 0.4864e-4;

        constexpr double mc = 0.0249;
        constexpr double Icxx = 0.959e-4;
        constexpr double Icyy = 0.959e-4;
        constexpr double Iczz = 0.0051e-4;

        constexpr double mbe = 0.2359;
        constexpr double Ibexx = 11.09e-4;
        constexpr double Ibeyy = 10.06e-4;
        constexpr double Ibezz = 0.591e-4;

        constexpr double mdf = 0.1906;
        constexpr double Idfxx = 7.11e-4;
        constexpr double Idfyy = 0.629e-4;
        constexpr double Idfzz = 6.246e-4;  

        constexpr double Ibaseyy = 11.87e-4;

        constexpr double l1 = 0.215;
        constexpr double l2 = 0.170;
        constexpr double l3 = 0.0325;
        constexpr double l5 = -0.0368;
        constexpr double l6 = 0.0527;

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

        C(0,0) = 0.125*(-2*std::sin(Q[1])*((4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz
                 + 4*l1*l1*ma + l1*l1*mc)*std::cos(Q[1]) + 2*l1*(l2*ma + l3*mc)*std::sin(Q[2]))*Qd[1]
                 + 2*std::cos(Q[2])*(2*l1*(l2*ma+l3*mc)*std::cos(Q[1])
                 + (-4*Iayy + 4*Iazz - 4*Idfyy + 4*Idfzz + l2*l2*ma
                 + 4*l3*l3*mc)*std::sin(Q[2]))*Qd[2]);        
        C(0,1) = -0.125*((4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz + l1*l1*(4*ma
                 + mc))*std::sin(2*Q[1]) + 4*l1*(l2*ma + l3*mc)*std::sin(Q[1])*std::sin(Q[2]))*Qd[0];        
        C(0,2) = -0.125*(-4*l1*(l2*ma + l3*mc)*std::cos(Q[1])*std::cos(Q[2]) -(-4*Iayy
                 + 4*Iazz - 4*Idfyy + 4*Idfzz + l2*l2*ma + 4*l3*l3*mc)*std::sin(2*Q[2]))*Qd[0];                
        C(1,0) = -C(0,1);
        C(1,1) = 0;
        C(1,2) = 0.5*l1*(l2*ma+l3*mc)*std::cos(Q[1]-Q[2])*Qd[2];
        C(2,0) = -C(0,2);
        C(2,1) = 0.5*l1*(l2*ma+l3*mc)*std::cos(Q[1]-Q[2])*Qd[1];
        C(2,2) = 0;

        G(0)   = 0;
        G(1)   = 0.5*g*(2*l1*ma + 2*l5*mbe + l1*mc)*std::cos(Q[1]);
        G(2)   = 0.5*g*(l2*ma + 2*l3*mc - 2*l6*mdf)*std::sin(Q[2]);

        common_update(dt);
    }

    inline void update2(double dt) {

        constexpr double m_a  = 0.02622;
        constexpr double m_c  = 0.026146;
        constexpr double m_be = 0.26784;
        constexpr double m_df = 0.21542;
        constexpr double m_g  = 0.36159;

        constexpr double l2   = 0.20955;
        constexpr double l3   = 0.03175;

        constexpr Point Pc_a  = {0.000024230000000, -0.052378920000000, 0};
        constexpr Point Pc_c  = {0, 0.104777390000000, 0};
        constexpr Point Pc_be = {-0.035188750000000, -0.000062800000000, 0.002324160000000};
        constexpr Point Pc_df = {-0.051535410000000, 0.000609030000000, 0.004028730000000};
        constexpr Point Pc_g  = {-0.010485630000000, 0.013963870000000, -0.053808270000000};

        constexpr Tensor Ic_a  = {6.029e-05, 0, 0, 5.7e-07, 0, 6.004e-05};
        constexpr Tensor Ic_c  = {0.00013177, 0, 0, 8.9e-07, 0, 0.00013138};
        constexpr Tensor Ic_be = {0.0001133, 4.1e-07, -2.296e-05, 0.00091667, 4.8e-07, 0.00082222};
        constexpr Tensor Ic_df = {0.0001214, -3.2e-07, -7.45e-06, 0.00026133, 3.23e-06, 0.00015823};
        constexpr Tensor Ic_g  = {0.0015202, 5.838e-05, 0.00018147, 0.0012112, 0.00023832, 0.00093887};

        const double q1 = Q[0]; const double qd1 = Qd[0];
        const double q2 = Q[1]; const double qd2 = Qd[1];
        const double q3 = Q[2]; const double qd3 = Qd[2];
        const double s1 = std::sin(q1); const double c1 = std::cos(q1);
        const double s2 = std::sin(q2); const double c2 = std::cos(q2);
        const double s3 = std::sin(q3); const double c3 = std::cos(q3);
        const double s23 = std::sin(q2-q3);
        const double s32 = std::sin(q3-q2);
        const double c23 = std::cos(q2-q3);
        const double c32 = std::cos(q3-q2);

        M(0,0) = Ic_a.xx+Ic_c.yy+Ic_g.zz+Ic_be.xx+Ic_df.yy+(Pc_a.y*Pc_a.y)*m_a+(Pc_a.z*Pc_a.z)*m_a+(Pc_be.y*Pc_be.y)*m_be+(Pc_be.z*Pc_be.z)*m_be+(Pc_c.x*Pc_c.x)*m_c+(Pc_c.z*Pc_c.z)*m_c+(Pc_df.x*Pc_df.x)*m_df+(Pc_df.z*Pc_df.z)*m_df+(Pc_g.x*Pc_g.x)*m_g+(Pc_g.y*Pc_g.y)*m_g+(l3*l3)*m_c-Ic_a.xx*pow(c3,2.0)+Ic_c.xx*pow(c2,2.0)+Ic_a.yy*pow(c3,2.0)-Ic_c.yy*pow(c2,2.0)-Ic_be.xx*pow(c2,2.0)+Ic_be.yy*pow(c2,2.0)+Ic_df.xx*pow(c3,2.0)-Ic_df.yy*pow(c3,2.0)-Ic_a.xy*sin(q3*2.0)+Ic_c.xy*sin(q2*2.0)-Ic_be.xy*sin(q2*2.0)+Ic_df.xy*sin(q3*2.0)+(Pc_a.x*Pc_a.x)*m_a*pow(c3,2.0)-(Pc_a.y*Pc_a.y)*m_a*pow(c3,2.0)+(Pc_be.x*Pc_be.x)*m_be*pow(c2,2.0)-(Pc_be.y*Pc_be.y)*m_be*pow(c2,2.0)-(Pc_c.x*Pc_c.x)*m_c*pow(c2,2.0)+(Pc_c.y*Pc_c.y)*m_c*pow(c2,2.0)-(Pc_df.x*Pc_df.x)*m_df*pow(c3,2.0)+(Pc_df.y*Pc_df.y)*m_df*pow(c3,2.0)+(l2*l2)*m_a*pow(c2,2.0)-(l3*l3)*m_c*pow(c3,2.0)-Pc_a.x*Pc_a.y*m_a*sin(q3*2.0)+Pc_c.x*Pc_c.y*m_c*sin(q2*2.0)-Pc_be.x*Pc_be.y*m_be*sin(q2*2.0)+Pc_df.x*Pc_df.y*m_df*sin(q3*2.0)+Pc_a.x*l2*m_a*c2*c3*2.0-Pc_a.y*l2*m_a*c2*s3*2.0+Pc_c.y*l3*m_c*c2*s3*2.0+Pc_c.x*l3*m_c*s2*s3*2.0;
        M(0,1) = -Ic_a.yz*c3+Ic_c.xz*c2-Ic_be.yz*c2+Ic_df.xz*c3-Ic_a.xz*s3-Ic_c.yz*s2-Ic_be.xz*s2-Ic_df.yz*s3+Pc_c.x*Pc_c.z*m_c*c2-Pc_be.y*Pc_be.z*m_be*c2-Pc_c.y*Pc_c.z*m_c*s2-Pc_be.x*Pc_be.z*m_be*s2-Pc_a.z*l2*m_a*s2;
        M(0,2) = -Ic_a.yz*c3+Ic_c.xz*c2-Ic_a.xz*s3-Ic_c.yz*s2-Pc_a.y*Pc_a.z*m_a*c3+Pc_df.x*Pc_df.z*m_df*c3-Pc_a.x*Pc_a.z*m_a*s3-Pc_df.y*Pc_df.z*m_df*s3+Pc_c.z*l3*m_c*c3;
        M(1,0) = -Ic_a.yz*c3+Ic_c.xz*c2-Ic_be.yz*c2+Ic_df.xz*c3-Ic_a.xz*s3-Ic_c.yz*s2-Ic_be.xz*s2-Ic_df.yz*s3+Pc_c.x*Pc_c.z*m_c*c2-Pc_be.y*Pc_be.z*m_be*c2-Pc_c.y*Pc_c.z*m_c*s2-Pc_be.x*Pc_be.z*m_be*s2-Pc_a.z*l2*m_a*s2;
        M(1,1) = Ic_a.zz+Ic_c.zz+Ic_be.zz+Ic_df.zz+(Pc_be.x*Pc_be.x)*m_be+(Pc_be.y*Pc_be.y)*m_be+(Pc_c.x*Pc_c.x)*m_c+(Pc_c.y*Pc_c.y)*m_c+(l2*l2)*m_a;
        M(1,2) = Ic_a.zz+Ic_c.zz+Pc_a.x*l2*m_a*c23+Pc_c.x*l3*m_c*c23+Pc_a.y*l2*m_a*s23-Pc_c.y*l3*m_c*s23;
        M(2,0) = -Ic_a.yz*c3+Ic_c.xz*c2-Ic_a.xz*s3-Ic_c.yz*s2-Pc_a.y*Pc_a.z*m_a*c3+Pc_df.x*Pc_df.z*m_df*c3-Pc_a.x*Pc_a.z*m_a*s3-Pc_df.y*Pc_df.z*m_df*s3+Pc_c.z*l3*m_c*c3;
        M(2,1) = Ic_a.zz+Ic_c.zz+Pc_a.x*l2*m_a*c23+Pc_c.x*l3*m_c*c23+Pc_a.y*l2*m_a*s23-Pc_c.y*l3*m_c*s23;
        M(2,2) = Ic_a.zz+Ic_c.zz+(Pc_a.x*Pc_a.x)*m_a+(Pc_a.y*Pc_a.y)*m_a+(Pc_df.x*Pc_df.x)*m_df+(Pc_df.y*Pc_df.y)*m_df+(l3*l3)*m_c;

        V[0] = -Ic_a.xz*(qd3*qd3)*c3-Ic_c.yz*(qd2*qd2)*c2-Ic_be.xz*(qd2*qd2)*c2-Ic_c.xz*(qd2*qd2)*s2+Ic_a.yz*(qd3*qd3)*s3+Ic_be.yz*(qd2*qd2)*s2+Ic_a.xy*qd1*qd3*2.0-Ic_c.xy*qd1*qd2*2.0+Ic_be.xy*qd1*qd2*2.0-Ic_df.xy*qd1*qd3*2.0-Ic_a.xz*qd2*qd3*c3-Ic_c.yz*qd2*qd3*c2-Ic_df.yz*qd2*qd3*c3+Ic_a.yz*qd2*qd3*s3-Ic_c.xz*qd2*qd3*s2-Ic_df.xz*qd2*qd3*s3-Ic_a.xy*qd1*qd3*pow(c3,2.0)*4.0+Ic_c.xy*qd1*qd2*pow(c2,2.0)*4.0-Ic_be.xy*qd1*qd2*pow(c2,2.0)*4.0+Ic_df.xy*qd1*qd3*pow(c3,2.0)*4.0+Ic_a.xx*qd1*qd3*sin(q3*2.0)-Ic_c.xx*qd1*qd2*sin(q2*2.0)-Ic_a.yy*qd1*qd3*sin(q3*2.0)+Ic_c.yy*qd1*qd2*sin(q2*2.0)+Ic_be.xx*qd1*qd2*sin(q2*2.0)-Ic_be.yy*qd1*qd2*sin(q2*2.0)-Ic_df.xx*qd1*qd3*sin(q3*2.0)+Ic_df.yy*qd1*qd3*sin(q3*2.0)-Pc_a.x*Pc_a.z*m_a*(qd3*qd3)*c3-Pc_c.y*Pc_c.z*m_c*(qd2*qd2)*c2-Pc_be.x*Pc_be.z*m_be*(qd2*qd2)*c2-Pc_df.y*Pc_df.z*m_df*(qd3*qd3)*c3+Pc_a.y*Pc_a.z*m_a*(qd3*qd3)*s3-Pc_c.x*Pc_c.z*m_c*(qd2*qd2)*s2+Pc_be.y*Pc_be.z*m_be*(qd2*qd2)*s2-Pc_df.x*Pc_df.z*m_df*(qd3*qd3)*s3-Pc_a.z*l2*m_a*(qd2*qd2)*c2-Pc_c.z*l3*m_c*(qd3*qd3)*s3+Pc_a.x*Pc_a.y*m_a*qd1*qd3*2.0-Pc_c.x*Pc_c.y*m_c*qd1*qd2*2.0+Pc_be.x*Pc_be.y*m_be*qd1*qd2*2.0-Pc_df.x*Pc_df.y*m_df*qd1*qd3*2.0-(Pc_a.x*Pc_a.x)*m_a*qd1*qd3*sin(q3*2.0)+(Pc_a.y*Pc_a.y)*m_a*qd1*qd3*sin(q3*2.0)-(Pc_be.x*Pc_be.x)*m_be*qd1*qd2*sin(q2*2.0)+(Pc_be.y*Pc_be.y)*m_be*qd1*qd2*sin(q2*2.0)+(Pc_c.x*Pc_c.x)*m_c*qd1*qd2*sin(q2*2.0)-(Pc_c.y*Pc_c.y)*m_c*qd1*qd2*sin(q2*2.0)+(Pc_df.x*Pc_df.x)*m_df*qd1*qd3*sin(q3*2.0)-(Pc_df.y*Pc_df.y)*m_df*qd1*qd3*sin(q3*2.0)-(l2*l2)*m_a*qd1*qd2*sin(q2*2.0)+(l3*l3)*m_c*qd1*qd3*sin(q3*2.0)-Pc_a.x*Pc_a.y*m_a*qd1*qd3*pow(c3,2.0)*4.0+Pc_c.x*Pc_c.y*m_c*qd1*qd2*pow(c2,2.0)*4.0-Pc_be.x*Pc_be.y*m_be*qd1*qd2*pow(c2,2.0)*4.0+Pc_df.x*Pc_df.y*m_df*qd1*qd3*pow(c3,2.0)*4.0-Pc_a.y*l2*m_a*qd1*qd3*c2*c3*2.0+Pc_c.y*l3*m_c*qd1*qd3*c2*c3*2.0-Pc_a.x*l2*m_a*qd1*qd2*c3*s2*2.0-Pc_a.x*l2*m_a*qd1*qd3*c2*s3*2.0+Pc_c.x*l3*m_c*qd1*qd2*c2*s3*2.0+Pc_c.x*l3*m_c*qd1*qd3*c3*s2*2.0+Pc_a.y*l2*m_a*qd1*qd2*s2*s3*2.0-Pc_c.y*l3*m_c*qd1*qd2*s2*s3*2.0;
        V[1] = -Ic_c.xy*(qd1*qd1)*cos(q2*2.0)+Ic_be.xy*(qd1*qd1)*cos(q2*2.0)+(Ic_c.xx*(qd1*qd1)*sin(q2*2.0))/2.0-(Ic_c.yy*(qd1*qd1)*sin(q2*2.0))/2.0-(Ic_be.xx*(qd1*qd1)*sin(q2*2.0))/2.0+(Ic_be.yy*(qd1*qd1)*sin(q2*2.0))/2.0-Ic_a.xz*qd1*qd3*c3+Ic_c.yz*qd1*qd3*c2-Ic_df.yz*qd1*qd3*c3+Ic_a.yz*qd1*qd3*s3+Ic_c.xz*qd1*qd3*s2-Ic_df.xz*qd1*qd3*s3+((Pc_be.x*Pc_be.x)*m_be*(qd1*qd1)*sin(q2*2.0))/2.0-((Pc_be.y*Pc_be.y)*m_be*(qd1*qd1)*sin(q2*2.0))/2.0-((Pc_c.x*Pc_c.x)*m_c*(qd1*qd1)*sin(q2*2.0))/2.0+((Pc_c.y*Pc_c.y)*m_c*(qd1*qd1)*sin(q2*2.0))/2.0+((l2*l2)*m_a*(qd1*qd1)*sin(q2*2.0))/2.0-Pc_c.x*Pc_c.y*m_c*(qd1*qd1)*cos(q2*2.0)+Pc_be.x*Pc_be.y*m_be*(qd1*qd1)*cos(q2*2.0)-Pc_a.y*l2*m_a*(qd3*qd3)*c2*c3+Pc_c.y*l3*m_c*(qd3*qd3)*c2*c3+Pc_a.x*l2*m_a*(qd1*qd1)*c3*s2-Pc_a.x*l2*m_a*(qd3*qd3)*c2*s3+Pc_a.x*l2*m_a*(qd3*qd3)*c3*s2-Pc_c.x*l3*m_c*(qd1*qd1)*c2*s3-Pc_c.x*l3*m_c*(qd3*qd3)*c2*s3+Pc_c.x*l3*m_c*(qd3*qd3)*c3*s2-Pc_a.y*l2*m_a*(qd1*qd1)*s2*s3-Pc_a.y*l2*m_a*(qd3*qd3)*s2*s3+Pc_c.y*l3*m_c*(qd1*qd1)*s2*s3+Pc_c.y*l3*m_c*(qd3*qd3)*s2*s3;
        V[2] = Ic_a.xy*(qd1*qd1)*cos(q3*2.0)-Ic_df.xy*(qd1*qd1)*cos(q3*2.0)-(Ic_a.xx*(qd1*qd1)*sin(q3*2.0))/2.0+(Ic_a.yy*(qd1*qd1)*sin(q3*2.0))/2.0+(Ic_df.xx*(qd1*qd1)*sin(q3*2.0))/2.0-(Ic_df.yy*(qd1*qd1)*sin(q3*2.0))/2.0+Ic_a.xz*qd1*qd2*c3-Ic_c.yz*qd1*qd2*c2+Ic_df.yz*qd1*qd2*c3-Ic_a.yz*qd1*qd2*s3-Ic_c.xz*qd1*qd2*s2+Ic_df.xz*qd1*qd2*s3+((Pc_a.x*Pc_a.x)*m_a*(qd1*qd1)*sin(q3*2.0))/2.0-((Pc_a.y*Pc_a.y)*m_a*(qd1*qd1)*sin(q3*2.0))/2.0-((Pc_df.x*Pc_df.x)*m_df*(qd1*qd1)*sin(q3*2.0))/2.0+((Pc_df.y*Pc_df.y)*m_df*(qd1*qd1)*sin(q3*2.0))/2.0-((l3*l3)*m_c*(qd1*qd1)*sin(q3*2.0))/2.0+Pc_a.x*Pc_a.y*m_a*(qd1*qd1)*cos(q3*2.0)-Pc_df.x*Pc_df.y*m_df*(qd1*qd1)*cos(q3*2.0)+Pc_a.y*l2*m_a*(qd1*qd1)*c2*c3+Pc_a.y*l2*m_a*(qd2*qd2)*c2*c3-Pc_c.y*l3*m_c*(qd1*qd1)*c2*c3-Pc_c.y*l3*m_c*(qd2*qd2)*c2*c3+Pc_a.x*l2*m_a*(qd1*qd1)*c2*s3+Pc_a.x*l2*m_a*(qd2*qd2)*c2*s3-Pc_a.x*l2*m_a*(qd2*qd2)*c3*s2-Pc_c.x*l3*m_c*(qd1*qd1)*c3*s2+Pc_c.x*l3*m_c*(qd2*qd2)*c2*s3-Pc_c.x*l3*m_c*(qd2*qd2)*c3*s2+Pc_a.y*l2*m_a*(qd2*qd2)*s2*s3-Pc_c.y*l3*m_c*(qd2*qd2)*s2*s3;

        G[0] = 0;   
        G[1] = g*(-Pc_be.y*m_be*s2+Pc_c.x*m_c*s2+l2*m_a*c2+Pc_be.x*m_be*c2+Pc_c.y*m_c*c2);
        G[2] = g*(-Pc_a.y*m_a*s3+Pc_df.x*m_df*s3+l3*m_c*s3+Pc_a.x*m_a*c3+Pc_df.y*m_df*c3);

        common_update(dt);
    }


    inline void common_update(double dt) {
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

        Qdd = M.householderQr().solve(Tau_prime - C*Qd - G);
        Qd  = Qdd_Qd.integrate(Qdd, dt);
        Q   = Qd_Q.integrate(Qd, dt);
    }


    inline void reset() {
        Qdd_Qd.val = Qd_Q.val = Qdd = Qd = Q = Tau = Vector3d::Zero();
    }

    std::vector<double> fk(Vector3d q){
        constexpr double l1 = 0.210;
        constexpr double l2 = 0.170;
        double theta1 = q(0);
        double theta2 = q(1);
        double theta3 = q(2);
        std::vector<double> ee_pos = {cos(theta1)*(l1*cos(theta2) + l2*sin(theta3)),
                                      sin(theta1)*(l1*cos(theta2) + l2*sin(theta3)),
                                      l1*sin(theta2) - l2*cos(theta3)};

        return ee_pos; 
    }

    std::vector<double> ik(Point ee_pos, std::vector<double> ref_angles){
        constexpr double l1 = 0.210;
        constexpr double l2 = 0.170;

        double l_star = sqrt(ee_pos.x*ee_pos.x + ee_pos.y*ee_pos.y);
        double ee_theta = atan2(ee_pos.z,l_star);
        double lh = sqrt(ee_pos.z*ee_pos.z + l_star*l_star);

        double theta1_1 = atan2(ee_pos.y,ee_pos.x);
        double theta2_1 = acos((l2*l2-l1*l1-lh*lh)/(-2*l1*lh))+ee_theta;
        double theta3_1 = acos((lh*lh-l1*l1-l2*l2)/(-2*l1*l2))+theta2_1-mahi::util::PI/2;

        double theta1_2 = mahi::util::wrap_to_pi(atan2(ee_pos.y,ee_pos.x)-mahi::util::PI);
        double theta2_2 = mahi::util::PI-ee_theta+acos((l2*l2-l1*l1-lh*lh)/(-2*l1*lh));
        double theta3_2 = acos((lh*lh-l1*l1-l2*l2)/(-2*l1*l2))+theta2_2-mahi::util::PI/2;

        double theta1_min = -90  * mahi::util::DEG2RAD;
        double theta1_max =  90  * mahi::util::DEG2RAD;
        double theta2_min = -124 * mahi::util::DEG2RAD;
        double theta2_max =  119 * mahi::util::DEG2RAD;
        double theta3_min = -29  * mahi::util::DEG2RAD;  
        double theta3_max =  214 * mahi::util::DEG2RAD;

        bool set1_valid = (theta1_1 < theta1_max) && (theta1_1 > theta1_min) && (theta2_1 < theta2_max) && (theta2_1 > theta2_min) && (theta3_1 < theta3_max) && (theta3_1 > theta3_min) && ((theta3_1 - theta2_1) <  55.0 * DEG2RAD) && ((theta3_1 - theta2_1) > -65.0 * DEG2RAD);

        bool set2_valid = (theta1_2 < theta1_max) && (theta1_2 > theta1_min) && (theta2_2 < theta2_max) && (theta2_2 > theta2_min) && (theta3_2 < theta3_max) && (theta3_2 > theta3_min) && ((theta3_2 - theta2_2) <  55.0 * DEG2RAD) && ((theta3_2 - theta2_2) > -65.0 * DEG2RAD);

        double sum_err1 = (ref_angles[0]-theta1_1)*(ref_angles[0]-theta1_1) + (ref_angles[1]-theta2_1)*(ref_angles[1]-theta2_1) + (ref_angles[2]-theta3_1)*(ref_angles[2]-theta3_1);
        double sum_err2 = (ref_angles[0]-theta1_2)*(ref_angles[0]-theta1_2) + (ref_angles[1]-theta2_2)*(ref_angles[1]-theta2_2) + (ref_angles[2]-theta3_2)*(ref_angles[2]-theta3_2);
        
        if ((sum_err1 < sum_err2) && set1_valid) return {theta1_1, theta2_1, theta3_1};
        else if (set2_valid) return {theta1_2, theta2_2, theta3_2};
        else return {0,0,0};
    }

    Matrix3d jacobian(Vector3d q){
        constexpr double l1 = 0.210;
        constexpr double l2 = 0.170;
        double theta1 = q[0]; 
        double theta2 = q[1]; 
        double theta3 = q[2];
        
        Matrix3d jac;
        
        jac(0,0) = -sin(theta1)*(l1*cos(theta2) + l2*sin(theta3));
        jac(0,1) = -l1*cos(theta1)*sin(theta2);
        jac(0,2) = l2*cos(theta1)*cos(theta3);
        jac(1,0) = cos(theta1)*(l1*cos(theta2) + l2*sin(theta3));
        jac(1,0) = -l1*sin(theta1)*sin(theta2);
        jac(1,0) = l2*cos(theta3)*sin(theta1);
        jac(2,0) = 0;
        jac(2,1) = l1*cos(theta2);
        jac(2,2) = l2*sin(theta3);

        return jac;
    }

public:
    Matrix3d M, C;
    Vector3d Q, Qd, Qdd, V, G, Tau;
    Integrator<Vector3d> Qdd_Qd, Qd_Q;

    static constexpr double g = 9.80665;
    double Khard = 50; 
    double Bhard = 0.5;
};
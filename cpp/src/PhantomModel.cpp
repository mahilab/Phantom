#include "PhantomModel.hpp"

using std::cos;
using std::sin;
using std::pow;

#define q1 Q[0]
#define q2 Q[1]
#define q3 Q[2]

#define qd1 Qd[0]
#define qd2 Qd[1]
#define qd3 Qd[2]

#define PHANTOM_TRIG const double s1 = std::sin(q1); const double c1 = std::cos(q1); \
                     const double s2 = std::sin(q2); const double c2 = std::cos(q2); \
                     const double s3 = std::sin(q3); const double c3 = std::cos(q3); 

#define PHANTOM_TRIG_EX const double s23 = std::sin(q2-q3); \
                        const double s32 = std::sin(q3-q2); \
                        const double c23 = std::cos(q2-q3); \
                        const double c32 = std::cos(q3-q2);                              

namespace Phantom {
namespace Model {

bool validate_angles(const Vector3d &Q) {
    for (int i = 0; i < 3; ++i) {
        if (Q[i] < Q_min[i] || Q[i] > Q_max[i])
            return false;
    }
    if ((q2 - q3) > Q23_max)
        return false;
    if ((q3 - q2) > Q32_max)
        return false;
    return true;
}

Point forward_kinematics(const Vector3d &Q) {
    PHANTOM_TRIG
    Point P;
    // copy/paste from MATLAB jacobian.m
    P.x = c1 * (l1 * c2 + l2 * s3);
    P.y = s1 * (l1 * c2 + l2 * s3);
    P.z = l1 * s2 - l2 * c3;
    return P;
}

Vector3d inverse_kinematics(const Point& P, const Vector3d Q_ref) {
    double l_star = std::sqrt(P.x * P.x + P.y * P.y);
    double ee_theta = std::atan2(P.z, l_star);
    double lh = std::sqrt(P.z * P.z + l_star * l_star);
    Vector3d Q_a, Q_b;

    Q_a[0] = std::atan2(P.y, P.x);
    Q_a[1] = std::acos((l2 * l2 - l1 * l1 - lh * lh) / (-2 * l1 * lh)) + ee_theta;
    Q_a[2] = std::acos((lh * lh - l1 * l1 - l2 * l2) / (-2 * l1 * l2)) + Q_a[1] - mahi::util::PI / 2;

    Q_b[0] = mahi::util::wrap_to_pi(atan2(P.y, P.x) - mahi::util::PI);
    Q_b[1] = mahi::util::PI - ee_theta + std::acos((l2 * l2 - l1 * l1 - lh * lh) / (-2 * l1 * lh));
    Q_b[2] = std::acos((lh * lh - l1 * l1 - l2 * l2) / (-2 * l1 * l2)) + Q_b[1] - mahi::util::PI / 2;

    bool a_valid = validate_angles(Q_a);
    bool b_valid = validate_angles(Q_b);

    double a_err = (Q_ref - Q_a).squaredNorm();
    double b_err = (Q_ref - Q_b).squaredNorm();

    if ((a_err < b_err) && a_valid) 
        return Q_a;
    else if (b_valid) 
        return Q_b;
    else 
        return Vector3d::Zero();
}

Matrix3d jacobian(const Vector3d& Q) {
    PHANTOM_TRIG
    Matrix3d J;
    // copy/paste from MATLAB jacobian.m
    J(0, 0) = -s1 * (l1 * c2 + l2 * s3);
    J(0, 1) = -l1 * c1 * s2;
    J(0, 2) = l2 * c1 * c3;
    J(1, 0) = c1 * (l1 * c2 + l2 * s3);
    J(1, 1) = -l1 * s1 * s2;
    J(1, 2) = l2 * c3 * s1;
    J(2, 1) = l1 * c2;
    J(2, 2) = l2 * s3;
    return J;
}

Matrix3d M(const Vector3d& Q) {
    Matrix3d _M;
    PHANTOM_TRIG;
    PHANTOM_TRIG_EX;
    // copy/paste from MATLAB dynamics.m
    _M(0,0) = Ic_a.xx+Ic_c.yy+Ic_g.zz+Ic_be.xx+Ic_df.yy+(Pc_a.y*Pc_a.y)*m_a+(Pc_a.z*Pc_a.z)*m_a+(Pc_be.y*Pc_be.y)*m_be+(Pc_be.z*Pc_be.z)*m_be+(Pc_c.x*Pc_c.x)*m_c+(Pc_c.z*Pc_c.z)*m_c+(Pc_df.x*Pc_df.x)*m_df+(Pc_df.z*Pc_df.z)*m_df+(Pc_g.x*Pc_g.x)*m_g+(Pc_g.y*Pc_g.y)*m_g+(l3*l3)*m_c-Ic_a.xx*pow(c3,2.0)+Ic_c.xx*pow(c2,2.0)+Ic_a.yy*pow(c3,2.0)-Ic_c.yy*pow(c2,2.0)-Ic_be.xx*pow(c2,2.0)+Ic_be.yy*pow(c2,2.0)+Ic_df.xx*pow(c3,2.0)-Ic_df.yy*pow(c3,2.0)-Ic_a.xy*sin(q3*2.0)+Ic_c.xy*sin(q2*2.0)-Ic_be.xy*sin(q2*2.0)+Ic_df.xy*sin(q3*2.0)+(Pc_a.x*Pc_a.x)*m_a*pow(c3,2.0)-(Pc_a.y*Pc_a.y)*m_a*pow(c3,2.0)+(Pc_be.x*Pc_be.x)*m_be*pow(c2,2.0)-(Pc_be.y*Pc_be.y)*m_be*pow(c2,2.0)-(Pc_c.x*Pc_c.x)*m_c*pow(c2,2.0)+(Pc_c.y*Pc_c.y)*m_c*pow(c2,2.0)-(Pc_df.x*Pc_df.x)*m_df*pow(c3,2.0)+(Pc_df.y*Pc_df.y)*m_df*pow(c3,2.0)+(l1*l1)*m_a*pow(c2,2.0)-(l3*l3)*m_c*pow(c3,2.0)-Pc_a.x*Pc_a.y*m_a*sin(q3*2.0)+Pc_c.x*Pc_c.y*m_c*sin(q2*2.0)-Pc_be.x*Pc_be.y*m_be*sin(q2*2.0)+Pc_df.x*Pc_df.y*m_df*sin(q3*2.0)+Pc_a.x*l1*m_a*c2*c3*2.0-Pc_a.y*l1*m_a*c2*s3*2.0+Pc_c.y*l3*m_c*c2*s3*2.0+Pc_c.x*l3*m_c*s2*s3*2.0;
    _M(0,1) = -Ic_a.yz*c3+Ic_c.xz*c2-Ic_be.yz*c2+Ic_df.xz*c3-Ic_a.xz*s3-Ic_c.yz*s2-Ic_be.xz*s2-Ic_df.yz*s3+Pc_c.x*Pc_c.z*m_c*c2-Pc_be.y*Pc_be.z*m_be*c2-Pc_c.y*Pc_c.z*m_c*s2-Pc_be.x*Pc_be.z*m_be*s2-Pc_a.z*l1*m_a*s2;
    _M(0,2) = -Ic_a.yz*c3+Ic_c.xz*c2-Ic_a.xz*s3-Ic_c.yz*s2-Pc_a.y*Pc_a.z*m_a*c3+Pc_df.x*Pc_df.z*m_df*c3-Pc_a.x*Pc_a.z*m_a*s3-Pc_df.y*Pc_df.z*m_df*s3+Pc_c.z*l3*m_c*c3;
    _M(1,0) = -Ic_a.yz*c3+Ic_c.xz*c2-Ic_be.yz*c2+Ic_df.xz*c3-Ic_a.xz*s3-Ic_c.yz*s2-Ic_be.xz*s2-Ic_df.yz*s3+Pc_c.x*Pc_c.z*m_c*c2-Pc_be.y*Pc_be.z*m_be*c2-Pc_c.y*Pc_c.z*m_c*s2-Pc_be.x*Pc_be.z*m_be*s2-Pc_a.z*l1*m_a*s2;
    _M(1,1) = Ic_a.zz+Ic_c.zz+Ic_be.zz+Ic_df.zz+(Pc_be.x*Pc_be.x)*m_be+(Pc_be.y*Pc_be.y)*m_be+(Pc_c.x*Pc_c.x)*m_c+(Pc_c.y*Pc_c.y)*m_c+(l1*l1)*m_a;
    _M(1,2) = Ic_a.zz+Ic_c.zz+Pc_a.x*l1*m_a*c23+Pc_c.x*l3*m_c*c23+Pc_a.y*l1*m_a*s23-Pc_c.y*l3*m_c*s23;
    _M(2,0) = -Ic_a.yz*c3+Ic_c.xz*c2-Ic_a.xz*s3-Ic_c.yz*s2-Pc_a.y*Pc_a.z*m_a*c3+Pc_df.x*Pc_df.z*m_df*c3-Pc_a.x*Pc_a.z*m_a*s3-Pc_df.y*Pc_df.z*m_df*s3+Pc_c.z*l3*m_c*c3;
    _M(2,1) = Ic_a.zz+Ic_c.zz+Pc_a.x*l1*m_a*c23+Pc_c.x*l3*m_c*c23+Pc_a.y*l1*m_a*s23-Pc_c.y*l3*m_c*s23;
    _M(2,2) = Ic_a.zz+Ic_c.zz+(Pc_a.x*Pc_a.x)*m_a+(Pc_a.y*Pc_a.y)*m_a+(Pc_df.x*Pc_df.x)*m_df+(Pc_df.y*Pc_df.y)*m_df+(l3*l3)*m_c;
    return _M;
}


Vector3d V(const Vector3d& Q, const Vector3d& Qd) {
    Vector3d _V;
    PHANTOM_TRIG;
    PHANTOM_TRIG_EX;
    // copy/paste from MATLAB dynamics.m
    _V[0] = -Ic_a.xz*(qd3*qd3)*c3-Ic_c.yz*(qd2*qd2)*c2-Ic_be.xz*(qd2*qd2)*c2-Ic_c.xz*(qd2*qd2)*s2+Ic_a.yz*(qd3*qd3)*s3+Ic_be.yz*(qd2*qd2)*s2+Ic_a.xy*qd1*qd3*2.0-Ic_c.xy*qd1*qd2*2.0+Ic_be.xy*qd1*qd2*2.0-Ic_df.xy*qd1*qd3*2.0-Ic_a.xz*qd2*qd3*c3-Ic_c.yz*qd2*qd3*c2-Ic_df.yz*qd2*qd3*c3+Ic_a.yz*qd2*qd3*s3-Ic_c.xz*qd2*qd3*s2-Ic_df.xz*qd2*qd3*s3-Ic_a.xy*qd1*qd3*pow(c3,2.0)*4.0+Ic_c.xy*qd1*qd2*pow(c2,2.0)*4.0-Ic_be.xy*qd1*qd2*pow(c2,2.0)*4.0+Ic_df.xy*qd1*qd3*pow(c3,2.0)*4.0+Ic_a.xx*qd1*qd3*sin(q3*2.0)-Ic_c.xx*qd1*qd2*sin(q2*2.0)-Ic_a.yy*qd1*qd3*sin(q3*2.0)+Ic_c.yy*qd1*qd2*sin(q2*2.0)+Ic_be.xx*qd1*qd2*sin(q2*2.0)-Ic_be.yy*qd1*qd2*sin(q2*2.0)-Ic_df.xx*qd1*qd3*sin(q3*2.0)+Ic_df.yy*qd1*qd3*sin(q3*2.0)-Pc_a.x*Pc_a.z*m_a*(qd3*qd3)*c3-Pc_c.y*Pc_c.z*m_c*(qd2*qd2)*c2-Pc_be.x*Pc_be.z*m_be*(qd2*qd2)*c2-Pc_df.y*Pc_df.z*m_df*(qd3*qd3)*c3+Pc_a.y*Pc_a.z*m_a*(qd3*qd3)*s3-Pc_c.x*Pc_c.z*m_c*(qd2*qd2)*s2+Pc_be.y*Pc_be.z*m_be*(qd2*qd2)*s2-Pc_df.x*Pc_df.z*m_df*(qd3*qd3)*s3-Pc_a.z*l1*m_a*(qd2*qd2)*c2-Pc_c.z*l3*m_c*(qd3*qd3)*s3+Pc_a.x*Pc_a.y*m_a*qd1*qd3*2.0-Pc_c.x*Pc_c.y*m_c*qd1*qd2*2.0+Pc_be.x*Pc_be.y*m_be*qd1*qd2*2.0-Pc_df.x*Pc_df.y*m_df*qd1*qd3*2.0-(Pc_a.x*Pc_a.x)*m_a*qd1*qd3*sin(q3*2.0)+(Pc_a.y*Pc_a.y)*m_a*qd1*qd3*sin(q3*2.0)-(Pc_be.x*Pc_be.x)*m_be*qd1*qd2*sin(q2*2.0)+(Pc_be.y*Pc_be.y)*m_be*qd1*qd2*sin(q2*2.0)+(Pc_c.x*Pc_c.x)*m_c*qd1*qd2*sin(q2*2.0)-(Pc_c.y*Pc_c.y)*m_c*qd1*qd2*sin(q2*2.0)+(Pc_df.x*Pc_df.x)*m_df*qd1*qd3*sin(q3*2.0)-(Pc_df.y*Pc_df.y)*m_df*qd1*qd3*sin(q3*2.0)-(l1*l1)*m_a*qd1*qd2*sin(q2*2.0)+(l3*l3)*m_c*qd1*qd3*sin(q3*2.0)-Pc_a.x*Pc_a.y*m_a*qd1*qd3*pow(c3,2.0)*4.0+Pc_c.x*Pc_c.y*m_c*qd1*qd2*pow(c2,2.0)*4.0-Pc_be.x*Pc_be.y*m_be*qd1*qd2*pow(c2,2.0)*4.0+Pc_df.x*Pc_df.y*m_df*qd1*qd3*pow(c3,2.0)*4.0-Pc_a.y*l1*m_a*qd1*qd3*c2*c3*2.0+Pc_c.y*l3*m_c*qd1*qd3*c2*c3*2.0-Pc_a.x*l1*m_a*qd1*qd2*c3*s2*2.0-Pc_a.x*l1*m_a*qd1*qd3*c2*s3*2.0+Pc_c.x*l3*m_c*qd1*qd2*c2*s3*2.0+Pc_c.x*l3*m_c*qd1*qd3*c3*s2*2.0+Pc_a.y*l1*m_a*qd1*qd2*s2*s3*2.0-Pc_c.y*l3*m_c*qd1*qd2*s2*s3*2.0;
    _V[1] = -Ic_c.xy*(qd1*qd1)*cos(q2*2.0)+Ic_be.xy*(qd1*qd1)*cos(q2*2.0)+(Ic_c.xx*(qd1*qd1)*sin(q2*2.0))/2.0-(Ic_c.yy*(qd1*qd1)*sin(q2*2.0))/2.0-(Ic_be.xx*(qd1*qd1)*sin(q2*2.0))/2.0+(Ic_be.yy*(qd1*qd1)*sin(q2*2.0))/2.0-Ic_a.xz*qd1*qd3*c3+Ic_c.yz*qd1*qd3*c2-Ic_df.yz*qd1*qd3*c3+Ic_a.yz*qd1*qd3*s3+Ic_c.xz*qd1*qd3*s2-Ic_df.xz*qd1*qd3*s3+((Pc_be.x*Pc_be.x)*m_be*(qd1*qd1)*sin(q2*2.0))/2.0-((Pc_be.y*Pc_be.y)*m_be*(qd1*qd1)*sin(q2*2.0))/2.0-((Pc_c.x*Pc_c.x)*m_c*(qd1*qd1)*sin(q2*2.0))/2.0+((Pc_c.y*Pc_c.y)*m_c*(qd1*qd1)*sin(q2*2.0))/2.0+((l1*l1)*m_a*(qd1*qd1)*sin(q2*2.0))/2.0-Pc_c.x*Pc_c.y*m_c*(qd1*qd1)*cos(q2*2.0)+Pc_be.x*Pc_be.y*m_be*(qd1*qd1)*cos(q2*2.0)-Pc_a.y*l1*m_a*(qd3*qd3)*c2*c3+Pc_c.y*l3*m_c*(qd3*qd3)*c2*c3+Pc_a.x*l1*m_a*(qd1*qd1)*c3*s2-Pc_a.x*l1*m_a*(qd3*qd3)*c2*s3+Pc_a.x*l1*m_a*(qd3*qd3)*c3*s2-Pc_c.x*l3*m_c*(qd1*qd1)*c2*s3-Pc_c.x*l3*m_c*(qd3*qd3)*c2*s3+Pc_c.x*l3*m_c*(qd3*qd3)*c3*s2-Pc_a.y*l1*m_a*(qd1*qd1)*s2*s3-Pc_a.y*l1*m_a*(qd3*qd3)*s2*s3+Pc_c.y*l3*m_c*(qd1*qd1)*s2*s3+Pc_c.y*l3*m_c*(qd3*qd3)*s2*s3;
    _V[2] = Ic_a.xy*(qd1*qd1)*cos(q3*2.0)-Ic_df.xy*(qd1*qd1)*cos(q3*2.0)-(Ic_a.xx*(qd1*qd1)*sin(q3*2.0))/2.0+(Ic_a.yy*(qd1*qd1)*sin(q3*2.0))/2.0+(Ic_df.xx*(qd1*qd1)*sin(q3*2.0))/2.0-(Ic_df.yy*(qd1*qd1)*sin(q3*2.0))/2.0+Ic_a.xz*qd1*qd2*c3-Ic_c.yz*qd1*qd2*c2+Ic_df.yz*qd1*qd2*c3-Ic_a.yz*qd1*qd2*s3-Ic_c.xz*qd1*qd2*s2+Ic_df.xz*qd1*qd2*s3+((Pc_a.x*Pc_a.x)*m_a*(qd1*qd1)*sin(q3*2.0))/2.0-((Pc_a.y*Pc_a.y)*m_a*(qd1*qd1)*sin(q3*2.0))/2.0-((Pc_df.x*Pc_df.x)*m_df*(qd1*qd1)*sin(q3*2.0))/2.0+((Pc_df.y*Pc_df.y)*m_df*(qd1*qd1)*sin(q3*2.0))/2.0-((l3*l3)*m_c*(qd1*qd1)*sin(q3*2.0))/2.0+Pc_a.x*Pc_a.y*m_a*(qd1*qd1)*cos(q3*2.0)-Pc_df.x*Pc_df.y*m_df*(qd1*qd1)*cos(q3*2.0)+Pc_a.y*l1*m_a*(qd1*qd1)*c2*c3+Pc_a.y*l1*m_a*(qd2*qd2)*c2*c3-Pc_c.y*l3*m_c*(qd1*qd1)*c2*c3-Pc_c.y*l3*m_c*(qd2*qd2)*c2*c3+Pc_a.x*l1*m_a*(qd1*qd1)*c2*s3+Pc_a.x*l1*m_a*(qd2*qd2)*c2*s3-Pc_a.x*l1*m_a*(qd2*qd2)*c3*s2-Pc_c.x*l3*m_c*(qd1*qd1)*c3*s2+Pc_c.x*l3*m_c*(qd2*qd2)*c2*s3-Pc_c.x*l3*m_c*(qd2*qd2)*c3*s2+Pc_a.y*l1*m_a*(qd2*qd2)*s2*s3-Pc_c.y*l3*m_c*(qd2*qd2)*s2*s3;
    return _V;
}

Vector3d G(const Vector3d& Q) {
    Vector3d _G;
    PHANTOM_TRIG
    // copy/paste from MATLAB dynamics.m
    _G[0] = 0;
    _G[1] = g*(-Pc_be.y*m_be*s2+Pc_c.x*m_c*s2+l1*m_a*c2+Pc_be.x*m_be*c2+Pc_c.y*m_c*c2);
    _G[2] = g*(-Pc_a.y*m_a*s3+Pc_df.x*m_df*s3+l3*m_c*s3+Pc_a.x*m_a*c3+Pc_df.y*m_df*c3);
    return _G;
}

Eigen::Matrix<double,5,1> K(const Vector3d& Q, const Vector3d& Qd) {
    PHANTOM_TRIG;
    Eigen::Matrix<double,5,1> _K;
    _K[0] = ((qd1*qd1)*(Ic_g.zz+(Pc_g.x*Pc_g.x)*m_g+(Pc_g.y*Pc_g.y)*m_g))/2.0;
    _K[1] = (m_be*pow(qd1*(Pc_be.z*c1-Pc_be.x*c2*s1+Pc_be.y*s1*s2)-qd2*(Pc_be.y*c1*c2+Pc_be.x*c1*s2),2.0))/2.0+(m_be*pow(qd2*(Pc_be.y*c2*s1+Pc_be.x*s1*s2)-qd1*(Pc_be.z*s1+Pc_be.x*c1*c2-Pc_be.y*c1*s2),2.0))/2.0-qd2*(Ic_be.zz*qd2*(-1.0/2.0)+(Ic_be.yz*qd1*c2)/2.0+(Ic_be.xz*qd1*s2)/2.0)-qd1*c2*((Ic_be.yz*qd2)/2.0-(Ic_be.yy*qd1*c2)/2.0+(Ic_be.xy*qd1*s2)/2.0)-qd1*s2*((Ic_be.xz*qd2)/2.0+(Ic_be.xy*qd1*c2)/2.0-(Ic_be.xx*qd1*s2)/2.0)+(m_be*(qd2*qd2)*pow(Pc_be.x*c2-Pc_be.y*s2,2.0))/2.0;
    _K[2] = (m_a*pow(-qd1*(Pc_a.z*c1-Pc_a.x*c3*s1+Pc_a.y*s1*s3-l1*c2*s1)+qd3*(Pc_a.y*c1*c3+Pc_a.x*c1*s3)+l1*qd2*c1*s2,2.0))/2.0-(qd2+qd3)*(-Ic_a.zz*(qd2/2.0+qd3/2.0)+(Ic_a.yz*qd1*c3)/2.0+(Ic_a.xz*qd1*s3)/2.0)+(m_a*pow(qd3*(Pc_a.y*c3*s1+Pc_a.x*s1*s3)-qd1*(Pc_a.z*s1+Pc_a.x*c1*c3-Pc_a.y*c1*s3+l1*c1*c2)+l1*qd2*s1*s2,2.0))/2.0+(m_a*pow(qd3*(Pc_a.x*c3-Pc_a.y*s3)+l1*qd2*c2,2.0))/2.0-qd1*c3*(Ic_a.yz*(qd2/2.0+qd3/2.0)-(Ic_a.yy*qd1*c3)/2.0+(Ic_a.xy*qd1*s3)/2.0)-qd1*s3*(Ic_a.xz*(qd2/2.0+qd3/2.0)+(Ic_a.xy*qd1*c3)/2.0-(Ic_a.xx*qd1*s3)/2.0);
    _K[3] = (m_df*pow(qd1*(-Pc_df.z*c1+Pc_df.y*c3*s1+Pc_df.x*s1*s3)-qd3*(Pc_df.x*c1*c3-Pc_df.y*c1*s3),2.0))/2.0+(m_df*pow(qd3*(Pc_df.x*c3*s1-Pc_df.y*s1*s3)+qd1*(Pc_df.z*s1+Pc_df.y*c1*c3+Pc_df.x*c1*s3),2.0))/2.0+qd2*((Ic_df.zz*qd2)/2.0+(Ic_df.xz*qd1*c3)/2.0-(Ic_df.yz*qd1*s3)/2.0)+qd1*c3*((Ic_df.xz*qd2)/2.0+(Ic_df.xx*qd1*c3)/2.0+(Ic_df.xy*qd1*s3)/2.0)+qd1*s3*(Ic_df.yz*qd2*(-1.0/2.0)+(Ic_df.xy*qd1*c3)/2.0+(Ic_df.yy*qd1*s3)/2.0)+(m_df*(qd3*qd3)*pow(Pc_df.y*c3+Pc_df.x*s3,2.0))/2.0;
    _K[4] = (m_c*pow(-qd1*(-Pc_c.z*c1+Pc_c.y*c2*s1+Pc_c.x*s1*s2+l3*s1*s3)+qd2*(Pc_c.x*c1*c2-Pc_c.y*c1*s2)+l3*qd3*c1*c3,2.0))/2.0+(qd2+qd3)*(Ic_c.zz*(qd2/2.0+qd3/2.0)+(Ic_c.xz*qd1*c2)/2.0-(Ic_c.yz*qd1*s2)/2.0)+(m_c*pow(qd1*(Pc_c.z*s1+Pc_c.y*c1*c2+Pc_c.x*c1*s2+l3*c1*s3)+qd2*(Pc_c.x*c2*s1-Pc_c.y*s1*s2)+l3*qd3*c3*s1,2.0))/2.0+(m_c*pow(qd2*(Pc_c.y*c2+Pc_c.x*s2)+l3*qd3*s3,2.0))/2.0+qd1*c2*(Ic_c.xz*(qd2/2.0+qd3/2.0)+(Ic_c.xx*qd1*c2)/2.0+(Ic_c.xy*qd1*s2)/2.0)+qd1*s2*(-Ic_c.yz*(qd2/2.0+qd3/2.0)+(Ic_c.xy*qd1*c2)/2.0+(Ic_c.yy*qd1*s2)/2.0);
    return _K;
}

Eigen::Matrix<double,5,1> U(const Vector3d& Q) {
    PHANTOM_TRIG;
    Eigen::Matrix<double,5,1> _U;
    _U[0] = Pc_g.z*g*m_g;
    _U[1] = g*m_be*(Pc_be.y*c2+Pc_be.x*s2);
    _U[2] = g*m_a*(Pc_a.y*c3+Pc_a.x*s3+l1*s2);
    _U[3] = -g*m_df*(Pc_df.x*c3-Pc_df.y*s3);
    _U[4] = -g*m_c*(Pc_c.x*c2-Pc_c.y*s2+l3*c3);
    return _U;
}

inline double hardstop(double q, double qd, double qmin, double qmax, double K, double B) {
    if (q < qmin)
        return K * (qmin - q) - B * qd;
    else if (q > qmax)
        return K * (qmax - q) - B * qd;
    else
        return 0;
}

inline Vector3d collision_torques(const Vector3d& Q, const Vector3d& Qd) {
    Vector3d col;
    // determine min/max joint angles
    Vector3d qmins;
    Vector3d qmaxs;
    qmins[0] = Q_min[0];
    qmaxs[0] = Q_max[0];
    qmins[1] = std::max(Q_min[1], Q[2] - Q32_max);
    qmaxs[1] = std::min(Q_max[1], Q[2] + Q23_max);
    qmins[2] = std::max(Q_min[2], Q[1] - Q23_max);  
    qmaxs[2] = std::min(Q_max[2], Q[1] + Q32_max);
    for (int i = 0; i < 3; ++i)
        col[i] = hardstop(Q[i], Qd[i], qmins[i], qmaxs[i], 10, 0.1);
    return col;
}

inline void integrate_state(State& s, const Vector3d& Qdd, double dt) {
    Vector3d Qd  = s.Qd + dt * 0.5 * (s.Qdd + Qdd);
    Vector3d Q   = s.Q  + dt * 0.5 * (s.Qd  + Qd);
    s.Qdd = Qdd; s.Qd = Qd; s.Q = Q;
}

void step_dynamics(State& s, double dt) {
    // external disturbances and collision torques
    Vector3d Tau_prime = collision_torques(s.Q, s.Qd);
    // reflected motor inertia
    static Matrix3d M_prime((Matrix3d() << Jm * Eta[0] * Eta[0], 0, 0,
                                           0, Jm * Eta[1] * Eta[1], 0,
                                           0, 0, Jm * Eta[2] * Eta[2] ).finished());
    // joint damping
    Vector3d B;
    B[0] = B_coef[0]*s.Qd[0];
    B[1] = B_coef[1]*s.Qd[1];
    B[2] = B_coef[2]*s.Qd[2];
    // joint friction
    Vector3d Fk;
    Fk[0] = Fk_coef[0]*std::tanh(s.Qd[0]*10);
    Fk[1] = Fk_coef[1]*std::tanh(s.Qd[1]*10);
    Fk[2] = Fk_coef[2]*std::tanh(s.Qd[2]*10);
    // Tau + Tau' = [M(Q) + M'] * Qdd + V(Q,Qdd) + G(Q) + B(Qd) + Fk(Qd)
    Matrix3d A = M(s.Q) + M_prime;
    Vector3d b = s.Tau + Tau_prime - V(s.Q,s.Qd) - G(s.Q) - B - Fk;    
    Vector3d Qdd = A.householderQr().solve(b);
    integrate_state(s, Qdd, dt);
}

void step_dynamics_cavusoglu(State& s, double dt) {

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

    Matrix3d _M;
    _M(0,0) = (0.125*(4*Iayy + 4*Iazz + 8*Ibaseyy + 4*Ibeyy + 4*Ibezz + 4*Icyy 
            + 4*Iczz + 4*Idfyy + 4*Idfzz + 4*l1*l1*ma + l2*l2*ma + l1*l1*mc + 4*l3*l3*mc) 
            + 0.125*(4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz 
            + l1 *l1 *(4*ma + mc))*std::cos(2*s.Q[1]) 
            + 0.125*(4*Iayy - 4*Iazz + 4*Idfyy - 4*Idfzz 
            - l2*l2*ma - 4*l3*l3*mc)*std::cos(2*s.Q[2]) + l1*(l2*ma + l3*mc)*std::cos(s.Q[1])*std::sin(s.Q[2]));
    _M(0,1) = 0;
    _M(0,2) = 0;
    _M(1,0) = 0;  
    _M(1,1) = 0.25*(4*(Ibexx + Icxx + l1*l1*ma) + l1*l1*mc);        
    _M(1,2) = -0.5*l1*(l2*ma + l3*mc)*std::sin(s.Q[1]-s.Q[2]);   
    _M(2,0) = 0;     
    _M(2,1) = -0.5*l1*(l2*ma + l3*mc)*std::sin(s.Q[1]-s.Q[2]);        
    _M(2,2) = 0.25*(4*Iaxx + 4*Idfxx + l2*l2*ma + 4*l3*l3*mc);

    Matrix3d _C;
    _C(0,0) = 0.125*(-2*std::sin(s.Q[1])*((4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz
                + 4*l1*l1*ma + l1*l1*mc)*std::cos(s.Q[1]) + 2*l1*(l2*ma + l3*mc)*std::sin(s.Q[2]))*s.Qd[1]
                + 2*std::cos(s.Q[2])*(2*l1*(l2*ma+l3*mc)*std::cos(s.Q[1])
                + (-4*Iayy + 4*Iazz - 4*Idfyy + 4*Idfzz + l2*l2*ma
                + 4*l3*l3*mc)*std::sin(s.Q[2]))*s.Qd[2]);        
    _C(0,1) = -0.125*((4*Ibeyy - 4*Ibezz + 4*Icyy - 4*Iczz + l1*l1*(4*ma
                + mc))*std::sin(2*s.Q[1]) + 4*l1*(l2*ma + l3*mc)*std::sin(s.Q[1])*std::sin(s.Q[2]))*s.Qd[0];        
    _C(0,2) = -0.125*(-4*l1*(l2*ma + l3*mc)*std::cos(s.Q[1])*std::cos(s.Q[2]) -(-4*Iayy
                + 4*Iazz - 4*Idfyy + 4*Idfzz + l2*l2*ma + 4*l3*l3*mc)*std::sin(2*s.Q[2]))*s.Qd[0];                
    _C(1,0) = -_C(0,1);
    _C(1,1) = 0;
    _C(1,2) = 0.5*l1*(l2*ma+l3*mc)*std::cos(s.Q[1]-s.Q[2])*s.Qd[2];
    _C(2,0) = -_C(0,2);
    _C(2,1) = 0.5*l1*(l2*ma+l3*mc)*std::cos(s.Q[1]-s.Q[2])*s.Qd[1];
    _C(2,2) = 0;

    Vector3d _G;
    _G(0)   = 0;
    _G(1)   = 0.5*g*(2*l1*ma + 2*l5*mbe + l1*mc)*std::cos(s.Q[1]);
    _G(2)   = 0.5*g*(l2*ma + 2*l3*mc - 2*l6*mdf)*std::sin(s.Q[2]);

    Vector3d col = collision_torques(s.Q, s.Qd);
    Vector3d Qdd = _M.householderQr().solve(s.Tau + col - _C*s.Qd - _G);
    integrate_state(s, Qdd, dt);
}

}
}
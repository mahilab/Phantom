using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Phantom
{
    const float l2 = 0.209550f;
    const float l3 = 0.031750f;
    const float l4 = 0.169545f;


    public static Vector3 ForwardKinematics(float q1, float q2, float q3) {
        Vector3 P = new Vector3();
        P.x = Mathf.Cos(q1*Mathf.Deg2Rad)*(l2*Mathf.Cos(q2*Mathf.Deg2Rad) + l4*Mathf.Sin(q3*Mathf.Deg2Rad));
        P.y = Mathf.Sin(q1*Mathf.Deg2Rad)*(l2*Mathf.Cos(q2*Mathf.Deg2Rad) + l4*Mathf.Sin(q3*Mathf.Deg2Rad));
        P.z = l2*Mathf.Sin(q2*Mathf.Deg2Rad) - l4*Mathf.Cos(q3*Mathf.Deg2Rad);
        return P;
    }

    // ux = -sy
    // uy = sx
    // uz = -sz

    /// Transforms point in Phantom {0} to Unity {0}
    public static Vector3 ToUnity(Vector3 p) {
        return new Vector3(-p.y, p.x, -p.z);
    }

    public static Vector3 FromUnity(Vector3 p) {
        return new Vector3(p.y, -p.x, -p.z);
    }
}


using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhantomCableCapstan : MonoBehaviour
{
    [Header("Parameters")]
    public float capstanRadius = 0.0889f;
    public float cableRadius = 0.0005f;
    public float hTotal = 0.010033f;
    public float angleStart = 0;
    public float angleEnd   = -180;
    public float angleQa    = -90;
    public float angleQb;

    [Header("References")]
    public PhantomModel model;
    public LineRenderer segment1;
    public LineRenderer segment2;
    public LineRenderer segment3;

    [Header("Computed")]
    public float h1;
    public float h2;
    public float h3;
    public float h4;

    public void UpdateGeometry(float qa) {


        int k = Mathf.Max( (int)((Mathf.Abs((angleQa - qa) - angleStart)) / 5 ), 4 );
        segment1.positionCount = k;
        for (int i = 0; i < k; ++i) {
            float t = (float)i / (k-1);
            float a = Mathf.Lerp(angleStart, angleQa - qa, t) * Mathf.Deg2Rad;
            float h = Mathf.Lerp(0,-h1,t);
            Vector3 pos = new Vector3();
            pos.x = (capstanRadius + cableRadius) * Mathf.Cos(a);
            pos.y = (capstanRadius + cableRadius) * Mathf.Sin(a);
            pos.z = h;
            segment1.SetPosition(i, pos);            
        }

        k = Mathf.Max( (int)((Mathf.Abs(angleEnd - (angleQa - qa))) / 5 ), 4 );
        segment2.positionCount = k;
        for (int i = 0; i < k; ++i) {
            float t = (float)i / (k-1);
            float a = Mathf.Lerp(angleQa - qa, angleEnd, t) * Mathf.Deg2Rad;
            float h = Mathf.Lerp(-h2,-hTotal,t);
            Vector3 pos = new Vector3();
            pos.x = (capstanRadius + cableRadius) * Mathf.Cos(a);
            pos.y = (capstanRadius + cableRadius) * Mathf.Sin(a);
            pos.z = h;
            segment2.SetPosition(i, pos);  
        }
    }

    public void UpdateGeometry(float qa, float qb) {           

        int k = Mathf.Max( (int)((Mathf.Abs((angleQa - qa) - angleStart)) / 5 ), 4 );
        segment1.positionCount = k;
        for (int i = 0; i < k; ++i) {
            float t = (float)i / (k-1);
            float a = Mathf.Lerp(angleStart, angleQa - qa, t) * Mathf.Deg2Rad;
            float h = Mathf.Lerp(0,-h1,t);
            Vector3 pos = new Vector3();
            pos.x = (capstanRadius + cableRadius) * Mathf.Cos(a);
            pos.y = (capstanRadius + cableRadius) * Mathf.Sin(a);
            pos.z = h;
            segment1.SetPosition(i, pos);            
        }

        k = Mathf.Max( (int)((Mathf.Abs((angleQb - qb) - (angleQa - qa))) / 5 ), 4 );
        segment2.positionCount = k;
        for (int i = 0; i < k; ++i) {
            float t = (float)i / (k-1);
            float a = Mathf.Lerp(angleQa - qa, angleQb - qb, t) * Mathf.Deg2Rad;
            float h = Mathf.Lerp(-h2,-h3,t);
            Vector3 pos = new Vector3();
            pos.x = (capstanRadius + cableRadius) * Mathf.Cos(a);
            pos.y = (capstanRadius + cableRadius) * Mathf.Sin(a);
            pos.z = h;
            segment2.SetPosition(i, pos);  
        }

        k = Mathf.Max( (int)((Mathf.Abs(angleEnd - (angleQb - qb))) / 5 ), 4 );
        segment3.positionCount = k;
        for (int i = 0; i < k; ++i) {
            float t = (float)i / (k-1);
            float a = Mathf.Lerp(angleQb - qb, angleEnd, t) * Mathf.Deg2Rad;
            float h = Mathf.Lerp(-h4,-hTotal,t);
            Vector3 pos = new Vector3();
            pos.x = (capstanRadius + cableRadius) * Mathf.Cos(a);
            pos.y = (capstanRadius + cableRadius) * Mathf.Sin(a);
            pos.z = h;
            segment3.SetPosition(i, pos);  
        }

    }
}

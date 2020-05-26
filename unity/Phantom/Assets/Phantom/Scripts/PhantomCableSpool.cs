using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhantomCableSpool : MonoBehaviour
{
    [Header("Parameters")]
    public float revs  = 3;
    public float pitch = 396850;
    public float spoolRadius = 0.006731f;
    public float cableRadius = 0.0005f;
    public float direction = 1;
    public int k = 100;

    [Header("References")]
    LineRenderer line;

    [Header("Computed")]
    public float hTotal;

    void Awake() {
        line = GetComponent<LineRenderer>();
        UpdateGeometry();
    }

    void UpdateGeometry() {
        line.positionCount = k;
        hTotal = 360 * revs / pitch;
        for (int i = 0; i < k; ++i)
        {
            float t = (float)i / (k-1);
            float a = Mathf.Lerp(0, 360*  revs, t) * Mathf.Deg2Rad;
            float h = Mathf.Lerp(0, direction * hTotal, t);
            Vector3 pos = new Vector3();
            pos.x = (spoolRadius + cableRadius) * Mathf.Cos(a);
            pos.y = (spoolRadius + cableRadius) * Mathf.Sin(a);
            pos.z = h;
            line.SetPosition(i, pos);   
        }
    }

}

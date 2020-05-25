using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhantomModel : MonoBehaviour
{

    [Header("Options")]
    public bool showTransforms = false;
    public bool showWorkspace  = false;

    [Header("Joint Angles")]
    public float[] Q = new float[3];

    [Header("EE Position")]
    public double[] ee_pos = new double[3];
    public GameObject ee_sphere;

    [Header("Transmission Ratios")]
    public float[] eta = {13.3f,11.2f,11.2f};

    [Header("References")]
    public Transform[] frames = new Transform[3];
    public Transform frame2p;
    public Transform frame3p;
    public Transform[] spools = new Transform[3];

    public GameObject workspace;
    public TransformRenderer[] transformRenderers;

    public PhantomCableCapstan cableCapstan1;
    public PhantomCableCapstan cableCapstan2;
    public PhantomCableSpool cableSpool1;
    public PhantomCableSpool cableSpool2;
    public PhantomCableSpool cableSpool3;

    Vector3 cableSpool1_init;
    Vector3 cableSpool2_init;
    Vector3 cableSpool3_init;

    void Awake() {
        cableSpool1_init = cableSpool1.transform.localPosition;
        cableSpool2_init = cableSpool2.transform.localPosition;
        cableSpool3_init = cableSpool3.transform.localPosition;
    }

    // Update is called once per frame
    void LateUpdate()
    {
        workspace.SetActive(showWorkspace);
        foreach (var tr in transformRenderers)
            tr.enabled = showTransforms; 

        UpdateAngles();
        UpdateCable1();
        UpdateCable2();
    }

    void UpdateAngles() {
        Vector3 angles;
        for (int i = 0; i < 3; ++i) {
            angles = frames[i].transform.localEulerAngles;
            angles.z = Q[i];
            frames[i].transform.localEulerAngles = angles;
            angles = spools[i].transform.localEulerAngles;
            angles.z = -Q[i] * eta[i];
            spools[i].transform.localEulerAngles = angles;
        }

        angles = frame2p.transform.localEulerAngles;
        angles.z = Q[2] - Q[1]; 
        frame2p.transform.localEulerAngles = angles;

        angles = frame3p.transform.localEulerAngles;
        angles.z = Q[1] - Q[2];
        frame3p.transform.localEulerAngles = angles;
    }

    void UpdateCable1() {
        var temp = cableSpool1_init;
        temp.z -= (Q[0] * eta[0] / cableSpool1.pitch);
        cableSpool1.transform.localPosition = temp;

        cableCapstan1.h1 = cableSpool1.transform.position.y - cableCapstan1.transform.position.y;
        cableCapstan1.h2 = cableCapstan1.h1 + cableSpool1.hTotal;
        cableCapstan1.UpdateGeometry(Q[0]);

    }

    void UpdateCable2() {
        var temp = cableSpool2_init;
        temp.z += (Q[1] * eta[1] / cableSpool2.pitch);
        cableSpool2.transform.localPosition = temp;
        temp = cableSpool3_init;
        temp.z += (Q[2] * eta[2] / cableSpool3.pitch);
        cableSpool3.transform.localPosition = temp;
        cableCapstan2.h1 = frames[0].InverseTransformPoint(cableCapstan2.transform.position).x - frames[0].InverseTransformPoint(cableSpool3.transform.position).x;
        cableCapstan2.h2 = cableCapstan2.h1 + cableSpool3.hTotal;
        cableCapstan2.h3 = cableCapstan2.h1 + (frames[0].InverseTransformPoint(cableSpool3.transform.position).x - frames[0].InverseTransformPoint(cableSpool2.transform.position).x);
        cableCapstan2.h4 = cableCapstan2.h3 + cableSpool2.hTotal;
        cableCapstan2.UpdateGeometry(Q[2],Q[1]);
    }
}


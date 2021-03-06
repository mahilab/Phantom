﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhantomModel : MonoBehaviour
{

    [Header("Options")]
    public bool showTransforms = false;
    public bool showHighlights = false;

    [Header("Joint Angles")]
    public float[] Q = new float[3];

    [Header("Transmission Ratios")]
    public float[] eta = {13.3f,11.2f,11.2f};

    [Header("References")]
    public Transform[] frames = new Transform[3];
    public Transform frame2p;
    public Transform frame3p;
    public Transform[] spools = new Transform[3];

    public PhantomCableCapstan cableCapstan1;
    public PhantomCableCapstan cableCapstan2;
    public PhantomCableSpool cableSpool1;
    public PhantomCableSpool cableSpool2;
    public PhantomCableSpool cableSpool3;

    TransformRenderer[] transformRenderers;
    HighlightGroup[] highlightGroups;

    Vector3 cableSpool1_init;
    Vector3 cableSpool2_init;
    Vector3 cableSpool3_init;

    const float l1 = 0.209550f;
    const float l2 = 0.169545f;
    const float l3 = 0.031750f;

    public static Vector3 ForwardKinematics(float q1, float q2, float q3) {
        Vector3 P = new Vector3();
        P.x = Mathf.Cos(q1*Mathf.Deg2Rad)*(l1*Mathf.Cos(q2*Mathf.Deg2Rad) + l2*Mathf.Sin(q3*Mathf.Deg2Rad));
        P.y = Mathf.Sin(q1*Mathf.Deg2Rad)*(l1*Mathf.Cos(q2*Mathf.Deg2Rad) + l2*Mathf.Sin(q3*Mathf.Deg2Rad));
        P.z = l1*Mathf.Sin(q2*Mathf.Deg2Rad) - l2*Mathf.Cos(q3*Mathf.Deg2Rad);
        return P;
    }

    /// Transforms point in Phantom {0} to Unity {0}
    public static Vector3 ToUnity(Vector3 p) {
        return new Vector3(-p.y, p.x, -p.z);
    }

    public static Vector3 FromUnity(Vector3 p) {
        return new Vector3(p.y, -p.x, -p.z);
    }

    void Awake() {

        transformRenderers = GetComponentsInChildren<TransformRenderer>();
        highlightGroups    = GetComponentsInChildren<HighlightGroup>();

        cableSpool1_init = cableSpool1.transform.localPosition;
        cableSpool2_init = cableSpool2.transform.localPosition;
        cableSpool3_init = cableSpool3.transform.localPosition;
    }

    void Update() {
        if (Input.GetKeyDown(KeyCode.H))
            showHighlights = !showHighlights;
        if (Input.GetKeyDown(KeyCode.T))
            showTransforms = !showTransforms;
    }

    // Update is called once per frame
    void LateUpdate()
    {
        foreach (var tr in transformRenderers)
            tr.enabled = showTransforms; 
        foreach (var hg in highlightGroups)
            hg.show = showHighlights;

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


using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhantomModel : MonoBehaviour
{

    [Header("Joint Angles")]
    public float[] theta = new float[3];

    [Header("Transmission Ratios")]
    public float[] eta = {13.3f,11.2f,11.2f};

    [Header("Transforms")]
    public Transform[] frames = new Transform[3];
    public Transform frameA;
    public Transform frameC;

    public Transform[] spools = new Transform[3];

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 angles;
        for (int i = 0; i < 3; ++i) {
            angles = frames[i].transform.localEulerAngles;
            angles.z = theta[i];
            frames[i].transform.localEulerAngles = angles;

            angles = spools[i].transform.localEulerAngles;
            angles.z = -theta[i] * eta[i];
            spools[i].transform.localEulerAngles = angles;
        }

        angles = frameA.transform.localEulerAngles;
        angles.z = theta[2] - theta[1];
        frameA.transform.localEulerAngles = angles;

        angles = frameC.transform.localEulerAngles;
        angles.z = theta[1] - theta[2];
        frameC.transform.localEulerAngles = angles;
    }
}

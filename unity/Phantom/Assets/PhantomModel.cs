using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

public class PhantomModel : MonoBehaviour
{

    public bool simulate = true;

    [Header("Joint Angles")]
    public float[] theta = new float[3];

    [Header("Transmission Ratios")]
    public float[] eta = {13.3f,11.2f,11.2f};

    [Header("Transforms")]
    public Transform[] frames = new Transform[3];
    public Transform frameA;
    public Transform frameC;

    public Transform[] spools = new Transform[3];

    double[] radians = new double[3];
    public double[] tau = new double[3];

    // Start is called before the first frame update
    void Start()
    {
        Dll.start();
    }

    void OnApplicationQuit() {
        Dll.stop();
    }

    // Update is called once per frame
    void Update()
    {

        tau[0] = 0.01*Input.GetAxis("Horizontal");
        tau[1] = 0.1*Input.GetAxis("Vertical");
        tau[2] = 0.1*Input.GetAxis("Vertical2");

        Dll.set_torques(tau);
        Dll.get_positions(radians);


        Vector3 angles;
        for (int i = 0; i < 3; ++i) {

            if (simulate)
                theta[i] = (float)radians[i] * Mathf.Rad2Deg;

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

        if (Input.GetKeyDown(KeyCode.R))
        {
            Dll.stop();
            Dll.start();
        }
    }

    /// Dll Imports
    public class Dll {
        [DllImport("phantom")] 
        public static extern void start();
        [DllImport("phantom")] 
        public static extern void stop();
        [DllImport("phantom")]
        public static extern void get_positions(double[] Q);   
        [DllImport("phantom")]
        public static extern void set_torques(double[] Tau);
    }
}


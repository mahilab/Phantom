using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;

public class PhantomSimulation : MonoBehaviour
{
    public PhantomModel model;

    double[] radians = new double[3];
    double[] tau = new double[3];

    [Header("EE Position")]
    public double[] ee_pos = new double[3];
    public GameObject ee_sphere;

    [Header("IK Angles")]
    public double[] theta_d = new double[3];

    // Start is called before the first frame update
    void Start()
    {
        Dll.start();
    }

    void OnApplicationQuit() {
        Dll.stop();
    }

    void Update() {

        tau[0] = 0.05*Input.GetAxis("Horizontal");
        tau[1] = 0.1*Input.GetAxis("Vertical");
        tau[2] = 0.1*Input.GetAxis("Vertical2");

        Dll.set_torques(tau);
        Dll.get_positions(radians);

        for (int i = 0; i < 3; ++i)
            model.Q[i] = Mathf.Rad2Deg * (float)radians[i];

        if (Input.GetKeyDown(KeyCode.R))
            Restart();     

        if (Input.GetKeyDown(KeyCode.T)) {
            var s = Dll.open_tuner();   
            print(s);
        }

        Dll.get_fk(ee_pos);
        ee_sphere.transform.localPosition = new Vector3((float)-ee_pos[1],(float)ee_pos[0],(float)-ee_pos[2]);

        double[] q_double = {(double)model.Q[0]*Mathf.Deg2Rad,(double)model.Q[1]*Mathf.Deg2Rad,(double)model.Q[2]*Mathf.Deg2Rad};
        Dll.get_ik(ee_pos,theta_d,q_double);
        theta_d[0] = theta_d[0]*Mathf.Rad2Deg;
        theta_d[1] = theta_d[1]*Mathf.Rad2Deg;
        theta_d[2] = theta_d[2]*Mathf.Rad2Deg;
    }

    void Restart() {
        Dll.stop();
        Dll.start();
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
        [DllImport("phantom")]
        public static extern bool open_tuner();
        [DllImport("phantom")]
        public static extern void get_fk(double[] EE_pos);
        [DllImport("phantom")]
        public static extern void get_ik(double[] EE_pos, double[] Theta_d, double[] Curr_angles);
    }

}

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

    [Header("Desired EE Position")]
    public double[] ee_d = new double[3];
    public GameObject ee_d_sphere;

    [Header("IK Angles")]
    public double[] theta_d = new double[3];

    private float nextTime = 1;
    private float deltaTime = 0.5f;
    private int frames;
    public float fps;

    // Start is called before the first frame update
    void Start()
    {
        Dll.start();
    }

    void OnApplicationQuit() {
        Dll.stop();
    }

    void Update() {

        if (Time.time > nextTime) {
            fps = frames / deltaTime;
            nextTime += deltaTime;
            frames = 0;
        }
        frames++;

        tau[0] = 0.05*Input.GetAxis("Horizontal");
        tau[1] = 0.1*Input.GetAxis("Vertical");
        tau[2] = 0.1*Input.GetAxis("Vertical2");

        Dll.get_positions(radians);

        for (int i = 0; i < 3; ++i)
            model.Q[i] = Mathf.Rad2Deg * (float)radians[i];
        
        Dll.set_torques(tau);

        if (Input.GetKeyDown(KeyCode.R))
            Restart();     

        if (Input.GetKeyDown(KeyCode.F1)) {
            Dll.open_tuner();   
        }

        // double[] q_double = {(double)model.Q[0]*Mathf.Deg2Rad,(double)model.Q[1]*Mathf.Deg2Rad,(double)model.Q[2]*Mathf.Deg2Rad};

        // Dll.get_fk(ee_pos);
        // ee_d_sphere.transform.localPosition = new Vector3((float)-ee_pos[1],(float)ee_pos[0],(float)-ee_pos[2]);

        // Dll.get_ee_d(ee_d);
        // ee_d_sphere.transform.localPosition = new Vector3((float)-ee_d[1],(float)ee_d[0],(float)-ee_d[2]);
        // double[] ee_check_pos = {0,0,0};
        // double[] compare_pos  = {0.0,0.0,0.0};
        // Dll.get_ik(ee_d, ee_check_pos, q_double);
        // if (arrays_eq(ee_check_pos,compare_pos,3,1e-8)){
        //     ee_d_sphere.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
        // }
        // else{
        //     ee_d_sphere.GetComponent<Renderer>().material.SetColor("_Color", Color.green);
        // } 
        
        // Dll.get_ik(ee_pos,theta_d,q_double);
        // theta_d[0] = theta_d[0]*Mathf.Rad2Deg;
        // theta_d[1] = theta_d[1]*Mathf.Rad2Deg;
        // theta_d[2] = theta_d[2]*Mathf.Rad2Deg;
    }

    bool arrays_eq(double[] arr1, double[] arr2, int size, double thresh){
        float diff = 0;
        for (int i = 0; i < size; i++)
        {
            diff += Mathf.Sqrt((float)((arr1[i] - arr2[i])*(arr1[i] - arr2[i])));
        }
        return diff < thresh;
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
        // [DllImport("phantom")]
        // public static extern void get_fk(double[] EE_pos);
        // [DllImport("phantom")]
        // public static extern void get_ik(double[] EE_pos, double[] Theta_d, double[] Curr_angles);
        // [DllImport("phantom")]
        // public static extern void get_ee_d(double[] EE_d);
    }

}

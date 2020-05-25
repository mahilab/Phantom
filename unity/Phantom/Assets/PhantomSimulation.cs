using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

public class PhantomSimulation : MonoBehaviour
{
    public PhantomModel model;

    double[] radians = new double[3];
    double[] tau = new double[3];

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
    }

}

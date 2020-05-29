using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;

public class PhantomSimulation : MonoBehaviour
{
    public PhantomModel model;

    double[] radians = new double[3];

    public GameObject target;

    private float nextTime = 1;
    private float deltaTime = 0.5f;
    private int frames;
    public float fps;

    GameObject[] targets;

    // Start is called before the first frame update
    void Start()
    {
        PhantomPlugin.start();
    }

    void OnApplicationQuit()
    {
        PhantomPlugin.stop();
    }

    void Update()
    {

        if (Time.time > nextTime)
        {
            fps = frames / deltaTime;
            nextTime += deltaTime;
            frames = 0;
        }
        frames++;

        // tau[0] = 0.05 * Input.GetAxis("Horizontal");
        // tau[1] = 0.1 * Input.GetAxis("Vertical");
        // tau[2] = 0.1 * Input.GetAxis("Vertical2");

        PhantomPlugin.get_positions(radians);

        Vector3 target_pos = PhantomModel.FromUnity(target.transform.localPosition);
        PhantomPlugin.set_target(target_pos.x, target_pos.y, target_pos.z);

        for (int i = 0; i < 3; ++i)
            model.Q[i] = Mathf.Rad2Deg * (float)radians[i];


        if (Input.GetKeyDown(KeyCode.R))
            Restart();

        if (Input.GetKeyDown(KeyCode.G))
        {
            PhantomPlugin.open_tuner();   
        }
    }

    void Restart()
    {
        PhantomPlugin.stop();
        PhantomPlugin.start();
    }

    public static class PhantomPlugin {
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
        public static extern void set_target(double x, double y, double z);
    }

}

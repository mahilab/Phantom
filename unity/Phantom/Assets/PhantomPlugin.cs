using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

public class PhantomPlugin : MonoBehaviour
{

    public PhantomModel model;

    double[] radians = new double[3];

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.G))        
            Plugin.open_gui();  

        Plugin.get_positions(radians);
        for (int i = 0; i < 3; ++i)
            model.Q[i] = Mathf.Rad2Deg * (float)radians[i]; 
    }

    public static class Plugin {
        [DllImport("phantom_plugin")] 
        public static extern bool open_gui();
        [DllImport("phantom_plugin")] 
        public static extern bool get_positions(double[] Q);  
    }

}

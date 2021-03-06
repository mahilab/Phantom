﻿using UnityEngine;
using System.Collections;
 
[AddComponentMenu("Camera-Control/Mouse Orbit")]
public class MouseOrbitImproved : MonoBehaviour {
 
    public Transform target;
    public float distance = 5.0f;
    public float xSpeed = 120.0f;
    public float ySpeed = 120.0f;
 
    public float yMinLimit = -20f;
    public float yMaxLimit = 80f;
 
    public float distanceMin = .5f;
    public float distanceMax = 15f;

    public int button = 0;
 
    public float x = 0.0f;
    public float y = 0.0f;

    Quaternion rotation;
 
    // Use this for initialization
    void Start () 
    {
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;
        Orbit();
        Scroll();
    }
 
    void LateUpdate () 
    {
        if (target) {
            if (Input.GetMouseButton(button))
                Orbit();
            Scroll();
        }
    }

    void Orbit() {
        x += Input.GetAxis("Mouse X") * xSpeed * distance * 0.02f;
        y -= Input.GetAxis("Mouse Y") * ySpeed * 0.02f;

        y = ClampAngle(y, yMinLimit, yMaxLimit);

        rotation = Quaternion.Euler(y, x, 0);
    }

    void Scroll() {
        distance = Mathf.Clamp(distance - Input.GetAxis("Mouse ScrollWheel"), distanceMin, distanceMax);
        Vector3 negDistance = new Vector3(0.0f, 0.0f, -distance);
        Vector3 position = rotation * negDistance + target.position;
        transform.rotation = rotation;
        transform.position = position;
    }
 
    public static float ClampAngle(float angle, float min, float max)
    {
        if (angle < -360F)
            angle += 360F;
        if (angle > 360F)
            angle -= 360F;
        return Mathf.Clamp(angle, min, max);
    }
}
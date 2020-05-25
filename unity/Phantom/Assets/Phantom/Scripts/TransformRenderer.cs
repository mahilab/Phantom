using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TransformRenderer : MonoBehaviour
{

    [Header("Directions")]
    public Vector3 xDir = Vector3.right;
    public Vector3 yDir = Vector3.up;
    public Vector3 zDir = Vector3.forward;
    [Space]
    public Vector3 offset = Vector3.zero;

    [Header("Widths")]
    public float scale   = 0.01f;
    public float width1  = 0.001f;
    public float width2  = 0.00025f;

    [Header("Colors")]
    public Color xColor1 = new Color(1,0,0,0);
    public Color xColor2 = Color.red;

    public Color yColor1 = new Color(0,1,0,0);
    public Color yColor2 = Color.green;

    public Color zColor1 = new Color(0,0,1,0);
    public Color zColor2 = Color.blue;


    GameObject root;
    LineRenderer xAxis;
    LineRenderer yAxis;
    LineRenderer zAxis;

    Material axisMaterial;

    // Start is called before the first frame update
    void Awake()
    {
        CreateMaterial();
        CreateRoot();
        CreateAxis(ref xAxis, "X-Axis", xDir, xColor1, xColor2);
        CreateAxis(ref yAxis, "Y-Axis", yDir, yColor1, yColor2);
        CreateAxis(ref zAxis, "Z-Axis", zDir, zColor1, zColor2);
    }

    // Update is called once per frame
    void LateUpdate()
    {
        UpdateAxis(ref xAxis, xDir, xColor1, xColor2);
        UpdateAxis(ref yAxis, yDir, yColor1, yColor2);
        UpdateAxis(ref zAxis, zDir, zColor1, zColor2);
    }

    void CreateMaterial() {
        axisMaterial = new Material(Shader.Find("Custom/Highlight"));
        axisMaterial.SetFloat("_Darken", 1.0f);
        axisMaterial.SetFloat("_SeeThru", 1.0f);
    }

    void CreateRoot() {
        root = new GameObject("_TransformRenderer");
        root.transform.parent = this.transform;
        root.transform.localPosition = Vector3.zero;
        root.transform.localEulerAngles = Vector3.zero;
        root.transform.localScale = Vector3.one;
    }

    void CreateAxis(ref LineRenderer axis, string name, Vector3 direction, Color color1, Color color2) {
        GameObject axisGo = new GameObject(name);
        axisGo.transform.parent = root.transform;
        axisGo.transform.localPosition = Vector3.zero;
        axisGo.transform.localEulerAngles = Vector3.zero;
        axisGo.transform.localScale = Vector3.one;
        axis = axisGo.AddComponent<LineRenderer>();
        axis.positionCount = 2;
        axis.useWorldSpace = false;
        axis.material = axisMaterial;
        UpdateAxis(ref axis, direction, color1, color2);
    }

    void UpdateAxis(ref LineRenderer axis, Vector3 direction, Color color1, Color color2) {
        axis.SetPosition(0, offset);
        axis.SetPosition(1, offset + direction * scale);
        axis.startWidth = width1;
        axis.endWidth   = width2;
        axis.startColor = color1;
        axis.endColor   = color2;
    }

    void OnEnable() {
        root.SetActive(true);
    }

    void OnDisable() {
        root.SetActive(false);
    }

    void OnDestroy() {
        Destroy(axisMaterial);
        Destroy(root);
    }
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HighlightGroup : MonoBehaviour
{

    public Material material;

    public bool show;

    GameObject group;

    void Awake() {
        MeshRenderer[] mrs = GetComponentsInChildren<MeshRenderer>();
        group = new GameObject();
        group.transform.parent = this.transform;
        group.transform.localPosition = Vector3.zero;
        group.transform.localScale    = Vector3.one;
        group.transform.localEulerAngles = Vector3.zero;
        group.name = "Highlights";
        foreach (var mr in mrs) {
            var hl = new GameObject();
            hl.name = mr.name;
            hl.transform.parent = group.transform;
            hl.transform.localPosition = Vector3.zero;
            hl.transform.localScale    = Vector3.one;
            hl.transform.localEulerAngles = Vector3.zero;
            var mf = hl.AddComponent<MeshFilter>();
            mf.mesh = mr.GetComponent<MeshFilter>().mesh;
            var mrhl = hl.AddComponent<MeshRenderer>();
            mrhl.material = material;
            mrhl.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            mrhl.receiveShadows = false;
        }
    }
    
    // Update is called once per frame
    void Update() {
        group.SetActive(show);
    }
}

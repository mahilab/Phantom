using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointCloud : MonoBehaviour
{
    // Start is called before the first frame update
    void Awake()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.SetIndices(mesh.GetIndices(0), MeshTopology.Points, 0);
    }


}

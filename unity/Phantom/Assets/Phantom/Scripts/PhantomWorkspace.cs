using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Linq;

public class PhantomWorkspace : MonoBehaviour
{

    public enum Mode {
        None,
        Hull,
        Cloud
    };

    public Mode mode;

    [Header("References")]
    public PhantomModel model;
    public Mesh meshHull;
    public Mesh meshCloud;

    // Start is called before the first frame update
    void Awake()
    {
        if (meshHull == null) {
            meshHull = GenerateMeshHull();
            #if UNITY_EDITOR
            // UnityEditor.AssetDatabase.CreateAsset(meshHull, "Assets/Phantom/Models/WorkspaceHull.asset");
            // UnityEditor.AssetDatabase.SaveAssets();
            #endif
        }
        if (meshCloud == null) {
            meshCloud = GenerateMeshCloud();
            #if UNITY_EDITOR
            // UnityEditor.AssetDatabase.CreateAsset(meshCloud, "Assets/Phantom/Models/WorkspaceCloud.asset");
            // UnityEditor.AssetDatabase.SaveAssets();
            #endif
        }
        SetMesh();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.W))
            ToggleMesh();
    }

    void ToggleMesh() {
        if (mode == Mode.None) 
            mode = Mode.Hull;        
        else if (mode == Mode.Hull) 
            mode = Mode.Cloud;
        else if (mode == Mode.Cloud)
            mode = Mode.None;
        SetMesh();        
    }

    void SetMesh() {
        var mf = GetComponent<MeshFilter>();
        if (mode == Mode.None)
            mf.mesh = null;
        else if (mode == Mode.Hull)
            mf.mesh = meshHull;
        else if (mode == Mode.Cloud)
            mf.mesh = meshCloud;
    }

    IEnumerator TraceWorkspace() {
        float[] durations = {3,1,1,1};
        float[] q2s       = {120,-85,35,120};
        float[] q3s       = {175,-30,-30,55};

        float elapsedTotal = 0;
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            float elapsed = 0;
            float duration = durations[i];
            while (elapsed < duration) {
                float t = elapsed / duration;
                float q2 = Mathf.Lerp(q2s[i],q2s[j],t);
                float q3 = Mathf.Lerp(q3s[i],q3s[j],t);

                // model.theta[0] = Mathf.Lerp(90,-90,elapsedTotal/6.0f);
                model.Q[1] = q2;
                model.Q[2] = q3;
                elapsed += Time.deltaTime;
                elapsedTotal += Time.deltaTime;
                yield return null;
            }
            model.Q[1] = q2s[j];
            model.Q[2] = q3s[j];
        }
    }

    Mesh GenerateMeshCloud() {
        Mesh mesh = new Mesh();
        mesh.name = "phantom_workspace";
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        int n = 200;
        List<Vector3> verts = new List<Vector3>(n*n*n);
        for (int i1 = 0; i1 < n; ++i1) {
            float t1 = (float)i1 / (float)(n - 1);
            for (int i2 = 0; i2 < n; ++i2) {
                float t2 = (float)i2 / (float)(n - 1);
                for (int i3 = 0; i3 < n; ++i3) {
                    float t3 = (float)i3 / (float)(n - 1);
                    float q1 = Mathf.Lerp(-90,90,t1)  + Random.Range(-1,1);
                    float q2 = Mathf.Lerp(-85,120,t2) + Random.Range(-1,1);
                    float q3 = Mathf.Lerp(-30,175,t3) + Random.Range(-1,1);
                    if ((q3-q2) <= 55 && (q2-q3) <= 65) {

                        verts.Add(PhantomModel.ToUnity(PhantomModel.ForwardKinematics(q1,q2,q3)));
                    }
                }
            }
        }
        mesh.SetVertices(verts);
        mesh.SetIndices(Enumerable.Range(0, verts.Count).ToArray(), MeshTopology.Points, 0);
        return mesh;
    }


    Mesh GenerateMeshHull() {
        Mesh mesh = new Mesh();
        mesh.name = "phantom_workspace";
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        float[] q2s = {120,-85,35,120};
        float[] q3s = {175,-30,-30,55};
        float[] ns  = {1000,200,80,200};

        int n = 200;
        List<Vector3> verts = new List<Vector3>(n*n*n);

        for (int i1 = 0; i1 < 1000; ++i1) {
            float t1 = (float)i1 / (float)(1000 - 1);
            int mult = 1;
            if (t1 == 0 || t1 == 1)
                mult = 50;
            for (int k = 0; k < 4; ++k) {
                int j = (k + 1) % 4;
                for (int i3 = 0; i3 < mult*ns[k]; ++i3) {
                    float t3 = (float)i3 / (float)(mult*ns[k] - 1);
                    float q1 = Mathf.Lerp(-90,90,t1) + Random.Range(-0.1f,0.1f);
                    float q2 = Mathf.Lerp(q2s[k],q2s[j],t3);
                    float q3 = Mathf.Lerp(q3s[k],q3s[j],t3);
                    verts.Add(PhantomModel.ToUnity(PhantomModel.ForwardKinematics(q1,q2,q3)));
                }
            }
        }


        n = 100;
        for (int i1 = 0; i1 < n; ++i1) {
            float t1 = (float)i1 / (float)(n - 1);
            for (int i2 = 0; i2 < n; ++i2) {
                float t2 = (float)i2 / (float)(n - 1);
                for (int i3 = 0; i3 < n; ++i3) {
                    float t3 = (float)i3 / (float)(n - 1);
                    float q1 = Mathf.Lerp(-90,90,t1)  + Random.Range(-1,1);
                    float q2 = Mathf.Lerp(-85,120,t2) + Random.Range(-1,1);
                    float q3 = Mathf.Lerp(-30,175,t3) + Random.Range(-1,1);
                    if ((q3-q2) <= 55 && (q2-q3) <= 65) {

                        verts.Add(PhantomModel.ToUnity(PhantomModel.ForwardKinematics(q1,q2,q3)));
                    }
                }
            }
        }

        mesh.SetVertices(verts);
        mesh.SetIndices(Enumerable.Range(0, verts.Count).ToArray(), MeshTopology.Points, 0);
        return mesh;
    }


}

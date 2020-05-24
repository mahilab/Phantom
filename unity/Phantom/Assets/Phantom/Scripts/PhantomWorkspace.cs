using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Linq;

public class PhantomWorkspace : MonoBehaviour
{

    public PhantomModel model;

    // Start is called before the first frame update
    void Start()
    {
        var mesh = new Mesh();
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

                        verts.Add(Phantom.ToUnity(Phantom.ForwardKinematics(q1,q2,q3)));
                    }
                }
            }
        }
        mesh.SetVertices(verts);
        mesh.SetIndices(
            Enumerable.Range(0, verts.Count).ToArray(),
            MeshTopology.Points, 0
        );


        GetComponent<MeshFilter>().mesh = mesh;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
            StartCoroutine(TraceWorkspace());
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
                model.theta[1] = q2;
                model.theta[2] = q3;
                elapsed += Time.deltaTime;
                elapsedTotal += Time.deltaTime;
                yield return null;
            }
            model.theta[1] = q2s[j];
            model.theta[2] = q3s[j];
        }
    }
}

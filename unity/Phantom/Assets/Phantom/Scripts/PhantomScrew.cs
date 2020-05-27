using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhantomScrew : MonoBehaviour
{

    public bool unscrewed = false;
    public bool canScrew  = true;

    bool screwing = false;

    void OnMouseDown() {
        if (!screwing && canScrew) {
            StartCoroutine(Screw());
        }
    }

    public IEnumerator Screw() {
        screwing = true;
        float translation = 0.01f;
        float rot = 1574 * 360 * translation;
        Vector3 startPos = transform.localPosition;
        Vector3 startRot = transform.localEulerAngles;
        Vector3 endPos   = startPos + new Vector3(0,unscrewed ? translation : -translation,0);
        Vector3 endRot   = startRot + new Vector3(0,0,unscrewed ? rot : -rot);
        float elapsed = 0;
        float duration = 2;
        while (elapsed < duration) {
            float t = elapsed / duration;
            t = Tween.Smoothstep(0,1,t);
            transform.localPosition = Vector3.Lerp(startPos, endPos, t);
            transform.localEulerAngles = Vector3.Lerp(startRot, endRot, t);
            elapsed += Time.deltaTime;
            yield return null;
        }
        transform.localPosition    = endPos;
        transform.localEulerAngles = endRot;
        unscrewed = !unscrewed;
        screwing = false;
    }
}

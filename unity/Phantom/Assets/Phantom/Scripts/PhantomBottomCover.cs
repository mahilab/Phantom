using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhantomBottomCover : MonoBehaviour
{

    public bool open = false;
    PhantomScrew[] screws;
    bool covering;

    // Start is called before the first frame update
    void Awake()
    {
        screws = GetComponentsInChildren<PhantomScrew>();
    }

    void OnMouseDown() {
        if (AllUnscrewed() && !open && !covering) {
            StartCoroutine(Uncover());
        }
        else if (open && !covering) {
            StartCoroutine(Cover());
        }
    }

    IEnumerator Uncover() {
        covering = true;
        foreach (var screw in screws) {
            screw.transform.parent = null;
            screw.canScrew = false;
        }
        float elapsed = 0;
        float duration = 1;
        Vector3 startPos = transform.localPosition;
        Vector3 endPos   = startPos + new Vector3(0,0,0.008f);
        while (elapsed < duration) {
            float t = elapsed / duration;
            t = Tween.Smoothstep(0,1,t);
            transform.localPosition = Vector3.Lerp(startPos, endPos, t);
            elapsed += Time.deltaTime;
            yield return null;
        }
        transform.localPosition = endPos;        
        foreach (var screw in screws) 
            screw.transform.parent = this.transform;
        elapsed = 0;
        startPos = endPos;
        endPos   = startPos - new Vector3(0.107f,0,0);
        while (elapsed < duration) {
            float t = elapsed / duration;
            t = Tween.Smoothstep(0,1,t);
            transform.localPosition = Vector3.Lerp(startPos, endPos, t);
            elapsed += Time.deltaTime;
            yield return null;
        }
        transform.localPosition = endPos;           
        
        covering = false;
        open = true;
    }

    IEnumerator Cover() {
        covering = true;
        float elapsed = 0;
        float duration = 1;
        Vector3 startPos = transform.localPosition;
        Vector3 endPos   = startPos + new Vector3(0.107f,0,0);
        while (elapsed < duration) {
            float t = elapsed / duration;
            t = Tween.Smoothstep(0,1,t);
            transform.localPosition = Vector3.Lerp(startPos, endPos, t);
            elapsed += Time.deltaTime;
            yield return null;
        }
        transform.localPosition = endPos;     
        foreach (var screw in screws) {
            screw.transform.parent = null;
        }
        startPos = endPos;
        endPos   = startPos - new Vector3(0,0,0.008f);
        elapsed = 0;
        while (elapsed < duration) {
            float t = elapsed / duration;
            t = Tween.Smoothstep(0,1,t);
            transform.localPosition = Vector3.Lerp(startPos, endPos, t);
            elapsed += Time.deltaTime;
            yield return null;
        }
        transform.localPosition = endPos; 
        foreach (var screw in screws) {
            screw.transform.parent = this.transform;
            screw.canScrew = true;
            screw.StartCoroutine(screw.Screw());
        }
        open = false;
        covering = false;
    }

    bool AllUnscrewed() {
        foreach (var screw in screws)
        {
            if (!screw.unscrewed)
                return false;
        }
        return true;
    }
}

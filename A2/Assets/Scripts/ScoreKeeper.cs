using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScoreKeeper : MonoBehaviour
{
    public GameObject formationLeader;
    public List<GameObject> followers = new List<GameObject>();

    public float costSoFar = 0f;
    public float timeToStop = 30f;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // aggregate devitions from line
        if (Time.time < timeToStop)
        {

            foreach (var follower in followers)
            {
                Vector3 relativePos = formationLeader.transform.position - follower.transform.position;

                float deviation = Vector3.Dot(relativePos, formationLeader.transform.forward);
                costSoFar += deviation * deviation * Time.deltaTime;
                Debug.Log("cost" + costSoFar);
            }
        }
    }
}

﻿using System.Collections;
//using Newtonsoft.Json; // Import JSON.NET from Unity Asset store (might not be needed 2022)


public class TrajectoryLogger : MonoBehaviour

    // Start is called before the first frame update
    void Start()

    // Update is called once per frame
    void Update()

    }
            // avoid index out of bounds
            if (current_index > myInfo.position_array.Length - 1)
    }
        // Save data structure 
        //Debug.Log("Application ending after " + Time.time + " seconds");

        if (RecordingOn && enabled)
            //Debug.Log(System.DateTime.Now.ToLongTimeString());
            //Debug.Log(System.DateTime.Now.ToShortTimeString());


            recordedInfo.WriteDataToFile();
        //Debug.Log("Reading json");
        //return JsonConvert.DeserializeObject<TrajectoryInfo>(jsonString);
        return JsonUtility.FromJson<TrajectoryInfo>(jsonString);
    }
        //Debug.Log("AssetPath:" + path);
        System.IO.File.WriteAllText(path, json_string);


#if UNITY_EDITOR


#endif
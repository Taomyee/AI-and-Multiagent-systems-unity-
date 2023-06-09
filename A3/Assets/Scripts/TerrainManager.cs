﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json; // Import JSON.NET from Unity Asset store (not needed in 2022 version)


public class TerrainManager : MonoBehaviour
{


    //public TestScriptNoObject testNoObject = new TestScriptNoObject();

    public string terrain_filename = "Text/terrain";
    public TerrainInfo myInfo;

    public GameObject flag;
    public int random_seed = 42;


    // Use this for initialization
    void Start()
    {
        
    }

    // Use this for initialization
    void Awake()
    {
        Random.seed = random_seed;
        var jsonTextFile = Resources.Load<TextAsset>(terrain_filename);

        // set to false for hard coding new terrain using TerrainInfo2 below
        bool loadFromFile = true;

        if (loadFromFile)
        {
            myInfo = TerrainInfo.CreateFromJSON(jsonTextFile.text);
        }
        else
        {
            myInfo.TerrainInfo2();
            //myInfo.file_name = "test88";
            string myString = myInfo.SaveToString();
            myInfo.WriteDataToFile(myString);
        }

        myInfo.CreateCubes();

        Instantiate(flag, myInfo.start_pos, Quaternion.identity);
        Instantiate(flag, myInfo.goal_pos, Quaternion.identity);



    }



    // Update is called once per frame
    void Update()
    {

    }
    
    //TODO: Temp fix for feature envy. Might need util class.
    public Vector3 CircularConfiguration(int i, int max_i, float factor_k)
    {
        float center_x = (myInfo.x_high + myInfo.x_low) / 2.0f;
        float center_z = (myInfo.z_high + myInfo.z_low) / 2.0f;
        Vector3 center = new Vector3(center_x, 1f, center_z);

        float alpha = i * 2.0f * Mathf.PI / max_i;
        float r = (myInfo.x_high - center_x) * factor_k;
        return center + new Vector3(Mathf.Sin(alpha), 0f, Mathf.Cos(alpha)) * r;
    }

    public Vector3 RandomConfiguration(int i, int max_i, float padding_factor)
    {
        float r1 = padding_factor + (1.0f - 2 * padding_factor) * Random.value; // gives random value in [1 - factor_k, factor_k] = [0.1, 0.9] if factor_k=0.9
        float r2 = padding_factor + (1.0f - 2 * padding_factor) * Random.value; 
        float x = r1 * myInfo.x_high + (1.0f - r1) * myInfo.x_low; 
        float z = r2 * myInfo.z_high + (1.0f - r2) * myInfo.z_low;
        return new Vector3(x, 0f, z);
    }

    public Vector3 GetCollisionFreePosNear(Vector3 startPos, float max_dist)
    {

        if (myInfo.is_traverable(startPos))
            return startPos;

        for (int k = 0; k <= 100; k++)
        {
            Vector3 delta_pos = new Vector3(Random.Range(0f, max_dist), 0f, Random.Range(0f, max_dist));
            if (myInfo.is_traverable(startPos + delta_pos))
                return startPos + delta_pos;
        }

        return Vector3.zero;
    }
}



[System.Serializable]
public class TerrainInfo
{
    public string file_name;
    public float x_low;
    public float x_high;
    public int x_N;
    public float z_low;
    public float z_high;
    public int z_N;
    public float[,] traversability;

    public Vector3 start_pos;
    public Vector3 goal_pos;


    public void TerrainInfo2()
    {
        file_name = "my_terrain";
        x_low = 50f;
        x_high = 250f;
        x_N = 45;
        z_low = 50f;
        z_high = 250f;
        z_N = 7;
        Debug.Log("Using hard coded info...");
        //traversability = new float[,] { { 1.1f, 2f }, { 3.3f, 4.4f } };
        traversability = new float[x_N, z_N]; // hardcoded now, needs to change
        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if ((i == 0 || i == x_N - 1) || (j == 0 || j == z_N - 1))
                {
                    traversability[i, j] = 1.0f;
                }
                else
                {
                    traversability[i, j] = 0.0f;
                }
            }
        }
    }

    public int get_i_index(float x)
    {
        int index = (int)Mathf.Floor(x_N * (x - x_low) / (x_high - x_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > x_N - 1)
        {
            index = x_N - 1;
        }
        return index;

    }
    public int get_j_index(float z) // get index of given coordinate
    {
        int index = (int)Mathf.Floor(z_N * (z - z_low) / (z_high - z_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > z_N - 1)
        {
            index = z_N - 1;
        }
        return index;
    }

    public bool is_traverable(Vector3 pos)
    {
        int i = get_i_index(pos.x);
        int j = get_j_index(pos.z);
        return traversability[i, j] < 0.1f;
    }

    public float get_x_pos(int i)
    {
        float step = (x_high - x_low) / x_N;
        return x_low + step / 2 + step * i;
    }

    public float get_z_pos(int j) // get position of given index
    {
        float step = (z_high - z_low) / z_N;
        return z_low + step / 2 + step * j;
    }

    public void CreateCubes()
    {
        float x_step = (x_high - x_low) / x_N;
        float z_step = (z_high - z_low) / z_N;
        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if (traversability[i, j] > 0.5f)
                {
                    GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cube.transform.position = new Vector3(get_x_pos(i), 0.0f, get_z_pos(j));
                    cube.transform.localScale = new Vector3(x_step, 15.0f, z_step);
                    cube.layer = 9; //"CubeWalls"
                }

            }
        }
    }

    public Vector3 GetRandomFreePos()
    {
        bool done = false;
        Vector3 pos = Vector3.zero;

        while (!done)
        {
            pos = new Vector3(Random.Range(x_low, x_high), 0f, Random.Range(z_low, z_high));
            if (traversability[get_i_index(pos.x), get_j_index(pos.z)] < 0.5f)
            {
                done = true;
            }

        }
        return pos;
    }



    public static TerrainInfo CreateFromJSON(string jsonString)
    {
        //Debug.Log("Reading json");
        return JsonConvert.DeserializeObject<TerrainInfo>(jsonString);
        //return JsonUtility.FromJson<TerrainInfo>(jsonString);
    }

    public string SaveToString()
    {
        JsonSerializerSettings JSS = new JsonSerializerSettings()
        {
            ReferenceLoopHandling = Newtonsoft.Json.ReferenceLoopHandling.Ignore
        };

        return JsonConvert.SerializeObject(this, Formatting.None, JSS);

        //return JsonConvert.SerializeObject(this); // throws ref loop error
        //return JsonUtility.ToJson(this); // cannot handle multi-dim arrays
    }

    public void WriteDataToFile(string jsonString)
    {
        string path = Application.dataPath + "/Resources/Text/" + file_name + ".json";
        Debug.Log("AssetPath:" + path);
        System.IO.File.WriteAllText(path, jsonString);
#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }


}
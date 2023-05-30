using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    public GameObject race_car;
    public bool random_positions = false;

    float start_time;
    public float completion_time;
    public float goal_tolerance = 10.0f;
    public bool finished = false;
    public int no_of_cars = 10;

    public List<GameObject> my_cars;

    // Use this for initialization
    void Awake()
    {
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

        start_time = Time.time;

        race_car.transform.position = terrain_manager.myInfo.start_pos + 2f * Vector3.up;
        race_car.transform.rotation = Quaternion.identity;
    }

    void Start()
    {
        start_time = Time.time;

        my_cars = new List<GameObject>();

        race_car.transform.position = terrain_manager.myInfo.start_pos + 2f * Vector3.up;
        race_car.transform.rotation = Quaternion.identity;

        Vector3 nominal_start_pos;
        Vector3 nominal_goal_pos;

        for (int i = 0; i < no_of_cars; i++)
        {
            GameObject new_car;

            // new_drone_AI;

            if(random_positions)
            {
                nominal_start_pos = terrain_manager.RandomConfiguration(i, no_of_cars, 0.07f);
                nominal_goal_pos = terrain_manager.RandomConfiguration(i, no_of_cars, 0.07f);
            }
            else
            {
                nominal_start_pos = terrain_manager.CircularConfiguration(i + (int)Mathf.Floor(no_of_cars / 2), no_of_cars, 0.75f);
                nominal_goal_pos = terrain_manager.CircularConfiguration(i, no_of_cars, 0.8f);
            }

            new_car = Instantiate(race_car, new Vector3(20.0f + i * 8.0f, 10.0f, 20f), Quaternion.identity);
            //Vector3 nominal_pos = terrain_manager.CircularConfiguration(i + (int) Mathf.Floor(no_of_cars / 2), no_of_cars, 0.75f);
            new_car.transform.position = terrain_manager.GetCollisionFreePosNear(nominal_start_pos, 50f);
            my_cars.Add(new_car);

            GameObject car_sphere = new_car.transform.Find("Sphere").gameObject;
            Color my_color = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
            car_sphere.GetComponent<Renderer>().material.SetColor("_Color", my_color);

            GameObject goal_sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);

            //nominal_pos = terrain_manager.CircularConfiguration(i, no_of_cars, 0.8f);
            //nominal_pos = terrain_manager.RandomConfiguration(i, no_of_cars, 0.2f);
            goal_sphere.transform.position = terrain_manager.GetCollisionFreePosNear(nominal_goal_pos, 50f);

            goal_sphere.transform.localScale = Vector3.one * 3f;
            goal_sphere.GetComponent<Renderer>().material.SetColor("_Color", my_color);
            goal_sphere.GetComponent<Collider>().isTrigger = true;


            // Assign goal point
            if (new_car.tag == "Car")
            {
                UnityStandardAssets.Vehicles.Car.CarAI new_AI;
                new_AI = new_car.GetComponent<UnityStandardAssets.Vehicles.Car.CarAI>();
                new_AI.my_goal_object = goal_sphere;
                new_AI.terrain_manager_game_object = terrain_manager_game_object;
            }

            if (new_car.tag == "Drone")
            {
                Debug.Log("stuff");
                DroneAI new_AI;
                new_AI = new_car.GetComponent<DroneAI>();
                new_AI.my_goal_object = goal_sphere;
                new_AI.terrain_manager_game_object = terrain_manager_game_object;
            }


            //new_AI.my_goal = goal_sphere.transform.position;
            //var cubeRenderer = new_sphere.GetComponent<Renderer>();
            //var cubeRenderer = new_car.GetComponent<Sphere>();


            //Call SetColor using the shader property name "_Color" and setting the color to red
            //cubeRenderer.material.SetColor("_Color", Color.cyan);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (!finished) // check is all cars have reached their goals, if so stop clock
        {
            bool all_cars_done = true;
            foreach (GameObject one_car in my_cars)
            {
                Vector3 car_pos = one_car.transform.position;
                Vector3 goal_pos = Vector3.zero;
                if (one_car.tag == "Car")
                {
                    goal_pos = one_car.GetComponent<UnityStandardAssets.Vehicles.Car.CarAI>().my_goal_object.transform.position;
                }

                if (one_car.tag == "Drone")
                {
                    goal_pos = one_car.GetComponent<DroneAI>().my_goal_object.transform.position;
                }


                if ((car_pos - goal_pos).sqrMagnitude > goal_tolerance * goal_tolerance)
                {
                    //Debug.Log("distSqr" + (car_pos - goal_pos).sqrMagnitude);
                    //Debug.DrawLine(car_pos, goal_pos, Color.magenta);
                    all_cars_done = false;
                    break;
                }
            }

            if (all_cars_done)
            {
                finished = true;
                completion_time = Time.time - start_time;
            }
        }
    }
}
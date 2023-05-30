using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Drone;
using Unity.Collections;
using Unity.Netcode;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;

public class GameManagerSoccer : MonoBehaviour
{
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    public GameObject vehicle_template;

    public GameObject blue_goal;
    public GameObject red_goal;

    public int blue_score;
    public int red_score;

    public GameObject ball;
    //public GameObject spawn_point_ball;

    float start_time;
    public float match_time;
    public float match_length = 100f;
    public float goal_tolerance = 10.0f;
    public bool finished = false;
    public int no_of_cars = 10;

    public List<GameObject> my_cars;
    public List<GameObject> cameras;
    private bool started = false;

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 300));
        if (!NetworkManager.Singleton.IsClient && !NetworkManager.Singleton.IsServer)
        {
            StartButtons();
        }
        else
        {
            StatusLabels(red_score, blue_score);
            if (!started && NetworkManager.Singleton.IsServer && NetworkManager.Singleton.ConnectedClients.Count == 2)
            {
                StartGame();
                started = true;
            }
            else if (!started && NetworkManager.Singleton.IsHost)
            {
                StartGame();
                started = true;
            }
            
            if (!NetworkManager.Singleton.IsServer && !NetworkManager.Singleton.IsHost)
            {
                cameras[0].SetActive(false);
            }

            if (!NetworkManager.Singleton.IsServer && NetworkManager.Singleton.LocalClientId == 1)
            {
                cameras[1].SetActive(true);
            }

            if (!NetworkManager.Singleton.IsServer && NetworkManager.Singleton.LocalClientId == 2)
            {
                cameras[2].SetActive(true);
            }

        }

      
        GUILayout.EndArea();
    }

    static void StartButtons()
    {
        if (GUILayout.Button("Host")) NetworkManager.Singleton.StartHost();
        if (GUILayout.Button("Client")) NetworkManager.Singleton.StartClient();
        if (GUILayout.Button("Server")) NetworkManager.Singleton.StartServer();
    }

    static void StatusLabels(int redScore, int blueScore)
    {
        var mode = NetworkManager.Singleton.IsHost ? "Host" : NetworkManager.Singleton.IsServer ? "Server" : "Client";

        GUILayout.Label("Transport: " + NetworkManager.Singleton.NetworkConfig.NetworkTransport.GetType().Name);
        GUILayout.Label("Mode: " + mode);
        if (NetworkManager.Singleton.IsServer)
        {
            GUILayout.Label("Clients: " + NetworkManager.Singleton.ConnectedClients.Count);
            GUILayout.Label("Time: " + NetworkManager.Singleton.ServerTime.Time.ToString("0.00"));
            GUILayout.Label("Red goals: " + redScore);
            GUILayout.Label("Blue goals: " + blueScore);
        }
        if (NetworkManager.Singleton.IsClient && NetworkManager.Singleton.LocalClientId == 1)
        {
            GUILayout.Label("Team: Blue");
        }
        if (NetworkManager.Singleton.IsClient && NetworkManager.Singleton.LocalClientId == 2)
        {
            GUILayout.Label("Team: Red");
        }
    }

    void Awake()
    {
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
        start_time = Time.time;
    }

    public void StartGame()
    {
        start_time = Time.time;

        my_cars = new List<GameObject>();

        blue_goal.transform.position = terrain_manager.myInfo.start_pos;
        red_goal.transform.position = terrain_manager.myInfo.goal_pos;

        var singletonConnectedClients = NetworkManager.Singleton.ConnectedClients;
        AddTeam("Blue", singletonConnectedClients.First().Key);
        AddTeam("Red", singletonConnectedClients.Last().Key);
    }

    public void AddTeam(string team_tag, ulong clientId)
    {
        for (int i = 0; i < 3; i++)
        {
            var position = CalculateVehiclePosition(my_cars.Count);
            var vehicle = CreateVehicle(position, team_tag);
            var networkObject = vehicle.GetComponent<NetworkObject>();
            networkObject.ChangeOwnership(clientId);
            my_cars.Add(vehicle);
        }
    }

    private GameObject CreateVehicle(Vector3 position, string team_tag)
    {
        GameObject vehicle = Instantiate(vehicle_template, Vector3.zero, Quaternion.identity);

        vehicle.tag = team_tag;
        vehicle.transform.position = position;

        if (vehicle.name.Contains("Car"))
        {
            var carManagerSoccer = vehicle.GetComponent<CarManagerSoccer>();
            carManagerSoccer.synctag.Value = team_tag;
        }

        if (vehicle.name.Contains("Drone"))
        {
            var carManagerSoccer = vehicle.GetComponent<DroneManagerSoccer>();
            carManagerSoccer.synctag.Value = team_tag;
        }


        var networkObject = vehicle.GetComponent<NetworkObject>();
        networkObject.Spawn();

        return vehicle;
    }

    private Vector3 CalculateVehiclePosition(int i)
    {
        Vector3 nominal_pos = terrain_manager.CircularConfiguration(i + (int) Mathf.Floor(no_of_cars / 2), no_of_cars, 0.2f);
        Vector3 position = terrain_manager.GetCollisionFreePosNear(nominal_pos, 10f);
        return position;
    }

    // Update is called once per frame
    void Update()
    {
        if (!finished)
        {
            match_time = Time.time - start_time;

            blue_score = ball.GetComponent<GoalCheck>().blue_score;
            red_score = ball.GetComponent<GoalCheck>().red_score;

            if (match_time > match_length) finished = true;
        }
    }
}
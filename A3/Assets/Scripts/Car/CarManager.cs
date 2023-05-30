using System;
using Unity.Netcode;
using Unity.Networking.Transport;
using UnityEngine;
using UnityEngine.Serialization;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarControllerNetwork))]
    public abstract class CarManager : NetworkBehaviour
    {
        public TerrainManager terrainManager;
        public CarControllerNetwork car;

        protected CarAIBehavior CarAIBehavior;

        private CarAction m_CarAction;

        public override void OnNetworkSpawn()
        {
            Initialize(); //Called on client and server when the server instantiates the car object.
            NetworkManager.Singleton.NetworkTickSystem.Tick += NetworkTick;
        }

        public virtual void Initialize()
        {
            terrainManager = FindObjectOfType<TerrainManager>();
            car = GetComponent<CarControllerNetwork>();
            CarAIBehavior = CreateAIBehavior();
            CarAIBehavior.Plan();
        }

        protected abstract CarAIBehavior CreateAIBehavior();
        protected abstract void DoServer(CarAction action);

        public void NetworkTick()
        {
            //TODO: When reading this, keep in mind that NetworkBehaviors are executed on each instance of the game in parallel.
            if (IsOwner)
            {
                var carAction = CarAIBehavior.Tick(); // Calculate new action, regardless of who the owner is.
                if (NetworkManager.IsServer)
                {
                    m_CarAction = carAction; // If owned by server (host mode) store action directly.
                }
                else
                {
                    UpdateServerSideActionServerRpc(carAction); // Send latest known action to server. Will be applied by the server.
                }
            }
        }

        public void FixedUpdate()
        {
            if (IsServer)
            {
                DoServer(m_CarAction); // Apply latest known action on server side fixed update.
            }
        }

        [ServerRpc]
        public void UpdateServerSideActionServerRpc(CarAction carAction)
        {
            m_CarAction = carAction; // Store latest known action on server side
        }
    }
}
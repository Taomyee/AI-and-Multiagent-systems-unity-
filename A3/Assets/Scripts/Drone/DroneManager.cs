using Unity.Netcode;
using UnityEngine;

namespace Drone
{
    [RequireComponent(typeof(DroneControllerNetwork))]
    public abstract class DroneManager : NetworkBehaviour
    {
        public TerrainManager terrainManager;
        public DroneControllerNetwork drone;

        protected DroneAIBehavior DroneAIBehavior;

        private DroneAction m_action;

        public override void OnNetworkSpawn()
        {
            Initialize(); //Called on client and server when the server instantiates the car object.
            NetworkManager.Singleton.NetworkTickSystem.Tick += NetworkTick;
        }

        public virtual void Initialize()
        {
            terrainManager = FindObjectOfType<TerrainManager>();
            drone = GetComponent<DroneControllerNetwork>();
            DroneAIBehavior = CreateAIBehavior();
            DroneAIBehavior.Plan();
        }

        public void NetworkTick()
        {
            //TODO: When reading this, keep in mind that NetworkBehaviors are executed on each instance of the game in parallel.
            if (IsOwner)
            {
                var action = DroneAIBehavior.Tick(); // Calculate new action, regardless of who the owner is.
                if (NetworkManager.IsServer)
                {
                    m_action = action; // If owned by server (host mode) store action directly.
                }
                else
                {
                    UpdateServerSideActionServerRpc(action); // Send latest known action to server. Will be applied by the server.
                }
            }
        }

        public void FixedUpdate()
        {
            if (IsServer)
            {
                DoServer(m_action); // Apply latest known action on server side fixed update.
            }
        }

        [ServerRpc]
        public void UpdateServerSideActionServerRpc(DroneAction action)
        {
            m_action = action; // Store latest known action on server side
        }

        protected abstract DroneAIBehavior CreateAIBehavior();
        protected abstract void DoServer(DroneAction action);
    }
}
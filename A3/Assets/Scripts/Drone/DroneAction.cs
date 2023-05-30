using Unity.Netcode;
using UnityEngine;

namespace Drone
{
    public struct DroneAction : INetworkSerializable
    {
        public Vector3 move_vector;
        public Vector3 kickDirection;

        public void NetworkSerialize<T>(BufferSerializer<T> serializer) where T : IReaderWriter
        {
            serializer.SerializeValue(ref move_vector);
            serializer.SerializeValue(ref kickDirection);
        }
    }
}
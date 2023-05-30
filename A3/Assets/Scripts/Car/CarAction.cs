using Unity.Netcode;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public struct CarAction : INetworkSerializable
    {
        public float steering;
        public float acceleration;
        public Vector3 kickDirection;

        public void NetworkSerialize<T>(BufferSerializer<T> serializer) where T : IReaderWriter
        {
            serializer.SerializeValue(ref steering);
            serializer.SerializeValue(ref acceleration);
            serializer.SerializeValue(ref kickDirection);
        }
    }
}
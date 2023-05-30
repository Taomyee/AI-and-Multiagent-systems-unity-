namespace UnityStandardAssets.Vehicles.Car
{
    public interface CarAIBehavior
    {
        public void Plan();
        public CarAction Tick();
    }
}
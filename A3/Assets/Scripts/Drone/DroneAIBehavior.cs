namespace Drone
{
    public interface DroneAIBehavior
    {
        public void Plan();
        public DroneAction Tick();
    }
}
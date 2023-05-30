using Drone;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class DroneAISoccer : DroneAIBehavior
    {
        private readonly DroneManagerSoccer manager;

        public DroneAISoccer(DroneManagerSoccer droneManagerSoccer)
        {
            manager = droneManagerSoccer;
        }

        public void Plan()
        {
        }

        public DroneAction Tick()
        {
            
            // Execute your path here
            // ...

            Vector3 avg_pos = Vector3.zero;

            foreach (GameObject friend in manager.soccer.friends)
            {
                avg_pos += friend.transform.position;
            }
            avg_pos = avg_pos / manager.soccer.friends.Length;
            //Vector3 direction = (avg_pos - transform.position).normalized;
            var transform = manager.CompareTag("Blue") ? manager.soccer.ball.transform : manager.soccer.own_goal.transform;
            Vector3 direction = (transform.transform.position - manager.drone.transform.position).normalized;



            // this is how you access information about the terrain
            int i =  manager.terrainManager.myInfo.get_i_index( manager.drone.transform.position.x);
            int j =  manager.terrainManager.myInfo.get_j_index( manager.drone.transform.position.z);
            float grid_center_x =  manager.terrainManager.myInfo.get_x_pos(i);
            float grid_center_z =  manager.terrainManager.myInfo.get_z_pos(j);
            /*
            Debug.DrawLine( manager.drone.transform.position, manager.soccer.ball.transform.position, Color.black);
            Debug.DrawLine( manager.drone.transform.position, manager.soccer.own_goal.transform.position, Color.green);
            Debug.DrawLine( manager.drone.transform.position, manager.soccer.other_goal.transform.position, Color.yellow);
            Debug.DrawLine( manager.drone.transform.position, manager.soccer.friends[0].transform.position, Color.cyan);
            Debug.DrawLine( manager.drone.transform.position, manager.soccer.enemies[0].transform.position, Color.magenta);
            */
            /*
            if (manager.soccer.CanKick())
            {
                Debug.DrawLine(manager.transform.position, manager.soccer.ball.transform.position, Color.red);
                //KickBall(maxKickSpeed * Vector3.forward);
            }
            */
            var droneAction = new DroneAction
            {
                move_vector = direction
            };
            
            // this is how you kick the ball (if close enough)
            // Note that the kick speed is added to the current speed of the ball (which might be non-zero)
            Vector3 kickDirection_goal = (manager.soccer.other_goal.transform.position - manager.soccer.transform.position).normalized;
            Vector3 kickDirection_ahead = (manager.soccer.ball.transform.position - manager.soccer.transform.position).normalized;

            // replace the human input below with some AI stuff
            if (Input.GetKeyDown("space"))
            {
                droneAction.kickDirection = 0.5f * manager.soccer.maxKickSpeed * kickDirection_goal;
            }

       
            return droneAction;
        }
    }
}
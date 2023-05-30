using System;
using Unity.Netcode;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class CarAISoccer : CarAIBehavior
    {
        private readonly CarManagerSoccer carManager;
        public Rigidbody myBody;

        public CarAISoccer(CarManagerSoccer carManager)
        {
            this.carManager = carManager;
            myBody = carManager.gameObject.GetComponent<Rigidbody>();
        }


        public void Plan()
        {
        }

        public CarAction Tick()
        {
            Vector3 avg_pos = Vector3.zero;

            foreach (GameObject friend in carManager.soccer.friends)
            {
                avg_pos += friend.transform.position;
            }

            avg_pos = avg_pos / carManager.soccer.friends.Length;
            //Vector3 direction = (avg_pos - transform.position).normalized;
            var transform = carManager.CompareTag("Blue") ? carManager.soccer.ball.transform : carManager.soccer.own_goal.transform;
            Vector3 direction = (transform.position - carManager.gameObject.transform.position).normalized;

            bool is_to_the_right = Vector3.Dot(direction, carManager.gameObject.transform.right) > 0f;
            bool is_to_the_front = Vector3.Dot(direction, carManager.gameObject.transform.forward) > 0f;
            float angle_to_destination = MathF.Acos(Vector3.Dot(direction, carManager.gameObject.transform.forward));
            if (!is_to_the_right) //steering uses negative angle to the left2
            {
                angle_to_destination = -1f * angle_to_destination;
            }
            float kP =  0.7f;  //proportional gain on pd-control for steering
            float kD = -0.2f;  //differential gain on pd-control for steering, -0.4
            float omega = myBody.angularVelocity.y;
            float speed = myBody.velocity.magnitude;
            float max_steering_bound = Mathf.Min(1f, 100f / speed * speed);
            //Debug.Log(myBody.angularVelocity);

            float steering = 0f;
            float acceleration = 0f;

            if (is_to_the_front)
            {
                steering = Mathf.Clamp(kP * angle_to_destination + kD * omega, -max_steering_bound, max_steering_bound);
                acceleration = 1f;
            }
            else if (is_to_the_right) // and behind
            {
                steering = -1f;
                acceleration = -1f;
            }
            else if (!is_to_the_right) // and behind
            {
                steering = 1f;
                acceleration = -1f;
            }
            

            // this is how you access information about the terrain
            int i = carManager.terrainManager.myInfo.get_i_index(carManager.gameObject.transform.position.x);
            int j = carManager.terrainManager.myInfo.get_j_index(carManager.gameObject.transform.position.z);
            float grid_center_x = carManager.terrainManager.myInfo.get_x_pos(i);
            float grid_center_z = carManager.terrainManager.myInfo.get_z_pos(j);

            Debug.DrawLine(carManager.gameObject.transform.position, carManager.soccer.ball.transform.position, Color.black);
            Debug.DrawLine(carManager.gameObject.transform.position, carManager.soccer.own_goal.transform.position, Color.green);
            Debug.DrawLine(carManager.gameObject.transform.position, carManager.soccer.other_goal.transform.position, Color.yellow);
            Debug.DrawLine(carManager.gameObject.transform.position, carManager.soccer.friends[0].transform.position, Color.cyan);
            Debug.DrawLine(carManager.gameObject.transform.position, carManager.soccer.enemies[0].transform.position, Color.magenta);

            if (carManager.soccer.CanKick())
            {
                Debug.DrawLine(carManager.gameObject.transform.position, carManager.soccer.ball.transform.position, Color.red);
                //KickBall(maxKickSpeed * Vector3.forward);
            }

            // We populate the action we wish to take with the car
            
            var carAction = new CarAction
            {
                steering = (float) (steering),
                acceleration = (float) (acceleration),
            };

            // replace the human input below with some AI stuff to set kick
            if (Input.GetKeyDown("space"))
            {
                Vector3 kickDirection = (carManager.soccer.other_goal.transform.position - carManager.gameObject.transform.position).normalized;
                carAction.kickDirection = carManager.soccer.maxKickSpeed * kickDirection;
            }

            return carAction;
        }
    }
}
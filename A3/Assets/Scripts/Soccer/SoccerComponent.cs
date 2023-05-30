using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class SoccerComponent : MonoBehaviour
    {
        public GameObject[] friends;
        public string friend_tag;
        public GameObject[] enemies;
        public string enemy_tag;

        public GameObject own_goal;
        public GameObject other_goal;
        public GameObject ball;

        public float dist;
        public float maxKickSpeed = 40f;
        public float lastKickTime = 0f;

        private Rigidbody m_BallRb;

        public void Initialize()
        {
            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friend_tag = gameObject.tag;
            if (friend_tag == "Blue")
                enemy_tag = "Red";
            else
                enemy_tag = "Blue";

            friends = GameObject.FindGameObjectsWithTag(friend_tag);
            enemies = GameObject.FindGameObjectsWithTag(enemy_tag);

            ball = GameObject.FindGameObjectWithTag("Ball");
            m_BallRb = ball.GetComponent<Rigidbody>();
        }

        public bool CanKick()
        {
            dist = (transform.position - ball.transform.position).magnitude;
            return dist < 7f && (Time.time - lastKickTime) > 0.5f;
        }

        public void KickBall(Vector3 velocity)
        {
            // impulse to ball object in direction away from agent
            if (CanKick())
            {
                velocity = Vector3.ClampMagnitude(velocity, maxKickSpeed); //maintain bound on kick speed
                velocity.y = 0f; // no kicking upwards
                // cars not allowed to kick towards themselves (lobbing)
                if (Vector3.Dot(velocity, ball.transform.position - transform.position) < 0f && name.Contains("Car"))
                {
                    print("Must kick away from car agent");
                    velocity = Vector3.zero;
                }
                if (Vector3.Magnitude(velocity + ball.GetComponent<Rigidbody>().velocity) > 2 * maxKickSpeed)
                {
                    print("No syncronized kicks allowed for extra speed");
                    velocity = Vector3.zero;
                }

                m_BallRb.AddForce(velocity, ForceMode.VelocityChange);
                lastKickTime = Time.time;
                print("ball was kicked by:" + name);
            }
        }
    }
}
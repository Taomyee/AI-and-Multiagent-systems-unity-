using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI4 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private Rigidbody m_Car_rb;
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject replayCar;
        public GameObject[] friends;
        public GameObject[] enemies;
        public int friends_num;

        public float interval;
        public Vector3[] carsPos;
        public Vector3[] startPos;
        public int carIndex;

        private float pre_time;
        private List<Vector3> pathway_point;
        private int pathway_index;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            m_Car_rb = GetComponent<Rigidbody>();

            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            replayCar = GameObject.FindWithTag("ReplayCar");
            friends = GameObject.FindGameObjectsWithTag("Player");
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            friends_num = friends.Length;

            carsPos = new Vector3[friends_num + 1];
            startPos = new Vector3[friends_num];
            interval = 8f;

            for (int i = 0; i < friends_num; i++)
            {
                startPos[i] = friends[i].transform.position;
            }
            Array.Sort(startPos, comparePostionByX);
            for (int i = 0; i < friends_num; i++)
            {
                if (Vector3.Distance(startPos[i], transform.position) <= 0.001)
                {
                    carIndex = i;
                }
            }

            pre_time = 0f;
            pathway_point = new List<Vector3>();
            pathway_index = 0;
            pathway_point[0] = startPos[carIndex];
            // Plan your path here
            // ...
    }

        int comparePostionByX(Vector3 a, Vector3 b)
        {
            if (a.x < b.x) return -1;
            else if (a.x > b.x) return 1;
            else return 0;
        }

        float hit_time = 0f;
        private void FixedUpdate()
        {
            Vector3 replayCarPos = replayCar.transform.position;
            Vector3 replayCarDir = replayCar.transform.right;
            Vector3 car_pos = transform.position;
            Debug.DrawLine(replayCarPos - replayCarDir * 50f, replayCarPos + replayCarDir * 50f, Color.red);

            Vector3 pointOnPerpendicular = carsPos[0]; 
            float angle = -105f;
            Vector3 toTransform = replayCar.transform.position - pointOnPerpendicular;
            Vector3 forwardProjection = Vector3.ProjectOnPlane(replayCar.transform.forward, Vector3.Cross(replayCar.transform.forward, toTransform).normalized);
            float angleBetween = Vector3.Angle(replayCar.transform.forward, forwardProjection);
            float rotateAngle = Mathf.Abs(angleBetween - angle);
            if (angleBetween > angle)
            {
                rotateAngle *= -1f;
            }
            Quaternion rotation = Quaternion.AngleAxis(rotateAngle, Vector3.Cross(replayCar.transform.forward, toTransform).normalized);
            Vector3 rotatedVector = rotation * toTransform;
            Vector3 targetPoint = pointOnPerpendicular + rotatedVector;

            Vector3 pointOnPerpendicular2 = carsPos[3];
            float angle2 = -105f;
            Vector3 toTransform2 = replayCar.transform.position - pointOnPerpendicular2;
            Vector3 forwardProjection2 = Vector3.ProjectOnPlane(replayCar.transform.forward, Vector3.Cross(replayCar.transform.forward, toTransform2).normalized);
            float angleBetween2 = Vector3.Angle(replayCar.transform.forward, forwardProjection2);
            float rotateAngle2 = Mathf.Abs(angleBetween2 - angle2);
            if (angleBetween2 > angle2)
            {
                rotateAngle2 *= -1f;
            }
            Quaternion rotation2 = Quaternion.AngleAxis(rotateAngle2, Vector3.Cross(replayCar.transform.forward, toTransform2).normalized);
            Vector3 rotatedVector2 = rotation2 * toTransform2;
            Vector3 targetPoint2 = pointOnPerpendicular2 + rotatedVector2;

            RaycastHit hit1;
            RaycastHit hit2;
            float maxRange = 50f;
            if (Physics.Raycast(carsPos[0], targetPoint-carsPos[0], out hit1, maxRange) || Physics.Raycast(carsPos[3], targetPoint2 - carsPos[3], out hit2, maxRange))
            {
                interval = 5f;
            }
            else
            {
                interval = 10f;
            }

            if (carIndex == 0)
            {
                carsPos[carIndex] = replayCarPos - replayCarDir * 2 * interval;
                Debug.DrawLine(carsPos[carIndex], carsPos[carIndex] + replayCar.transform.forward, Color.blue);
                Debug.DrawLine(carsPos[0], targetPoint, Color.yellow);
            }
            else if(carIndex == 1)
            {
                carsPos[carIndex] = replayCarPos - replayCarDir * interval;
                Debug.DrawLine(carsPos[carIndex], carsPos[carIndex] + replayCar.transform.forward, Color.blue);

            }
            else if (carIndex == 2)
            {
                carsPos[carIndex] = replayCarPos + replayCarDir * interval;
                Debug.DrawLine(carsPos[carIndex], carsPos[carIndex] + replayCar.transform.forward, Color.blue);

            }
            else if (carIndex == 3)
            {
                carsPos[carIndex] = replayCarPos + replayCarDir * 2 * interval;
                Debug.DrawLine(carsPos[carIndex], carsPos[carIndex] + replayCar.transform.forward, Color.blue);
                Debug.DrawLine(carsPos[3], targetPoint2, Color.yellow);
            }
            /*if (Time.time - pre_time >= 0.1f)
            {
                pathway_point.Add(carsPos[carIndex]);
                pre_time = Time.time;
                    
            }*/
            pathway_point.Add(carsPos[carIndex]);
            Vector3 next_pos = pathway_point[pathway_index];
            float speed = 10f;

            float min_dis = float.MaxValue;
            int min_idx = pathway_index;

            for (int idx = pathway_index; idx < pathway_point.Count; idx++)
            {
                float car_pathpoint_dis = Vector3.Distance(pathway_point[idx], car_pos);
                if (car_pathpoint_dis < min_dis)
                {
                    min_dis = car_pathpoint_dis;
                    min_idx = idx;
                }
                if (Vector3.Distance(car_pos, pathway_point[pathway_point.Count-1]) < 30f)
                {
                    speed = 6f;
                }
                else
                {
                    speed = 10f;
                }
            }
            pathway_index = min_idx;
            next_pos = pathway_point[pathway_index];
            
            float dist_to_next = Vector3.Distance(car_pos, next_pos);
            if (dist_to_next <= 30f && pathway_index + 1 < pathway_point.Count)
            {
                pathway_index += 1;
            }

            Debug.DrawLine(car_pos, pathway_point[pathway_index], Color.blue);
            // Vector3 car_pos = transform.position;

            // Execute your path here
            // ...

            /*
            Vector3 acceleration = 0.5f * ((next_pos - car_pos) + 0.5f * m_Car_rb.velocity * (speed - m_Car_rb.velocity.magnitude));
            float angle = Vector3.Angle(acceleration, transform.forward);
            var left_or_right = Vector3.Cross(acceleration, transform.forward);
            if (left_or_right.y > 0) angle = -angle;
            float steering = angle / 180f;
            if (Vector3.Angle(next_pos - car_pos, transform.forward) > 60f)
            {
                Debug.Log("Large angle");
                if (left_or_right.y > 0) steering = -1;
                else steering = 1;
                if (m_Car_rb.velocity.magnitude <= 2) m_Car.Move(steering, 0.5f, 0.5f, 0f);
                else m_Car.Move(steering, 0.3f, 0.3f, 0f);
            }
            else if (Vector3.Angle(next_pos - car_pos, transform.forward) < 60f && Vector3.Angle(next_pos - car_pos, transform.forward) > 30f)
            {
                Debug.Log("Small angle");
                if (left_or_right.y > 0) steering = -steering;
                //else steering = 1;
                if (m_Car_rb.velocity.magnitude <= speed) m_Car.Move(steering, 1f, 1f, 0f);
                else m_Car.Move(steering, 1f, 1f, 0f);
            }
            else
            {
                Debug.Log("Normal");
                if (left_or_right.y > 0) steering = -steering;
                m_Car.Move(steering, 1f, 1f, 0f);
            }
            */
            
            Vector3 direction = (next_pos - car_pos).normalized;

            float steering = 0f;
            float acceleration = 0f;
            Vector3 acceleration_vector = 1f * ((next_pos - car_pos) + m_Car_rb.velocity * (speed - m_Car_rb.velocity.magnitude));
            float acceleration_val = Vector3.Dot(acceleration_vector, transform.forward);
            bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
            bool is_to_the_front = Vector3.Angle(acceleration_vector, transform.forward) < 180f;// Vector3.Dot(direction, transform.forward) > 0f;
            if (is_to_the_right && is_to_the_front) //right front
            {
                steering = 1f;
                acceleration = 1f;
            }
            else if (is_to_the_right && !is_to_the_front) //right back
            {
                steering = -1f;
                acceleration = -1f;
            }
            else if (!is_to_the_right && is_to_the_front) //left front
            {
                steering = -1f;
                acceleration = 1f;
            }
            else if (!is_to_the_right && !is_to_the_front) //left back
            {
                steering = 1f;
                acceleration = -1f;
            }
            if (Vector3.Angle(next_pos - car_pos, transform.forward) > 60f)
            {
                acceleration = acceleration_val;
            }
            /*
            if (!is_to_the_front && Vector3.Angle(next_pos - car_pos, transform.forward) < 150f)
            {
                steering = -steering;
            }*/
            m_Car.Move(steering, acceleration, acceleration, 0f);
            

            /*
            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));
            */

        }
    }
}

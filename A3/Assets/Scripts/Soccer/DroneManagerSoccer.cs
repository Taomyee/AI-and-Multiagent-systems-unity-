using Drone;
using Unity.Collections;
using Unity.Netcode;
using UnityEngine;
using System.Collections.Generic;
using System;
using System.Linq;
using Panda;

namespace UnityStandardAssets.Vehicles.Car
{
    public class DroneManagerSoccer : DroneManager
    {
        public SoccerComponent soccer;
        public NetworkVariable<FixedString64Bytes> synctag = new NetworkVariable<FixedString64Bytes>();

        private string team = "Blue - ";
        private bool printAction = false;
        private bool printPositionObj = true;
        private bool printObstacleShooting = true;

        private DroneController m_Drone; // the drone controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public string friend_tag;
        public GameObject[] enemies;
        public string enemy_tag;

        public GameObject own_goal;
        public GameObject other_goal;
        public GameObject ball;
        private Vector3 frontPost;
        private Vector3 backPost;
        private Vector3 oldTargetPos;

        PandaBehaviour myPandaBT;
        private Rigidbody my_rigidbody;

        // Fixe parameters
        private float ballDiameter = 4.3f;
        private float robotDiameter = 2.4f;
        private float GoalHeight = 30f;
        public float maxKickSpeed = 40f;
        private float MaxSpeedRobot = 18f; // ?
        private float lastKickTime = 0f;
        bool areBlue = false;

        // Tunable parameters
        //The distance at which attacking players should back up from the ball carrier to create space and passing options.
        private float distanceBackupAttack = 40f;
        //The distance at which defending players should back up to maintain defensive shape and prevent the attacking team from penetrating
        private float distanceBackupDefend = 60f;
        //The distance that attacking players should aim to be positioned in order to create passing options and maintain possession.
        private float distanceSolutionAttackZ = 25f;
        private float distanceSolutionAttackX = 30f;  // right-left
        //The distance that defending players should aim to be positioned in order to prevent the attacking team from advancing.
        private float distanceSolutionDefendZ = 20f;
        private float distanceSolutionDefendX = 45f;  // right-left

        //The distance at which an opposing player is considered to be "arriving" and should be closed down or tackled.
        private float distanceEnemyArriving = 17f;
        //The maximum distance at which a player should attempt to dribble the ball past an opponent.
        private float distanceDribbling = 20f;
        //The distance at which a player should attempt to win the ball back from an opponent who has possession.
        private float distanceChallenge = 15f;
        //The maximum distance that the last defender should be positioned from their own goal to prevent the opposing team from scoring.
        private float limitLastDefender = 22.5f;
        //The minimum distance that a player should aim to clear the ball when attempting to relieve pressure from their own defensive zone.
        private float limitClearance = 22.5f;
        private float ballInAirLimit = 5f;

        private float kP = 2.75f;  // 2f
        private float kD = 2f;  // 1f
        private float offsetFromPost = 1.5f;


        Vector3 EnemyGoalPos;
        Vector3 EnemyGoalTop;
        Vector3 EnemyGoalBottom;
        Vector3 OurGoalPos;
        Vector3 OurGoalTop;
        Vector3 OurGoalBottom;

        float durationDraw = 0.2f;

        private bool challenger = false;
        private bool backup = false;
        private bool solution = false;

        private Vector3 angleToShoot;
        private Vector3 bestDirectionForShooting;

        public override void Initialize()
        {
            base.Initialize();
            soccer = GetComponent<SoccerComponent>();
            tag = synctag.Value.ToString();
        }

        private void Start()
        {
            GameObject go = transform.Find("Sphere").gameObject;

            if (CompareTag("Blue"))
            {
                team = "Blue - ";
                soccer.own_goal = GameObject.FindWithTag("BlueGoal");
                soccer.other_goal = GameObject.FindWithTag("RedGoal");
                go.GetComponent<Renderer>().material.SetColor("_Color", Color.blue);
            }
            else
            {
                team = "Red - ";
                printAction = true;
                soccer.own_goal = GameObject.FindWithTag("RedGoal");
                soccer.other_goal = GameObject.FindWithTag("BlueGoal");
                go.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
            }

            soccer.Initialize();

            // get the car controller
            m_Drone = GetComponent<DroneController>();
            terrain_manager = FindObjectOfType<TerrainManager>();

            myPandaBT = GetComponent<PandaBehaviour>();

            ball = GameObject.FindGameObjectWithTag("Ball");
            my_rigidbody = GetComponent<Rigidbody>();

            friend_tag = gameObject.tag;
            if (friend_tag == "Blue")
                enemy_tag = "Red";
            else
                enemy_tag = "Blue";

            friends = GameObject.FindGameObjectsWithTag(friend_tag);
            enemies = GameObject.FindGameObjectsWithTag(enemy_tag);

            if (AreWeBlue())
                areBlue = true;

            OurGoalPos = terrain_manager.myInfo.start_pos;
            OurGoalTop = OurGoalPos;
            OurGoalBottom = OurGoalPos;
            OurGoalTop.z = OurGoalTop.z + GoalHeight / 2;
            OurGoalBottom.z = OurGoalBottom.z - GoalHeight / 2;

            EnemyGoalPos = terrain_manager.myInfo.goal_pos;
            EnemyGoalTop = EnemyGoalPos;
            EnemyGoalBottom = EnemyGoalPos;
            EnemyGoalTop.z = EnemyGoalTop.z + GoalHeight / 2;
            EnemyGoalBottom.z = EnemyGoalBottom.z - GoalHeight / 2;

            own_goal = soccer.own_goal;
            other_goal = soccer.other_goal;

        }

        protected override DroneAIBehavior CreateAIBehavior()
        {
            return new DroneAISoccer(this);
        }

        protected override void DoServer(DroneAction action)
        {
            drone.Move_vect(action.move_vector);
            if (action.kickDirection != Vector3.zero)
                soccer.KickBall(action.kickDirection);
        }

        private bool AreWeBlue()
        {
            return soccer.own_goal.transform.position == terrain_manager.myInfo.start_pos;
        }

        
        // Update is called once per frame
        private void Update()
        {
            myPandaBT.Reset();
            myPandaBT.Tick();

            Vector3 ballPos = ball.transform.position;
            Vector3 ballTop = ballPos;
            ballTop.z = ballTop.z + ballDiameter / 2;
            Vector3 ballBottom = ballPos;
            ballBottom.z = ballBottom.z - ballDiameter / 2;

            if (areBlue)
            {
                Debug.DrawLine(ballTop, OurGoalTop, Color.blue);
                Debug.DrawLine(ballBottom, OurGoalBottom, Color.blue);
                Debug.DrawLine(ballTop, EnemyGoalTop, Color.red);
                Debug.DrawLine(ballBottom, EnemyGoalBottom, Color.red);
            }
            else
            {
                Debug.DrawLine(ballTop, OurGoalTop, Color.red);
                Debug.DrawLine(ballBottom, OurGoalBottom, Color.red);
                Debug.DrawLine(ballTop, EnemyGoalTop, Color.blue);
                Debug.DrawLine(ballBottom, EnemyGoalBottom, Color.blue);
            }

            DrawVelocity();
        }
        

        // ###############################
        // FUNCTIONS
        // ###############################

        private void drawPoint(Vector3 p)
        {
            float length = 1f;
            Vector3 d1 = new Vector3(length, 0, length);
            Vector3 d2 = new Vector3(-length, 0, length);
            if (areBlue)
            {
                Debug.DrawLine(p + d1, p - d1, Color.blue);
                Debug.DrawLine(p + d2, p - d2, Color.blue);
            }
            else
            {
                Debug.DrawLine(p + d1, p - d1, Color.red);
                Debug.DrawLine(p + d2, p - d2, Color.red);
            }
        }

        private float distance(Vector3 pA, Vector3 pB)
        {
            return (pA - pB).magnitude;
        }

        private float mindistanceFromRobotsToPoint(Vector3 position)
        {
            float min_dist = 1000000f;
            foreach (GameObject friend in friends) // look for smallest distance to the point
            {
                float dist = distance(friend.transform.position, position);
                if (dist < min_dist)
                    min_dist = dist;
            }

            return min_dist;
        }

        private float mindistanceFromRobotsOnGoodSideOfBall(Vector3 position)
        {
            float min_dist = 1000000f;
            foreach (GameObject friend in friends) // look for smallest distance to the point
            {
                if ((friend.transform.position.x - own_goal.transform.position.x) / (ball.transform.position.x - own_goal.transform.position.x) < 1)  // good side of the ball
                {
                    float dist = distance(friend.transform.position, position);
                    if (dist < min_dist)
                        min_dist = dist;
                }
            }

            if (min_dist == 1000000f)  // all robots on the wrong side
            {
                foreach (GameObject friend in friends) // look for smallest distance to the point
                {
                    float dist = distance(friend.transform.position, position);
                    if (dist < min_dist)
                        min_dist = dist;
                }
            }

            return min_dist;
        }

        private float distanceEnemyToBall()
        {
            float min_dist_enemy = 1000000f;
            foreach (GameObject enemy in enemies) // look for smallest enemy distance to ball
            {
                float dist = distance(enemy.transform.position, ball.transform.position);
                if (dist < min_dist_enemy)
                    min_dist_enemy = dist;
            }

            return min_dist_enemy;
        }

        private void DrawVelocity()
        {
            Vector3 ballPos = ball.transform.position;
            Rigidbody rbBall = ball.GetComponent<Rigidbody>();
            Vector3 ballSpeed = rbBall.velocity;

            Vector3 robotPos = transform.position;
            Rigidbody rbCurrentRobot = GetComponent<Rigidbody>();
            Vector3 currentRobotSpeed = rbCurrentRobot.velocity;

            Vector3 RobotWithMaxSpeed = robotPos;
            RobotWithMaxSpeed.x = RobotWithMaxSpeed.x + MaxSpeedRobot;

            Debug.DrawLine(ballPos, ballPos + ballSpeed, Color.black);
            Debug.DrawLine(robotPos, robotPos + currentRobotSpeed, Color.black);
            //Debug.DrawLine(robotPos, RobotWithMaxSpeed, Color.green);
        }

        private void CalculateFrontAndBackPost()
        {
            if (ball.transform.position.z < 100)  // bottom
            {
                frontPost = own_goal.transform.position - new Vector3(0f, 0f, GoalHeight / 2 - offsetFromPost);
                backPost = own_goal.transform.position + new Vector3(0f, 0f, GoalHeight / 6 - offsetFromPost);
            }
            else  // top
            {
                frontPost = own_goal.transform.position + new Vector3(0f, 0f, GoalHeight / 2 - offsetFromPost);
                backPost = own_goal.transform.position - new Vector3(0f, 0f, GoalHeight / 6 - offsetFromPost);
            }
        }

        private void AssignChallenger()
        {
            Vector3 pos = transform.position;
            Vector3 ball_pos = ball.transform.position;

            float dist = distance(pos, ball_pos);
            float min_dist = mindistanceFromRobotsOnGoodSideOfBall(ball_pos);  // takes into account the side where the robots are (/ball)
            if (dist == min_dist)  // between our_goal and ball
                challenger = true;
        }

        private void AssignBackupOrSolution()
        {
            if (!challenger)  // no role assigned yet
            {
                Vector3 pos = transform.position;
                Vector3 ball_pos = ball.transform.position;
                Vector3 backupPosition = ball.transform.position;
                Vector3 solutionPosition = ball.transform.position;
                backupPosition.y = 0.01f;
                solutionPosition.y = 0.01f;
                if (areBlue)
                {
                    backupPosition.x = backupPosition.x - distanceBackupAttack;
                    solutionPosition.x = solutionPosition.x - distanceSolutionAttackX;
                }
                else  // red
                {
                    backupPosition.x = backupPosition.x + distanceBackupAttack;
                    solutionPosition.x = solutionPosition.x + distanceSolutionAttackX;
                }
                if (solutionPosition.z > 100)  // top of the field
                    solutionPosition.z = solutionPosition.z - distanceSolutionAttackZ;
                else  // bottom of the field
                    solutionPosition.z = solutionPosition.z + distanceSolutionAttackZ;

                float dist = distance(pos, ball_pos);
                float min_dist = mindistanceFromRobotsOnGoodSideOfBall(ball_pos);  // takes into account the side where the robots are (/ball)

                // find other free robot for backup/solution
                Vector3 otherPos = pos;
                foreach (GameObject friend in friends)  // look for smallest distance to the point
                {
                    float otherDist = distance(friend.transform.position, ball_pos); // distance to ball
                    if ((otherDist != min_dist) && (otherDist != dist))
                        otherPos = friend.transform.position;
                }

                // Calculate cost: -> backup / other -> solution
                float dist1 = Mathf.Max(distance(pos, backupPosition), distance(otherPos, solutionPosition));
                // Calculate cost: -> solution / other -> backup
                float dist2 = Mathf.Max(distance(pos, solutionPosition), distance(otherPos, backupPosition));

                if (dist1 < dist2)
                    backup = true;
                else
                    solution = true;
            }
        }

        private void AttackRoleAssignment()
        {
            // initialize
            challenger = false;
            backup = false;
            solution = false;

            // challenger
            AssignChallenger();

            // backup & solution
            AssignBackupOrSolution();

            if (printAction)
                Debug.Log(team + "AttackRoleAssignment : position = " + transform.position + "; challenger = " + challenger + "; backup = " + backup + "; solution = " + solution);
        }

        bool IsCloserToBallThanEnemy()
        {
            Vector3 ball_pos = ball.transform.position;
            float min_dist = mindistanceFromRobotsToPoint(ball_pos);
            float min_dist_enemy = distanceEnemyToBall();

            bool res = (min_dist <= min_dist_enemy);
            return res;
        }

        bool NotInDefendPart()  // false if we are close to our goal
        {
            float limitDefend = 50f;
            if (areBlue)
            {
                float dist = ball.transform.position.x - own_goal.transform.position.x;
                if (dist < limitDefend)
                    return false;
            }
            else
            {
                float dist = own_goal.transform.position.x - ball.transform.position.x;
                if (dist < limitDefend)
                    return false;
            }
            return true;
        }

        private bool CanKick()
        {
            float dist = distance(transform.position, ball.transform.position);
            return dist < 7f && (Time.time - lastKickTime) > 0.5f;
        }

        private void KickBall(Vector3 velocity)
        {
            // impulse to ball object in direction away from agent
            if (CanKick())
            {
                velocity.y = 0f;
                Rigidbody rb = ball.GetComponent<Rigidbody>();
                rb.AddForce(velocity, ForceMode.VelocityChange);
                lastKickTime = Time.time;
                //print("ball was kicked ");
            }
        }

        

        private bool DirectedToEnemyGoal(Vector3 dir)
        {
            Vector3 ballPos = ball.transform.position;
            Vector3 enemyGoalPos; // center of the enemy goal
            if (areBlue)
                enemyGoalPos = terrain_manager.myInfo.goal_pos;
            else // we are red
                enemyGoalPos = terrain_manager.myInfo.start_pos;

            if ((enemyGoalPos.x - ballPos.x) / dir.x > 0) // shoot in direction of the enemy goal
            {
                float zAtGoal = ballPos.z + (enemyGoalPos.x - ballPos.x) / dir.x * dir.z; // z coordinate of the ball when at the x-level of the enemy goal
                Debug.Log(team + "dteg: zAtGoal = " + zAtGoal);
                Debug.DrawLine(ballPos, ballPos + 4 * dir, Color.white);
                if (enemyGoalPos.z - GoalHeight / 2 <= zAtGoal && zAtGoal <= enemyGoalPos.z + GoalHeight / 2)
                    return true;
            }

            return false;
        }

        private bool DirectedToOurGoal(Vector3 dir)
        {
            Vector3 ballPos = ball.transform.position;
            Vector3 ourGoalPos; // center of our goal
            if (areBlue)
                ourGoalPos = terrain_manager.myInfo.start_pos;
            else // we are red
                ourGoalPos = terrain_manager.myInfo.goal_pos;


            if ((ourGoalPos.x - ballPos.x) / dir.x > 0) // shoot in direction of our goal
            {
                float zAtGoal = ballPos.z + (ourGoalPos.x - ballPos.x) / dir.x * dir.z; // z coordinate of the ball when at the x-level of our goal
                                                                                        //Debug.Log(team + "dteg: zAtGoal = " + zAtGoal);
                                                                                        //Debug.DrawLine(ballPos, ballPos+4*dir, Color.white);
                if (ourGoalPos.z - GoalHeight / 2 <= zAtGoal && zAtGoal <= ourGoalPos.z + GoalHeight / 2)
                    return true;
            }

            return false;
        }

        private bool NoObstacleForShooting(Vector3 dir)
        {
            Vector3 dirOrthogonal = new Vector3(-dir.z, 0, dir.x).normalized;

            Vector3 ballPos = ball.transform.position;
            Vector3 ballPosUp = ballPos + dirOrthogonal * ballDiameter;
            Vector3 ballPosDown = ballPos - dirOrthogonal * ballDiameter;

            Vector3 enemyGoalPos = other_goal.transform.position;

            float xAfterGoal = enemyGoalPos.x + 25f;
            float zAfterGoal = ballPos.z + (xAfterGoal - ballPos.x) / dir.x * dir.z;

            Vector3 trajectory = new Vector3(xAfterGoal - ballPos.x, 0, zAfterGoal - ballPos.z);
            float dist = trajectory.magnitude;
            float timeBallArrive = dist / maxKickSpeed;

            Vector3 afterGoalUp = new Vector3(xAfterGoal + dirOrthogonal.x * timeBallArrive * MaxSpeedRobot, 0, zAfterGoal + dirOrthogonal.z * timeBallArrive * MaxSpeedRobot);
            Vector3 afterGoalDown = new Vector3(xAfterGoal - dirOrthogonal.x * timeBallArrive * MaxSpeedRobot, 0, zAfterGoal - dirOrthogonal.z * timeBallArrive * MaxSpeedRobot);
            Vector3 dirUp = (afterGoalUp - ballPosUp).normalized;
            Vector3 dirDown = (afterGoalDown - ballPosDown).normalized;

            Debug.Log(team + "nofs: zAtGoal = " + zAfterGoal + "; " + afterGoalUp + "; " + afterGoalDown + "; " + dirUp + "; " + dirDown);
            Debug.DrawLine(ballPos, ballPos + dirUp, Color.white);
            Debug.DrawLine(ballPos, ballPos + dirDown, Color.white);

            // Look if friends or enemies are in the VO
            foreach (GameObject friend in friends) // look for smallest distance to ball
            {
                Vector3 pos = friend.transform.position;
                if (pos.z < ballPosUp.z + dirUp.z * (pos.x - ballPosUp.x) / dirUp.x) // below dirUp
                {
                    if (pos.z > ballPosDown.z + dirDown.z * (pos.x - ballPosDown.x) / dirDown.x) // above dirDown
                        return false;
                }
            }

            foreach (GameObject enemy in enemies) // look for smallest enemy distance to ball
            {
                Vector3 pos = enemy.transform.position;
                if (pos.z < ballPosUp.z + dirUp.z * (pos.x - ballPosUp.x) / dirUp.x) // below dirUp
                {
                    if (pos.z > ballPosDown.z + dirDown.z * (pos.x - ballPosDown.x) / dirDown.x) // above dirDown
                        return false;
                }
            }

            Vector3 pointAfterGoal = new Vector3(xAfterGoal, 0, zAfterGoal);
            Debug.DrawLine(ballPos, pointAfterGoal, Color.white);

            return true;
        }

        private bool FreeAngleOnGoal()
        {
            Vector3 ballPos = ball.transform.position;
            Vector3 direction = ballPos - transform.position;  // direction : robot -> ball

            if (DirectedToEnemyGoal(direction))
            {
                if (NoObstacleForShooting(direction))
                {
                    return true;
                }
            }
            return false;
        }

        private float convertToRadian(float angle)
        {
            return angle * (float)Math.PI / 180;
        }

        private bool PossibilityShootOnGoal()
        {
            bool foundAnAngleToshoot = false;
            Vector3 ballPos = ball.transform.position;
            Vector3 enemyGoalPos = other_goal.transform.position;
            //Vector3 direction = enemyGoalPos - ballPos; 

            List<(Vector3, float)> friendsList = new List<(Vector3, float)>();
            List<(Vector3, float)> enemiesList = new List<(Vector3, float)>();

            foreach (GameObject friend in friends)  // look for smallest distance to the point
            {
                if ((friend.transform.position.x - enemyGoalPos.x) / (ball.transform.position.x - enemyGoalPos.x) < 1)  // between ball and enemyGoal
                    friendsList.Add((friend.transform.position, robotDiameter / 2 + ballDiameter / 2));
            }

            foreach (GameObject enemy in enemies)  // look for smallest distance to the point
            {
                if ((enemy.transform.position.x - enemyGoalPos.x) / (ball.transform.position.x - enemyGoalPos.x) < 1)  // between ball and enemyGoal
                {
                    float angleBetween = (float)Vector3.Angle(enemyGoalPos - ballPos, enemy.transform.position - ballPos);
                    angleBetween = Mathf.Abs(convertToRadian(angleBetween));
                    float xDistToBall = (float)distance(ballPos, enemy.transform.position) * Mathf.Cos(angleBetween);
                    float radius = robotDiameter / 2 + ballDiameter / 2 + xDistToBall * MaxSpeedRobot / 400;

                    if (printObstacleShooting)
                    {
                        Vector3 newPoint = enemy.transform.position;
                        newPoint.z = newPoint.z + radius;
                        Debug.DrawLine(enemy.transform.position, newPoint, Color.green, durationDraw);
                        newPoint.z = newPoint.z - 2 * radius;
                        Debug.DrawLine(enemy.transform.position, newPoint, Color.green, durationDraw);
                    }

                    enemiesList.Add((enemy.transform.position, radius));
                }
            }


            int numberAngles = 100;
            float step = (float)1 / numberAngles;
            Vector3 enemyGoalObj = enemyGoalPos;
            enemyGoalObj.z = enemyGoalObj.z + GoalHeight / 2;  // up

            bool ShootOnTopOfGoal = true;  // start at the top
            bool ShootOnBottomOfGoal = false;

            float boundaryFreeAngle = enemyGoalObj.z;
            float currentFreeAngle = 0;
            float currentMaxFreeAngle = 0;
            float correspondingDistance = 0;

            Vector3 vectorObj = ballPos;

            for (int i = 0; i < numberAngles; i++)  // up to down
            {
                enemyGoalObj.z = enemyGoalObj.z - step * GoalHeight;
                Vector3 directionObj = enemyGoalObj - ballPos;

                bool shotNotBlocked = true;
                foreach ((Vector3 p, float r) obj in friendsList)  // look for smallest distance to the point
                {
                    float angleBetween = (float)Vector3.Angle(directionObj, obj.p - ballPos);
                    angleBetween = Mathf.Abs(convertToRadian(angleBetween));
                    float zDistToBall = (float)distance(ballPos, obj.p) * Mathf.Sin(angleBetween);
                    if (zDistToBall < obj.r)
                        shotNotBlocked = false;
                }
                foreach ((Vector3 p, float r) obj in enemiesList)  // look for smallest distance to the point
                {
                    float angleBetween = (float)Vector3.Angle(directionObj, obj.p - ballPos);
                    //Debug.Log("angleBetweenDegree = " + angleBetween);
                    angleBetween = Mathf.Abs(convertToRadian(angleBetween));
                    float zDistToBall = (float)distance(ballPos, obj.p) * Mathf.Sin(angleBetween);
                    //Debug.Log("psog : zDistToBall = " + zDistToBall + "; angleBetween = " + angleBetween + "; Sin(angleBetween) = " + Mathf.Sin(angleBetween));
                    if (zDistToBall < obj.r)
                        shotNotBlocked = false;
                }

                if (!shotNotBlocked)
                    ShootOnTopOfGoal = false;
                if ((i == numberAngles - 1) && shotNotBlocked)
                    ShootOnBottomOfGoal = true;

                if (!shotNotBlocked)
                {
                    boundaryFreeAngle = 0;
                }

                if (shotNotBlocked)
                {
                    if (boundaryFreeAngle == 0)  // start of a new free zone to shoot on goal
                        boundaryFreeAngle = enemyGoalObj.z;  // new boundary
                    else  // interval of free angles
                    {
                        Vector3 vectorBoundaryFreeAngle = enemyGoalObj;
                        vectorBoundaryFreeAngle.z = boundaryFreeAngle;
                        currentFreeAngle = (float)Vector3.Angle(vectorBoundaryFreeAngle - ballPos, enemyGoalObj - ballPos);  // current free angle
                        currentFreeAngle = Mathf.Abs(convertToRadian(currentFreeAngle));
                        if (currentFreeAngle > currentMaxFreeAngle)
                        {
                            currentMaxFreeAngle = currentFreeAngle;
                            correspondingDistance = boundaryFreeAngle - enemyGoalObj.z;  // z free distance on the goal

                            if (ShootOnTopOfGoal)
                            {
                                float distToTopOfGoal = 0.5f * (correspondingDistance / GoalHeight) * (correspondingDistance / GoalHeight);
                                float obj = (enemyGoalPos.z + GoalHeight / 2) - distToTopOfGoal;
                                vectorObj = enemyGoalPos;
                                vectorObj.z = obj;
                                angleToShoot = vectorObj - ballPos;  // shooting angle
                                foundAnAngleToshoot = true;
                                //Debug.Log("psog : obj = " + obj);
                            }

                            else if (ShootOnBottomOfGoal)
                            {
                                float distToBottomOfGoal = 0.5f * (correspondingDistance / GoalHeight) * (correspondingDistance / GoalHeight);
                                float obj = (enemyGoalPos.z - GoalHeight / 2) + distToBottomOfGoal;
                                vectorObj = enemyGoalPos;
                                vectorObj.z = obj;
                                angleToShoot = vectorObj - ballPos;  // shooting angle
                                foundAnAngleToshoot = true;
                                //Debug.Log("psog : obj = " + obj);
                            }

                            else
                            {
                                float obj = boundaryFreeAngle - correspondingDistance / 2;
                                vectorObj = enemyGoalPos;
                                vectorObj.z = obj;
                                angleToShoot = vectorObj - ballPos;  // shooting angle
                                foundAnAngleToshoot = true;
                                //Debug.Log("psog : obj = " + obj);
                            }

                        }
                    }
                }
            }

            if (foundAnAngleToshoot)
            {
                drawPoint(vectorObj);
                //Debug.Log("PSOG : angleToShoot = " + angleToShoot + "; vectorObj = " + vectorObj + " | Goal : " + (enemyGoalPos.z + GoalHeight/2) + " / " + (enemyGoalPos.z - GoalHeight/2));
                return true;
            }

            return false;
        }

        private Vector3 findBallVelocityToAdd(Vector3 dirObj)  // todo look if angle < 90бу
        {

            Vector3 ball_pos = ball.transform.position;
            Rigidbody rbBall = ball.GetComponent<Rigidbody>();
            Vector3 ballSpeed = rbBall.velocity;
            dirObj = dirObj.normalized;
            Vector3 dirObjOrtho = new Vector3(-dirObj.z, 0, dirObj.x);

            float angleBetween = (float)Vector3.Angle(ballSpeed, dirObj);
            angleBetween = convertToRadian(angleBetween);
            float zDistToObj = (float)ballSpeed.magnitude * Mathf.Sin(angleBetween);

            int sign = Math.Sign(dirObjOrtho.x * ballSpeed.x + dirObjOrtho.z * ballSpeed.z);
            Vector3 closestVelocity = ball_pos - sign * zDistToObj * dirObjOrtho;

            if (distance(ball_pos, closestVelocity) > maxKickSpeed)  // maxKickSpeed not enough power
                return closestVelocity;

            float angleCos = distance(ball_pos, closestVelocity) / maxKickSpeed;
            float angleNewObj = Mathf.Acos(angleCos);
            float distNew = distance(ball_pos, closestVelocity) * Mathf.Tan(angleNewObj);

            Vector3 newObj = closestVelocity + distNew * dirObj;

            return newObj - ball_pos;
        }

        private bool ClearancePossible()
        {
            if (CanKick())  // close enough to ball
            {
                bool res = false;
                Vector3 ballPos = ball.transform.position;

                List<Vector3> enemiesList = new List<Vector3>();
                List<Vector3> wallBottom = new List<Vector3>();
                List<Vector3> wallTop = new List<Vector3>();
                foreach (GameObject enemy in enemies) // look for smallest distance to the point
                {
                    Vector3 wallBottomPos = new Vector3(enemy.transform.position.x, 0, 55f);
                    wallBottom.Add(wallBottomPos);
                    Vector3 wallTopPos = new Vector3(enemy.transform.position.x, 0, 145f);
                    wallTop.Add(wallTopPos);
                    enemiesList.Add(enemy.transform.position);
                }

                float maxAngleBetween = 0;
                float maxForward = 0;

                for (int i = 0; i < 9; i++)
                {
                    Vector3 directionForShooting = ballPos;
                    if (i < 3)  // angle with 3 enemies
                    {
                        Vector3 enemy1 = enemiesList[i];
                        Vector3 enemy2 = enemiesList[(i + 1) % 3];

                        directionForShooting = 0.5f * (enemy1 - ballPos) + 0.5f * (enemy2 - ballPos);  // bisector

                        // if directionForShooting is collinear with third player, change to opposite
                        Vector3 dirThirdPlay = enemiesList[(i + 2) % 3] - ballPos;
                        if ((directionForShooting.x * dirThirdPlay.x + directionForShooting.z * dirThirdPlay.z) > 0)
                            directionForShooting = -directionForShooting;
                    }
                    else if (i >= 3 && i < 6)  // angle if shooting on the bootom wall 
                        directionForShooting = wallBottom[i % 3] - ballPos;
                    else
                        directionForShooting = wallTop[i % 3] - ballPos;



                    if ((areBlue && directionForShooting.x > 0) || (!areBlue && directionForShooting.x < 0))  // shoot in good direction
                    {
                        float minDirShootAngleBetween = 1000f;

                        foreach (GameObject enemy in enemies) // look for smallest distance to the point
                        {
                            float angleBetween = (float)Vector3.Angle(directionForShooting, enemy.transform.position - ballPos);  // angle dirShoot-ball-enemy
                            angleBetween = Mathf.Abs(convertToRadian(angleBetween));
                            if (angleBetween < minDirShootAngleBetween)
                                minDirShootAngleBetween = angleBetween;
                        }


                        if (minDirShootAngleBetween > 0.5f)
                        {
                            if (areBlue)
                            {
                                if (directionForShooting.x > maxForward)
                                {
                                    maxForward = directionForShooting.x;
                                    bestDirectionForShooting = directionForShooting;
                                    res = true;
                                }
                            }
                            else
                            {
                                if (-directionForShooting.x > maxForward)
                                {
                                    maxForward = -directionForShooting.x;
                                    bestDirectionForShooting = directionForShooting;
                                    res = true;
                                }
                            }

                        }
                    }
                }

                if (res)
                {
                    bestDirectionForShooting = maxKickSpeed * bestDirectionForShooting.normalized;
                    //Debug.Log("CTBAFS : bestDirectionForShooting = " + bestDirectionForShooting);
                    return true;
                }
            }
            return false;
        }

        private bool IsBallInAir()
        {
            Vector3 ballPos = ball.transform.position;
            return ballPos.y > ballInAirLimit;
        }


        private void PDcontroller(Vector3 targetPos)
        {
            targetPos = transform.position + targetPos;
            Vector3 targetVelocity = (targetPos - oldTargetPos) / Time.fixedDeltaTime;
            oldTargetPos = targetPos;
            Vector3 positionError = targetPos - transform.position;
            Vector3 velocityError = targetVelocity - my_rigidbody.velocity;
            Vector3 desiredAcceleration = kP * positionError + kD * velocityError;
            m_Drone.Move_vect(desiredAcceleration);
        }


        // ###############################
        // TASKS
        // ###############################

        [Task]
        bool AttackFormation()
        {
            if (IsCloserToBallThanEnemy() && NotInDefendPart())
                return true;
            return false;
        }

        [Task]
        void PrintIfCloserToBallThanEnemy(bool cond)
        {
            if (cond)
                Debug.Log(team + "closest to the ball");
            else
                Debug.Log(team + "not closer to the ball");
        }

        [Task]
        bool CurrentRobotclosest()
        {
            AttackRoleAssignment();  // define challenger, backup and solution (even if we need only the challenger)
            if (challenger && printAction)
                Debug.Log(team + "IsCloserToBallThanEnemy : true");
            return challenger;
        }

        [Task]
        bool AngleOnFreeGoal()
        {
            if (CanKick())
            {
                if (PossibilityShootOnGoal())
                    return true;
            }

            return false;
        }

        [Task]
        bool Shoot()
        {
            if (printAction)
                Debug.Log(team + "Shoot : true");
            //Vector3 direction = ball.transform.position - transform.position;
            //Vector3 velocity = maxKickSpeed * direction;
            Vector3 velocity = findBallVelocityToAdd(angleToShoot);
            Debug.DrawLine(ball.transform.position, ball.transform.position + velocity, Color.cyan);
            KickBall(velocity);
            return true;
        }


        [Task]
        bool EnemyArriving()
        {
            float min_dist_enemy = distanceEnemyToBall();
            if (min_dist_enemy < distanceEnemyArriving)
                return true;
            return false;
        }


        [Task]
        bool Challenge()
        {
            if (printAction)
            {
                Debug.Log(team + "Challenge : true");
                Vector3 tmp = new Vector3(transform.position.x + 5, transform.position.y, transform.position.z + 5);
                Debug.DrawLine(transform.position, tmp, Color.blue);
            }
            Vector3 ballPos = ball.transform.position;

            Vector3 ourGoalPos = own_goal.transform.position; // center of our goal
            // Go against ball, on axis own_goal/ball
            Vector3 axis = ourGoalPos - ballPos;
            axis = axis.normalized;

            Rigidbody rbBall = ball.GetComponent<Rigidbody>();
            Vector3 ballSpeed = rbBall.velocity;
            if (DirectedToOurGoal(ballSpeed))  // shooted towards our goal
            {
                if (areBlue)
                    axis = new Vector3(-1f, 0, 0);
                else // we are red
                    axis = new Vector3(1f, 0, 0);
            }

            if (distance(transform.position, ballPos) > distanceChallenge)  // far away from ball, take into account velocity
            {
                ballPos = ballPos + ballSpeed;
            }

            ballPos.x = ballPos.x + ballDiameter / 2 * axis.x;  // TODO: velocity -> goal: go_side
            ballPos.z = ballPos.z + ballDiameter / 2 * axis.z;

            Vector3 direction = ballPos - transform.position;
            PDcontroller(direction); // move the robot
            return true;
        }

        [Task]
        bool Dribble() // TODO: direction of dribbling, avoid enemies, etc
        {
            Vector3 ballPos = ball.transform.position;
            Vector3 EnemyGoalPos; // center of the enemy goal
            if (areBlue)
                EnemyGoalPos = terrain_manager.myInfo.goal_pos;
            else // we are red
                EnemyGoalPos = terrain_manager.myInfo.start_pos;

            if (distance(transform.position, ballPos) > distanceDribbling)  // far away from ball, take into account velocity
            {
                Rigidbody rbBall = ball.GetComponent<Rigidbody>();
                Vector3 ballSpeed = rbBall.velocity;
                ballPos = ballPos + ballSpeed;
            }

            // Go against ball, on axis own_goal/ball
            Vector3 axis = ballPos - EnemyGoalPos;
            axis = axis.normalized;
            ballPos.x = ballPos.x + ballDiameter / 2 * axis.x; // take into account size of drone ?!
            ballPos.z = ballPos.z + ballDiameter / 2 * axis.z;

            if (printAction)
                Debug.Log(team + "Dribble : true " + transform.position + "; " + ballPos);

            Vector3 direction = ballPos - transform.position;
            m_Drone.Move_vect(direction); // move the robot
                                          // PDcontroller(direction); // move the robot
            return true;
        }

        [Task]
        bool Backup()
        {
            return backup;
        }

        [Task]
        bool GoBackup()
        {
            if (printAction)
            {
                Debug.Log(team + "GoBackup : true");
                Vector3 tmp = new Vector3(transform.position.x + 5, transform.position.y, transform.position.z + 5);
                Debug.DrawLine(transform.position, tmp, Color.green);
            }
            Vector3 backupPosition = ball.transform.position;
            backupPosition.y = 0.01f;
            if (IsCloserToBallThanEnemy())  // attack
            {
                if (areBlue)
                    backupPosition.x = backupPosition.x - distanceBackupAttack;  // between our_goal and the ball
                else
                    backupPosition.x = backupPosition.x + distanceBackupAttack;  // between our_goal and the ball
            }
            else  // defend
            {
                if (areBlue)
                    backupPosition.x = backupPosition.x - distanceBackupDefend;  // between our_goal and the ball
                else
                    backupPosition.x = backupPosition.x + distanceBackupDefend;  // between our_goal and the ball
            }
            if (printPositionObj)
                drawPoint(backupPosition);
            Vector3 direction = backupPosition - transform.position;
            PDcontroller(direction); // move the robot
            return true;
        }

        [Task]
        bool GoSolution()
        {
            if (printAction) { 
                Debug.Log(team + "GoSolution : true");
                Vector3 tmp = new Vector3(transform.position.x + 5, transform.position.y, transform.position.z + 5);
                Debug.DrawLine(transform.position, tmp, Color.red);
            }
            Vector3 solutionPosition = ball.transform.position;
            solutionPosition.y = 0.01f;

            if (IsCloserToBallThanEnemy())  // attack
            {
                if (areBlue)
                    solutionPosition.x = solutionPosition.x - distanceSolutionAttackX;
                else
                    solutionPosition.x = solutionPosition.x + distanceSolutionAttackX;

                if (solutionPosition.z > 100) // top of the field
                    solutionPosition.z = solutionPosition.z - distanceSolutionAttackZ;
                else // bottom of the field
                    solutionPosition.z = solutionPosition.z + distanceSolutionAttackZ;
            }
            else  // defend
            {
                if (areBlue)
                    solutionPosition.x = solutionPosition.x - distanceSolutionDefendX;
                else
                    solutionPosition.x = solutionPosition.x + distanceSolutionDefendX;

                if (solutionPosition.z > 100) // top of the field
                    solutionPosition.z = solutionPosition.z - distanceSolutionDefendZ;
                else // bottom of the field
                    solutionPosition.z = solutionPosition.z + distanceSolutionDefendZ;
            }

            if (printPositionObj)
                drawPoint(solutionPosition);
            Vector3 direction = solutionPosition - transform.position;
            // m_Drone.Move_vect(direction); // move the robot
            PDcontroller(direction); // move the robot
            return true;
        }

        [Task]
        bool IsLastDefender()
        {
            //float xPos = transform.position.x;
            Vector3 ball_pos = ball.transform.position;
            float xPos = ball_pos.x;

            Vector3 ourGoalPos; // center of our goal
            if (areBlue)
            {
                ourGoalPos = terrain_manager.myInfo.start_pos;
                if ((ball_pos.x - ourGoalPos.x) < limitLastDefender)
                    return false;
            }
            else // we are red
            {
                ourGoalPos = terrain_manager.myInfo.goal_pos;
                if ((ourGoalPos.x - ball_pos.x) < limitLastDefender)
                    return false;
            }

            float dist = distance(transform.position, ball_pos);
            float min_dist = mindistanceFromRobotsOnGoodSideOfBall(ball_pos);  // TODO: onGoodSide ?!
            if (dist == min_dist) // closest defender
            {
                foreach (GameObject friend in friends) // look for smallest distance to the point
                {
                    float xPosFriend = friend.transform.position.x;
                    if (areBlue)
                    {
                        if (xPosFriend < xPos) // left of the closest robot to the ball
                            return false;
                    }
                    else
                    {
                        if (xPosFriend > xPos) // right of the closest robot to the ball
                            return false;
                    }
                }
                return true; // last defender
            }

            return false; // not closest defender
        }

        [Task]
        bool ShadowDefense()
        {
            if (printAction)
                Debug.Log(team + "ShadowDefense : true");
            Vector3 objective = ball.transform.position;

            Vector3 ourGoalPos; // center of our goal
            if (areBlue)
                ourGoalPos = terrain_manager.myInfo.start_pos;
            else // we are red
                ourGoalPos = terrain_manager.myInfo.goal_pos;

            // Go between ball and own_goal, on axis own_goal/ball
            float distBallToOwnGoal = distance(ourGoalPos, objective);
            Vector3 axis = ourGoalPos - objective;
            axis = axis.normalized;
            objective.x = objective.x + distBallToOwnGoal / 3 * axis.x;  // 1/3 from ball to our_goal
            objective.z = objective.z + distBallToOwnGoal / 3 * axis.z;

            Vector3 direction = objective - transform.position;
            // m_Drone.Move_vect(direction); // move the robot
            PDcontroller(direction); // move the robot
            return true;
        }

        [Task]
        bool InOurHalfField()
        // Return True if ball is in our half-field
        {
            float distOurGoal = (ball.transform.position - own_goal.transform.position).magnitude;
            float distOtherGoal = (ball.transform.position - other_goal.transform.position).magnitude;
            if (distOurGoal < distOtherGoal)
                return true;
            return false;
        }

        [Task]
        bool CloseToBallAndFreeSpace()  // return true if the challenger (defenser) can shoot away the ball
        {
            if (Mathf.Abs(own_goal.transform.position.x - ball.transform.position.x) < limitClearance)
            {
                if (ClearancePossible())
                    return true;
            }
            return false;
        }

        [Task]
        bool ShootClearance()
        {
            if (printAction)
                Debug.Log(team + "ShootClearance : true ; bestDirectionForShooting = " + bestDirectionForShooting);
            //Vector3 direction = ball.transform.position - transform.position;
            //Vector3 velocity = maxKickSpeed * direction;
            Vector3 velocity = findBallVelocityToAdd(bestDirectionForShooting);
            Debug.DrawLine(ball.transform.position, ball.transform.position + velocity, Color.magenta, 0.3f);
            KickBall(velocity);
            return true;
        }

        [Task]
        bool FrontPost()  // Return true if current agent should be at fronpost
        {
            CalculateFrontAndBackPost();
            float minDist = mindistanceFromRobotsToPoint(ball.transform.position);
            GameObject other = null;
            foreach (GameObject friend in friends)
            {
                if (friend.transform == transform)
                {
                    continue;
                }
                else if (Math.Abs((friend.transform.position - ball.transform.position).magnitude - minDist) == 0)
                {
                    continue;
                }
                other = friend;
            }
            // Calculate cost: -> backup / other -> solution
            float dist1 = Mathf.Max(distance(transform.position, frontPost), distance(other.transform.position, backPost));
            // Calculate cost: -> solution / other -> backup
            float dist2 = Mathf.Max(distance(transform.position, backPost), distance(other.transform.position, frontPost));

            if (dist1 < dist2)
                return true;
            else
                return false;
        }

        [Task]
        bool GoFrontPost()
        {
            Vector3 position = frontPost + (ball.transform.position - frontPost) * 0.33f;
            position.y = 0.01f;
            drawPoint(position);
            Vector3 direction = position - transform.position;
            // m_Drone.Move_vect(direction); // move the robot
            PDcontroller(direction); // move the robot
            return true;
        }

        [Task]
        bool RotateBackPost()
        {
            Vector3 position = backPost + (ball.transform.position - backPost) * 0.33f;
            position.y = 0.01f;
            if (printPositionObj)
                drawPoint(position);
            Vector3 direction = position - transform.position;
            // m_Drone.Move_vect(direction); // move the robot
            PDcontroller(direction); // move the robot
            return true;
        }
        
    }
}
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Swarm : MonoBehaviour
{
    public struct BBoid
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 velocity;
        public Vector3 alignment;
        public Vector3 cohesion;
        public Vector3 separation;
        public Vector3 obstacle;
        public Vector3 currentTotalForce;
    }

    public Transform boidPrefab;

    public int numberOfBoids = 200;

    public float boidForceScale = 20f;

    public float maxSpeed = 5.0f;

    public float rotationSpeed = 40.0f;

    public float obstacleCheckRadius = 1.0f;

    public float separationWeight = 1.1f;
    
    public float alignmentWeight = 0.5f;

    public float cohesionWeight = 1f;

    public float goalWeight = 1f;

    public float obstacleWeight = 0.9f;

    public float wanderWeight = 0.3f;

    public float neighbourDistance = 2.0f;

    public float initializationRadius = 1.0f;

    public float initializationForwardRandomRange = 50f;

    private BBoid[] boids;

    private Transform[] boidObjects;

    private float sqrNeighbourDistance;

    private Vector3 boidZeroGoal;
    private NavMeshPath boidZeroPath;
    private int currentCorner;
    private bool boidZeroNavigatingTowardGoal = false;


    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        sqrNeighbourDistance = neighbourDistance * neighbourDistance;
        boidZeroPath = new NavMeshPath();
        InitBoids();
        
    }

    /// <summary>
    /// Initialize the array of boids
    /// </summary>
    private void InitBoids()
    {
        // allocate arrays before use to avoid NullReferenceException
        boids = new BBoid[numberOfBoids];
        boidObjects = new Transform[numberOfBoids];

        for (int i = 0; i < numberOfBoids; i++)
        {
            BBoid boid = new BBoid();
            Transform boidObj = Instantiate(boidPrefab, new Vector3(this.transform.position.x +
                Random.Range(-initializationRadius, initializationRadius), this.transform.position.y +
                Random.Range(-initializationRadius, initializationRadius), this.transform.position.z +
                Random.Range(-initializationRadius, initializationRadius)), 
                Quaternion.Euler(0f,Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange),0f));

            boidObj.name = "Boid_" + i.ToString();
            boid.position = boidObj.position;
            boid.forward = boidObj.forward;
            boids[i] = boid;
            boidObjects[i] = boidObj;
        }
       
    }


    /// <summary>
    /// Reset the particle forces
    /// </summary>
    public void ResetBoidForces()
    {
        for(int i = 0; i < numberOfBoids; i++)
        {
            boids[i].currentTotalForce = Vector3.zero;
        }
        
    }


    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        ResetBoidForces();
        CalculateThreeRules();
        CalculateObstacleAvoidance();
        BoidZeroPathing();
        UpdateBoidPositions();
        
    }

    public void CalculateObstacleAvoidance()
    {
         //World bounds
        const float minX = -8f;
        const float maxX = 8f;
        const float minZ = -8f;
        const float maxZ = 8f;
        const float minY =  1f;
        const float maxY = 4f;
        LayerMask obstacleLayer = LayerMask.GetMask("Obstacle");

        for(int i = 0; i < numberOfBoids; i++)
        {
            //Obstacle avoidance
            Vector3 obstacleForce = Vector3.zero;
            Collider[] hitColliders = Physics.OverlapSphere(boids[i].position, obstacleCheckRadius);
            foreach (var hitCollider in hitColliders)
            {
                Vector3 distanceToObsatcle = boids[i].position - hitCollider.ClosestPoint(boids[i].position);
                float distanceMag = distanceToObsatcle.magnitude;
                if (distanceMag > 0f)
                {
                    obstacleForce += distanceToObsatcle.normalized;
                }
            }

            //World bounds avoidance
            if(boids[i].position.x < minX)
            {
                obstacleForce += new Vector3(1f,0f,0f);
            }
            else if(boids[i].position.x > maxX)
            {
                obstacleForce += new Vector3(-1f,0f,0f);
            }

            if(boids[i].position.y < minY)
            {
                obstacleForce += new Vector3(0f,1f,0f);
            }
            else if(boids[i].position.y > maxY)
            {
                obstacleForce += new Vector3(0f,-1f,0f);
            }

            if(boids[i].position.z < minZ)
            {
                obstacleForce += new Vector3(0f,0f,1f);
            }
            else if(boids[i].position.z > maxZ)
            {
                obstacleForce += new Vector3(0f,0f,-1f);
            }
            boids[i].obstacle = obstacleForce.normalized;
            boids[i].currentTotalForce += boids[i].obstacle * obstacleWeight * boidForceScale;
            
        }
    }
    public void BoidZeroPathing(){
        if(boidZeroNavigatingTowardGoal)
        {
          if(currentCorner < boidZeroPath.corners.Length)
          {
            Vector3 curPos = boids[0].position;
            Vector3 cornerPos = boidZeroPath.corners[currentCorner];
            Vector3 toCorner = cornerPos - curPos;
            if(toCorner.magnitude < 1.0f)
            {
                currentCorner++;
                if(currentCorner >= boidZeroPath.corners.Length)
                {
                    boidZeroNavigatingTowardGoal = false;
                    return;
                }
                cornerPos = boidZeroPath.corners[currentCorner];
                toCorner = cornerPos - curPos;
                if(toCorner.magnitude <= 0.0001f)
                {
                    boidZeroNavigatingTowardGoal = false;
                    return;
                }
            }
            Vector3 goalForce = toCorner.normalized;
            Vector3 steerForce = (goalForce * boidForceScale - boids[0].velocity) * goalWeight;
            boids[0].currentTotalForce += steerForce;
          }
          else
          {
              boidZeroNavigatingTowardGoal = false;
              return;
          }
           
        }

    }
    private void Update()
    {
        //Render information for boidzero, useful for debugging forces and path planning
        int boidCount = boids.Length;
        for (int i = 1; i < boidCount; i++)
        {
            Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
            if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance &&
                    Vector3.Dot(boidNeighbourVec, boids[0].forward) > 0f)
            { 
                Debug.DrawLine(boids[0].position, boids[i].position, Color.blue);
            }
        }
        
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].alignment, Color.green);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].separation, Color.magenta);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].cohesion, Color.yellow);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].obstacle, Color.red);

        if (boidZeroPath != null)
        {
            int cornersLength = boidZeroPath.corners.Length;
            for (int i = 0; i < cornersLength - 1; i++)
                Debug.DrawLine(boidZeroPath.corners[i], boidZeroPath.corners[i + 1], Color.black);
        }
        
    }

    public void CalculateThreeRules()
    {
        Vector3 vs = Vector3.zero;
        Vector3 vc = Vector3.zero;
        Vector3 va = Vector3.zero;
        float neighbourCount = 0f;
        float fov = Mathf.Cos(Mathf.PI/2);
        for(int i = 0; i < numberOfBoids; i++)
        {
              //Calulate Separtion, Cohesion, Alignment
            for (int j = 0; j < numberOfBoids; j++)
            {
                if (i != j)
                {
                    //Check if within neighbour distance
                   if(sqrNeighbourDistance > (boids[i].position - boids[j].position).sqrMagnitude && Vector3.Dot(boids[j].position - boids[i].position, boids[i].forward) >= fov)
                   {
                        neighbourCount++;
                        vs += boids[i].position - boids[j].position;
                        vc += boids[j].position;
                        va += boids[j].forward;
                   }
                }
                
            }
            if(neighbourCount == 0)
            {
                //Wander Rule, no neighbours
                boids[i].currentTotalForce = wanderWeight*(boids[i].velocity.normalized * boidForceScale - boids[i].velocity);
                continue;
            }
            // Average using the actual neighbour count (not the total number of boids)
            vs /= neighbourCount;
            vc /= neighbourCount;
            vc = vc - boids[i].position;
            va /= neighbourCount;

            boids[i].separation = vs;
            boids[i].cohesion = vc;
            boids[i].alignment = va;
            

            // (ω_k * ((rule_ki * α) - v_i))
            Vector3 separationForce = separationWeight*(vs.normalized * boidForceScale - boids[i].velocity);
            Vector3 cohesionForce = cohesionWeight*(vc.normalized * boidForceScale - boids[i].velocity);
            Vector3 alignmentForce = alignmentWeight*(va.normalized * boidForceScale - boids[i].velocity);
            boids[i].currentTotalForce += separationForce + cohesionForce + alignmentForce;
        
           
            neighbourCount = 0f;
            vs = Vector3.zero;

            vc = Vector3.zero;
            va = Vector3.zero;
        }   
    }

    public void UpdateBoidPositions()
    {
        for(int i = 0; i < numberOfBoids; i++)
        {
            //Update velocity
            boids[i].velocity += boids[i].currentTotalForce * Time.fixedDeltaTime;

            //Clamp speed
            if(boids[i].velocity.magnitude > maxSpeed)
            {
                boids[i].velocity = boids[i].velocity.normalized * maxSpeed;
            }

            //Update position
            boids[i].position += boids[i].velocity * Time.fixedDeltaTime;

            //Update forward
            if(boids[i].velocity.magnitude > 0.0001f)
            {
                boids[i].forward = Vector3.Slerp(boids[i].forward, boids[i].velocity.normalized,
                    rotationSpeed * Time.fixedDeltaTime).normalized;
            }

            //Update Transform
            boidObjects[i].position = boids[i].position;
            boidObjects[i].forward = boids[i].forward;
        }
    }

    
    public void SetGoal(Vector3 goal)
    {
      
        NavMeshHit goalhit;
        NavMeshHit boidHit;
    
        if(NavMesh.SamplePosition(goal, out goalhit, 5.0f, NavMesh.AllAreas) && NavMesh.SamplePosition(boids[0].position, out boidHit, 5.0f, NavMesh.AllAreas))
        {
            boidZeroGoal = goalhit.position;
            boidZeroPath = new NavMeshPath();
            NavMesh.CalculatePath(boidHit.position, boidZeroGoal, NavMesh.AllAreas, boidZeroPath);
            boidZeroNavigatingTowardGoal = true;
            currentCorner = 0;

        }
       
    }
}


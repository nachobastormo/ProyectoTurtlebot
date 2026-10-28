using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
 
public class TurtleBotAgent : Agent
{
    [Header("Referencias del Escenario")]
    public Transform startPoint;
    public Transform goalPoint;
 
    [Header("Parámetros de Movimiento")]
    public float moveSpeed = 2.0f;
    public float turnSpeed = 120f;
 
    [Header("Observaciones y arranque")]
    [Tooltip("Distancia máxima aproximada del mapa (para normalizar). Pon la diagonal del escenario.")]
    public float maxDistance = 30f;
    [Tooltip("Giro aleatorio al empezar cada episodio (grados a cada lado).")]
    public float randomStartYaw = 60f;
 
    private Rigidbody rb;
    private float previousDistanceToGoal;
    private bool resolved = true; // true = el episodio anterior terminó por meta o choque
 
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }
 
    public override void OnEpisodeBegin()
    {
        // Si el episodio anterior terminó por MaxStep (sin meta ni choque), cuenta como fallo
        if (!resolved)
        {
            Academy.Instance.StatsRecorder.Add("Fase2/LlegadaEsquina", 0f);
        }
        resolved = false;
 
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
 
        if (startPoint != null)
        {
            // Ligeramente elevado para que la gravedad lo asiente
            transform.position = startPoint.position + new Vector3(0, 0.05f, 0);
 
            // Giro inicial aleatorio: así cada episodio es distinto y el robot tiene que aprender a orientarse
            float yaw = Random.Range(-randomStartYaw, randomStartYaw);
            transform.rotation = startPoint.rotation * Quaternion.Euler(0f, yaw, 0f);
        }
 
        previousDistanceToGoal = PlanarDistanceToGoal();
    }
 
    private float PlanarDistanceToGoal()
    {
        if (goalPoint == null) return 0f;
        Vector3 delta = goalPoint.position - transform.position;
        delta.y = 0f;
        return delta.magnitude;
    }
 
    public override void CollectObservations(VectorSensor sensor)
    {
        // Siempre se añaden 4 valores (Space Size = 4 en Behavior Parameters)
        if (goalPoint != null)
        {
            // Dirección a la meta en el sistema LOCAL del robot:
            // x > 0 = meta a la derecha, z > 0 = meta delante
            Vector3 worldDelta = goalPoint.position - transform.position;
            worldDelta.y = 0f;
            float dist = worldDelta.magnitude;
 
            Vector3 localDir = transform.InverseTransformDirection(worldDelta);
            localDir.y = 0f;
            localDir = dist > 0.001f ? localDir.normalized : Vector3.zero;
 
            sensor.AddObservation(localDir.x);
            sensor.AddObservation(localDir.z);
            sensor.AddObservation(Mathf.Clamp01(dist / maxDistance));
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }
 
        // Velocidad hacia delante normalizada
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        sensor.AddObservation(localVel.z / moveSpeed);
    }
 
    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveInput = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float turnInput = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
 
        // 1. Rotación en el eje Y
        transform.Rotate(0f, turnInput * turnSpeed * Time.fixedDeltaTime, 0f);
 
        // 2. Movimiento lineal manteniendo la velocidad Y de la gravedad
        Vector3 forwardVelocity = transform.forward * moveInput * moveSpeed;
        rb.linearVelocity = new Vector3(forwardVelocity.x, rb.linearVelocity.y, forwardVelocity.z);
 
        // Recompensa por progreso hacia la meta
        if (goalPoint != null)
        {
            float currentDistance = PlanarDistanceToGoal();
            float distanceDelta = previousDistanceToGoal - currentDistance;
            AddReward(distanceDelta * 0.1f);
            previousDistanceToGoal = currentDistance;
        }
 
        // Penalización por girar sin avanzar
        if (Mathf.Abs(moveInput) < 0.15f && Mathf.Abs(turnInput) > 0.1f)
        {
            AddReward(-0.003f);
        }
 
        // Pequeño incentivo por avanzar
        if (moveInput > 0.2f)
        {
            AddReward(0.0005f);
        }
 
        // Penalización temporal por paso
        AddReward(-0.0005f);
    }
 
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
 
        float forward = 0f;
        float turn = 0f;
 
        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) forward = 1f;
        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) forward = -1f;
        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow)) turn = 1f;
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow)) turn = -1f;
 
        continuousActions[0] = forward;
        continuousActions[1] = turn;
    }
 
    private void OnCollisionEnter(Collision collision)
    {
        if (resolved) return;
 
        if (collision.gameObject.CompareTag("Obstacle"))
        {
            AddReward(-0.5f);
            Academy.Instance.StatsRecorder.Add("Fase2/LlegadaEsquina", 0f);
            resolved = true;
            EndEpisode();
        }
    }
 
    private void OnTriggerEnter(Collider other)
    {
        if (resolved) return;
 
        if (other.CompareTag("Goal"))
        {
            AddReward(2.0f);
            Academy.Instance.StatsRecorder.Add("Fase2/LlegadaEsquina", 1f);
            resolved = true;
            EndEpisode();
        }
    }
}
 
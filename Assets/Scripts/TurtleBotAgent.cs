using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// FASE 4: meta aleatoria uniforme + Ray Perception Sensor 3D (lidar).
/// - Observaciones propias: 4 (dirección local x, z a la meta, distancia normalizada, velocidad).
///   Space Size en Behavior Parameters = 4 (el Ray Perception Sensor añade las suyas aparte).
/// - La meta aparece en un punto uniforme del suelo, a una distancia entre min y max del robot.
///   Esas distancias se pueden controlar desde el YAML (curriculum).
/// - Estadísticas por episodio: Fase4/LlegadaMeta, Fase4/Choque, Fase4/Timeout (0 o 1).
/// </summary>
public class TurtleBotAgent : Agent
{
    [Header("Referencias del Escenario")]
    public Transform startPoint;
    public Transform goalPoint;
    [Tooltip("Warehouse_Floor: se usa para conocer los límites donde puede aparecer la meta.")]
    public Transform floor;

    [Header("Parámetros de Movimiento")]
    public float moveSpeed = 2.0f;
    public float turnSpeed = 120f;

    [Header("Observaciones y arranque")]
    [Tooltip("Distancia máxima aproximada del mapa (para normalizar). Pon la diagonal del escenario.")]
    public float maxDistance = 30f;
    [Tooltip("Giro aleatorio al empezar cada episodio (grados a cada lado).")]
    public float randomStartYaw = 60f;

    [Header("Meta aleatoria (valores por defecto; el curriculum del YAML los sobrescribe)")]
    public float minGoalDistance = 1.5f;
    public float maxGoalDistance = 3f;
    [Tooltip("Si está activo, la meta solo aparece con vista libre desde el robot (útil en las primeras fases).")]
    public bool requireLineOfSight = true;
    [Tooltip("Margen respecto a los bordes del suelo.")]
    public float wallMargin = 0.4f;
    [Tooltip("Radio libre de obstáculos alrededor de la meta.")]
    public float goalClearRadius = 0.3f;
    public string obstacleTag = "Obstacle";

    [Header("Recompensas")]
    public float progressReward = 0.1f;     // por metro que se acerca (negativo si se aleja)
    public float goalReward = 2.0f;
    public float collisionPenalty = 0.5f;
    public float stepPenalty = 0.0005f;
    public float spinPenalty = 0.003f;      // girar sin avanzar
    public float forwardBonus = 0.0005f;

    private Rigidbody rb;
    private Bounds floorBounds;
    private bool hasBounds;
    private float previousDistanceToGoal;
    private bool resolved = true; // true = el episodio anterior terminó por meta o choque

    // Historial de posiciones de la meta, solo para visualizarlas con gizmos en el editor
    private readonly List<Vector3> spawnHistory = new List<Vector3>();

    private enum EpisodeResult { Goal, Collision, Timeout }

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();

        if (floor != null)
        {
            Renderer r = floor.GetComponentInChildren<Renderer>();
            if (r != null)
            {
                floorBounds = r.bounds;
                hasBounds = true;
            }
            else
            {
                Collider c = floor.GetComponentInChildren<Collider>();
                if (c != null)
                {
                    floorBounds = c.bounds;
                    hasBounds = true;
                }
            }
        }
    }

    public override void OnEpisodeBegin()
    {
        // Si el episodio anterior terminó por MaxStep (sin meta ni choque), cuenta como timeout
        if (!resolved)
        {
            RecordResult(EpisodeResult.Timeout);
        }
        resolved = false;

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        if (startPoint != null)
        {
            transform.position = startPoint.position + new Vector3(0, 0.05f, 0);
            float yaw = Random.Range(-randomStartYaw, randomStartYaw);
            transform.rotation = startPoint.rotation * Quaternion.Euler(0f, yaw, 0f);
        }

        // Parámetros de curriculum (si no vienen del YAML, se usan los del Inspector)
        var env = Academy.Instance.EnvironmentParameters;
        float minD = env.GetWithDefault("goal_min_dist", minGoalDistance);
        float maxD = env.GetWithDefault("goal_max_dist", maxGoalDistance);
        bool needLos = env.GetWithDefault("goal_line_of_sight", requireLineOfSight ? 1f : 0f) > 0.5f;

        PlaceGoal(minD, maxD, needLos);

        previousDistanceToGoal = PlanarDistanceToGoal();
    }

    // ---------- Estadísticas ----------

    private void RecordResult(EpisodeResult result)
    {
        var stats = Academy.Instance.StatsRecorder;
        stats.Add("Fase4/LlegadaMeta", result == EpisodeResult.Goal ? 1f : 0f);
        stats.Add("Fase4/Choque", result == EpisodeResult.Collision ? 1f : 0f);
        stats.Add("Fase4/Timeout", result == EpisodeResult.Timeout ? 1f : 0f);
    }

    // ---------- Colocación de la meta ----------

    private void PlaceGoal(float minD, float maxD, bool needLineOfSight)
    {
        if (goalPoint == null) return;

        Vector3 origin = transform.position;
        float y = goalPoint.position.y;
        Vector3 originAtGoalHeight = new Vector3(origin.x, y, origin.z);

        // Zona de muestreo: cuadrado de lado 2*maxD alrededor del robot, recortado al suelo
        float xMin = origin.x - maxD, xMax = origin.x + maxD;
        float zMin = origin.z - maxD, zMax = origin.z + maxD;
        if (hasBounds)
        {
            xMin = Mathf.Max(xMin, floorBounds.min.x + wallMargin);
            xMax = Mathf.Min(xMax, floorBounds.max.x - wallMargin);
            zMin = Mathf.Max(zMin, floorBounds.min.z + wallMargin);
            zMax = Mathf.Min(zMax, floorBounds.max.z - wallMargin);
        }

        for (int i = 0; i < 100; i++)
        {
            Vector3 candidate = new Vector3(Random.Range(xMin, xMax), y, Random.Range(zMin, zMax));

            float d = new Vector2(candidate.x - origin.x, candidate.z - origin.z).magnitude;
            if (d < minD || d > maxD) continue;
            if (IsBlocked(candidate)) continue;
            if (needLineOfSight && HasObstacleBetween(originAtGoalHeight, candidate)) continue;

            spawnHistory.Add(candidate);
            if (spawnHistory.Count > 500) spawnHistory.RemoveAt(0);

            goalPoint.position = candidate;
            return;
        }

        Debug.LogWarning($"[{name}] No se encontró una posición válida para la meta; se mantiene la anterior.");
    }

    private bool IsBlocked(Vector3 point)
    {
        Collider[] hits = Physics.OverlapSphere(point, goalClearRadius, ~0, QueryTriggerInteraction.Ignore);
        foreach (Collider h in hits)
        {
            if (h.CompareTag(obstacleTag)) return true;
        }
        return false;
    }

    private bool HasObstacleBetween(Vector3 from, Vector3 to)
    {
        Vector3 dir = to - from;
        float len = dir.magnitude;
        if (len < 0.001f) return false;

        RaycastHit[] hits = Physics.RaycastAll(from, dir / len, len, ~0, QueryTriggerInteraction.Ignore);
        foreach (RaycastHit h in hits)
        {
            if (h.collider.CompareTag(obstacleTag)) return true;
        }
        return false;
    }

    private float PlanarDistanceToGoal()
    {
        if (goalPoint == null) return 0f;
        Vector3 delta = goalPoint.position - transform.position;
        delta.y = 0f;
        return delta.magnitude;
    }

    // ---------- Observaciones ----------

    public override void CollectObservations(VectorSensor sensor)
    {
        // Siempre se añaden 4 valores (Space Size = 4 en Behavior Parameters)
        if (goalPoint != null)
        {
            Vector3 worldDelta = goalPoint.position - transform.position;
            worldDelta.y = 0f;
            float dist = worldDelta.magnitude;

            Vector3 localDir = transform.InverseTransformDirection(worldDelta);
            localDir.y = 0f;
            localDir = dist > 0.001f ? localDir.normalized : Vector3.zero;

            sensor.AddObservation(localDir.x);   // x > 0 = meta a la derecha
            sensor.AddObservation(localDir.z);   // z > 0 = meta delante
            sensor.AddObservation(Mathf.Clamp01(dist / maxDistance));
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }

        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        sensor.AddObservation(localVel.z / moveSpeed);
    }

    // ---------- Acciones y recompensas ----------

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveInput = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float turnInput = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        transform.Rotate(0f, turnInput * turnSpeed * Time.fixedDeltaTime, 0f);

        Vector3 forwardVelocity = transform.forward * moveInput * moveSpeed;
        rb.linearVelocity = new Vector3(forwardVelocity.x, rb.linearVelocity.y, forwardVelocity.z);

        // Progreso hacia la meta: premio si se acerca, castigo si se aleja
        if (goalPoint != null)
        {
            float currentDistance = PlanarDistanceToGoal();
            float distanceDelta = previousDistanceToGoal - currentDistance;
            AddReward(distanceDelta * progressReward);
            previousDistanceToGoal = currentDistance;
        }

        // Girar sin avanzar
        if (Mathf.Abs(moveInput) < 0.15f && Mathf.Abs(turnInput) > 0.1f)
        {
            AddReward(-spinPenalty);
        }

        // Pequeño incentivo por avanzar
        if (moveInput > 0.2f)
        {
            AddReward(forwardBonus);
        }

        // Penalización temporal por paso
        AddReward(-stepPenalty);
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

    // ---------- Fin de episodio ----------

    private void OnCollisionEnter(Collision collision)
    {
        if (resolved) return;

        if (collision.gameObject.CompareTag(obstacleTag))
        {
            AddReward(-collisionPenalty);
            RecordResult(EpisodeResult.Collision);
            resolved = true;
            EndEpisode();
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (resolved) return;

        if (other.CompareTag("Goal"))
        {
            AddReward(goalReward);
            RecordResult(EpisodeResult.Goal);
            resolved = true;
            EndEpisode();
        }
    }

    // ---------- Depuración: puntos verdes con las últimas metas generadas ----------

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        foreach (Vector3 p in spawnHistory)
        {
            Gizmos.DrawSphere(p, 0.15f);
        }
    }
}
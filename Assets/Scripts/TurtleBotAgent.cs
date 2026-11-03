using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// FASE 5: meta aleatoria + lidar + palets que aparecen y desaparecen.
/// - Observaciones propias: 4 (igual que la Fase 4). Space Size = 4.
/// - En cada episodio se activan N palets al azar de los que cuelgan de "Obstacles"
///   (N viene del curriculum: parámetro 'num_obstacles').
/// - La meta solo aparece en zonas ALCANZABLES desde el punto de salida
///   (se calcula con un recorrido sobre una cuadrícula del suelo).
/// - Estadísticas por episodio: Fase5/LlegadaMeta, Fase5/Choque, Fase5/Timeout (0 o 1)
///   y Fase5/NumPalets (palets activos).
/// </summary>
public class TurtleBotAgent : Agent
{
    [Header("Referencias del Escenario")]
    public Transform startPoint;
    public Transform goalPoint;
    [Tooltip("Warehouse_Floor: límites del suelo donde puede aparecer la meta.")]
    public Transform floor;
    [Tooltip("Objeto 'Obstacles' con los palets. Si se deja vacío se busca como hermano del agente.")]
    public Transform obstaclesRoot;

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
    public float maxGoalDistance = 30f;
    [Tooltip("Si está activo, la meta solo aparece con vista libre desde el robot.")]
    public bool requireLineOfSight = false;
    [Tooltip("Margen respecto a los bordes del suelo.")]
    public float wallMargin = 0.4f;
    [Tooltip("Radio libre de obstáculos alrededor de la meta.")]
    public float goalClearRadius = 0.3f;
    public string obstacleTag = "Obstacle";

    [Header("Palets (obstáculos)")]
    [Tooltip("Palets activos por episodio si no hay curriculum (el YAML usa 'num_obstacles').")]
    public int numObstacles = 4;
    [Tooltip("Distancia mínima entre el punto de salida y cualquier palet activo (m).")]
    public float startClearance = 1.0f;

    [Header("Alcanzabilidad de la meta")]
    [Tooltip("Tamaño de celda de la cuadrícula (m).")]
    public float cellSize = 0.25f;
    [Tooltip("Radio del robot + margen (m). Las celdas más cerca que esto de un obstáculo se consideran cerradas.")]
    public float robotRadius = 0.2f;
    [Tooltip("Dibuja en azul las celdas alcanzables al seleccionar el agente (solo para depurar).")]
    public bool debugDrawReachable = false;

    [Header("Progreso por camino real")]
    [Tooltip("Si está activo, el premio de progreso usa la distancia por el camino libre (rodeando obstáculos) y no la distancia en línea recta.")]
    public bool useGeodesicProgress = true;
    [Tooltip("Escribe en la consola, al empezar cada episodio, la distancia en línea recta y por camino real (solo para comprobar).")]
    public bool debugLogPath = false;

    [Header("Recompensas")]
    public float progressReward = 0.1f;     // por metro que se acerca (negativo si se aleja)
    public float goalReward = 2.0f;
    public float collisionPenalty = 2.0f;
    public float timeoutPenalty = 2.0f;     // si se agota el tiempo sin llegar (que no compense quedarse parado)
    public float stepPenalty = 0.0005f;
    public float spinPenalty = 0.003f;      // girar sin avanzar
    public float forwardBonus = 0.0005f;

    // ---------- Estado interno ----------

    private class Pallet
    {
        public GameObject go;
        public BoxCollider col;
    }

    private Rigidbody rb;
    private Bounds floorBounds;
    private bool hasBounds;
    private float previousDistanceToGoal;
    private bool resolved = true; // true = el episodio anterior terminó por meta o choque
    private bool timeoutPenaltyApplied;

    private readonly List<Pallet> pallets = new List<Pallet>();
    private readonly List<int> order = new List<int>();

    // Cuadrícula de alcanzabilidad
    private bool gridReady;
    private int nx, nz;
    private Vector2 gridOrigin;
    private float gridY;
    private bool[] staticBlocked;
    private bool[] blocked;
    private bool[] reachable;
    private int[] queue;
    private readonly Collider[] overlapBuffer = new Collider[32];

    // Distancia por camino hasta la meta (se recalcula en cada episodio)
    private float[] goalDist;
    private bool[] inQueue;
    private int[] spfaQueue;
    private bool goalFieldReady;
    private float lastProgressDistance;

    // Historial de metas, solo para gizmos en el editor
    private readonly List<Vector3> spawnHistory = new List<Vector3>();

    private enum EpisodeResult { Goal, Collision, Timeout }

    // ---------- Inicialización ----------

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

        FindPallets();
        DeactivateAllPallets();
        BuildStaticGrid();
    }

    private void FindPallets()
    {
        if (obstaclesRoot == null && transform.parent != null)
        {
            obstaclesRoot = transform.parent.Find("Obstacles");
        }

        pallets.Clear();
        if (obstaclesRoot == null)
        {
            Debug.LogWarning($"[{name}] No se encontró el objeto 'Obstacles': no habrá palets.");
            return;
        }

        foreach (Transform t in obstaclesRoot)
        {
            BoxCollider c = t.GetComponent<BoxCollider>();
            if (c == null)
            {
                Debug.LogWarning($"[{name}] El palet '{t.name}' no tiene BoxCollider y se ignora.");
                continue;
            }
            pallets.Add(new Pallet { go = t.gameObject, col = c });
        }
    }

    private void DeactivateAllPallets()
    {
        foreach (Pallet p in pallets) p.go.SetActive(false);
    }

    // ---------- Episodio ----------

    public override void OnEpisodeBegin()
    {
        // Si el episodio anterior terminó por MaxStep (sin meta ni choque), cuenta como timeout
        if (!resolved)
        {
            RecordResult(EpisodeResult.Timeout);
        }
        resolved = false;
        timeoutPenaltyApplied = false;

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
        int requested = Mathf.RoundToInt(env.GetWithDefault("num_obstacles", numObstacles));

        int active = SetupObstacles(requested);
        RebuildReachability();

        bool placed = PlaceGoal(minD, maxD, needLos);
        if (!placed && active > 0)
        {
            // Sin sitio válido con estos palets: se repite el episodio sin obstáculos
            active = SetupObstacles(0);
            RebuildReachability();
            placed = PlaceGoal(minD, maxD, needLos);
        }
        if (!placed)
        {
            Debug.LogWarning($"[{name}] No se encontró una posición válida para la meta; se mantiene la anterior.");
        }

        Academy.Instance.StatsRecorder.Add("Fase5/NumPalets", active);

        // Distancia por camino real hasta la meta, con los palets de este episodio
        RebuildGoalField();
        lastProgressDistance = PlanarDistanceToGoal();
        previousDistanceToGoal = ProgressDistance();

        if (debugLogPath)
        {
            Debug.Log($"[{name}] Meta: línea recta {PlanarDistanceToGoal():F1} m, camino real {previousDistanceToGoal:F1} m, palets {active}");
        }
    }

    // ---------- Estadísticas ----------

    private void RecordResult(EpisodeResult result)
    {
        var stats = Academy.Instance.StatsRecorder;
        stats.Add("Fase5/LlegadaMeta", result == EpisodeResult.Goal ? 1f : 0f);
        stats.Add("Fase5/Choque", result == EpisodeResult.Collision ? 1f : 0f);
        stats.Add("Fase5/Timeout", result == EpisodeResult.Timeout ? 1f : 0f);
    }

    // ---------- Palets: activación aleatoria ----------

    /// <summary>Activa hasta 'requested' palets al azar, sin tocar el punto de salida. Devuelve cuántos quedaron activos.</summary>
    private int SetupObstacles(int requested)
    {
        DeactivateAllPallets();
        if (pallets.Count == 0 || requested <= 0) return 0;

        // Orden aleatorio de los palets (Fisher-Yates)
        order.Clear();
        for (int i = 0; i < pallets.Count; i++) order.Add(i);
        for (int i = order.Count - 1; i > 0; i--)
        {
            int j = Random.Range(0, i + 1);
            int tmp = order[i];
            order[i] = order[j];
            order[j] = tmp;
        }

        Vector3 pos = transform.position;
        int active = 0;
        foreach (int idx in order)
        {
            if (active >= requested) break;

            Pallet p = pallets[idx];
            if (PlanarDistanceToPallet(p, pos) < startClearance) continue; // demasiado cerca de la salida

            p.go.SetActive(true);
            active++;
        }
        return active;
    }

    /// <summary>Distancia en el plano XZ desde un punto al rectángulo del palet (0 si está dentro).</summary>
    private float PlanarDistanceToPallet(Pallet p, Vector3 worldPoint)
    {
        Vector3 local = p.col.transform.InverseTransformPoint(worldPoint) - p.col.center;
        Vector3 half = p.col.size * 0.5f;
        float dx = Mathf.Max(Mathf.Abs(local.x) - half.x, 0f);
        float dz = Mathf.Max(Mathf.Abs(local.z) - half.z, 0f);
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    private void PalletWorldBoundsXZ(Pallet p, out float minX, out float maxX, out float minZ, out float maxZ)
    {
        Vector3 h = p.col.size * 0.5f;
        minX = minZ = float.MaxValue;
        maxX = maxZ = float.MinValue;
        for (int sx = -1; sx <= 1; sx += 2)
        {
            for (int sz = -1; sz <= 1; sz += 2)
            {
                Vector3 w = p.col.transform.TransformPoint(p.col.center + new Vector3(sx * h.x, 0f, sz * h.z));
                minX = Mathf.Min(minX, w.x);
                maxX = Mathf.Max(maxX, w.x);
                minZ = Mathf.Min(minZ, w.z);
                maxZ = Mathf.Max(maxZ, w.z);
            }
        }
    }

    // ---------- Cuadrícula de alcanzabilidad ----------

    private Vector3 CellCenter(int ix, int iz, float y)
    {
        return new Vector3(gridOrigin.x + (ix + 0.5f) * cellSize, y, gridOrigin.y + (iz + 0.5f) * cellSize);
    }

    /// <summary>Se calcula una vez, con todos los palets desactivados: celdas ocupadas por paredes y estanterías.</summary>
    private void BuildStaticGrid()
    {
        gridReady = false;
        if (!hasBounds) return;

        nx = Mathf.Max(1, Mathf.CeilToInt(floorBounds.size.x / cellSize));
        nz = Mathf.Max(1, Mathf.CeilToInt(floorBounds.size.z / cellSize));
        gridOrigin = new Vector2(floorBounds.min.x, floorBounds.min.z);
        gridY = floorBounds.max.y + robotRadius + 0.05f;

        int n = nx * nz;
        staticBlocked = new bool[n];
        blocked = new bool[n];
        reachable = new bool[n];
        queue = new int[n];
        goalDist = new float[n];
        inQueue = new bool[n];
        spfaQueue = new int[n + 1];

        for (int iz = 0; iz < nz; iz++)
        {
            for (int ix = 0; ix < nx; ix++)
            {
                staticBlocked[ix + iz * nx] = IsStaticBlocked(CellCenter(ix, iz, gridY));
            }
        }
        gridReady = true;
    }

    private bool IsStaticBlocked(Vector3 center)
    {
        int count = Physics.OverlapSphereNonAlloc(center, robotRadius, overlapBuffer, ~0, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < count; i++)
        {
            if (overlapBuffer[i].CompareTag(obstacleTag)) return true;
        }
        return false;
    }

    /// <summary>Recalcula qué celdas son alcanzables desde la posición actual del robot, con los palets activos.</summary>
    private void RebuildReachability()
    {
        if (!gridReady) return;

        System.Array.Copy(staticBlocked, blocked, blocked.Length);

        // Añadir las celdas ocupadas por los palets activos
        foreach (Pallet p in pallets)
        {
            if (!p.go.activeSelf) continue;

            PalletWorldBoundsXZ(p, out float minX, out float maxX, out float minZ, out float maxZ);
            int ix0 = Mathf.Max(0, Mathf.FloorToInt((minX - robotRadius - gridOrigin.x) / cellSize));
            int ix1 = Mathf.Min(nx - 1, Mathf.FloorToInt((maxX + robotRadius - gridOrigin.x) / cellSize));
            int iz0 = Mathf.Max(0, Mathf.FloorToInt((minZ - robotRadius - gridOrigin.y) / cellSize));
            int iz1 = Mathf.Min(nz - 1, Mathf.FloorToInt((maxZ + robotRadius - gridOrigin.y) / cellSize));

            for (int iz = iz0; iz <= iz1; iz++)
            {
                for (int ix = ix0; ix <= ix1; ix++)
                {
                    if (PlanarDistanceToPallet(p, CellCenter(ix, iz, p.col.transform.position.y)) <= robotRadius)
                    {
                        blocked[ix + iz * nx] = true;
                    }
                }
            }
        }

        // Recorrido desde la posición del robot (la celda de salida se da siempre por libre)
        System.Array.Clear(reachable, 0, reachable.Length);
        Vector3 s = transform.position;
        int sx = Mathf.Clamp(Mathf.FloorToInt((s.x - gridOrigin.x) / cellSize), 0, nx - 1);
        int sz = Mathf.Clamp(Mathf.FloorToInt((s.z - gridOrigin.y) / cellSize), 0, nz - 1);

        int head = 0, tail = 0;
        int startCell = sx + sz * nx;
        reachable[startCell] = true;
        queue[tail++] = startCell;

        while (head < tail)
        {
            int cur = queue[head++];
            int cx = cur % nx;
            int cz = cur / nx;
            TryVisit(cx + 1, cz, ref tail);
            TryVisit(cx - 1, cz, ref tail);
            TryVisit(cx, cz + 1, ref tail);
            TryVisit(cx, cz - 1, ref tail);
        }
    }

    private void TryVisit(int x, int z, ref int tail)
    {
        if (x < 0 || x >= nx || z < 0 || z >= nz) return;
        int i = x + z * nx;
        if (blocked[i] || reachable[i]) return;
        reachable[i] = true;
        queue[tail++] = i;
    }

    private bool IsReachable(Vector3 p)
    {
        if (!gridReady) return true; // sin cuadrícula no se puede comprobar
        int ix = Mathf.FloorToInt((p.x - gridOrigin.x) / cellSize);
        int iz = Mathf.FloorToInt((p.z - gridOrigin.y) / cellSize);
        if (ix < 0 || ix >= nx || iz < 0 || iz >= nz) return false;
        return reachable[ix + iz * nx];
    }

    // ---------- Colocación de la meta ----------

    private bool PlaceGoal(float minD, float maxD, bool needLineOfSight)
    {
        if (goalPoint == null) return false;

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

        for (int i = 0; i < 150; i++)
        {
            Vector3 candidate = new Vector3(Random.Range(xMin, xMax), y, Random.Range(zMin, zMax));

            float d = new Vector2(candidate.x - origin.x, candidate.z - origin.z).magnitude;
            if (d < minD || d > maxD) continue;
            if (!IsReachable(candidate)) continue;
            if (IsBlocked(candidate)) continue;
            if (needLineOfSight && HasObstacleBetween(originAtGoalHeight, candidate)) continue;

            spawnHistory.Add(candidate);
            if (spawnHistory.Count > 500) spawnHistory.RemoveAt(0);

            goalPoint.position = candidate;
            return true;
        }

        return false;
    }

    private bool IsBlocked(Vector3 point)
    {
        int count = Physics.OverlapSphereNonAlloc(point, goalClearRadius, overlapBuffer, ~0, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < count; i++)
        {
            if (overlapBuffer[i].CompareTag(obstacleTag)) return true;
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

    // ---------- Distancia por camino real hasta la meta ----------

    /// <summary>
    /// Calcula, para cada celda libre, la longitud del camino más corto hasta la meta
    /// (8 vecinos, sin cortar esquinas), teniendo en cuenta los palets activos de este episodio.
    /// </summary>
    private void RebuildGoalField()
    {
        goalFieldReady = false;
        if (!useGeodesicProgress || !gridReady || goalPoint == null) return;

        int n = nx * nz;
        for (int i = 0; i < n; i++)
        {
            goalDist[i] = float.PositiveInfinity;
            inQueue[i] = false;
        }

        Vector3 g = goalPoint.position;
        int gx = Mathf.Clamp(Mathf.FloorToInt((g.x - gridOrigin.x) / cellSize), 0, nx - 1);
        int gz = Mathf.Clamp(Mathf.FloorToInt((g.z - gridOrigin.y) / cellSize), 0, nz - 1);
        int startCell = gx + gz * nx;

        int cap = n + 1;
        int head = 0, tail = 0;
        goalDist[startCell] = 0f;
        spfaQueue[tail] = startCell;
        tail = (tail + 1) % cap;
        inQueue[startCell] = true;

        float diagCost = cellSize * 1.4142f;
        while (head != tail)
        {
            int cur = spfaQueue[head];
            head = (head + 1) % cap;
            inQueue[cur] = false;

            int cx = cur % nx;
            int cz = cur / nx;
            float d = goalDist[cur];

            for (int dz = -1; dz <= 1; dz++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    if (dx == 0 && dz == 0) continue;

                    int x = cx + dx;
                    int z = cz + dz;
                    if (x < 0 || x >= nx || z < 0 || z >= nz) continue;

                    int j = x + z * nx;
                    if (blocked[j]) continue;

                    bool diagonal = dx != 0 && dz != 0;
                    if (diagonal && (blocked[x + cz * nx] || blocked[cx + z * nx])) continue;

                    float nd = d + (diagonal ? diagCost : cellSize);
                    if (nd < goalDist[j])
                    {
                        goalDist[j] = nd;
                        if (!inQueue[j])
                        {
                            spfaQueue[tail] = j;
                            tail = (tail + 1) % cap;
                            inQueue[j] = true;
                        }
                    }
                }
            }
        }

        goalFieldReady = true;
    }

    private float CellGoalDist(int x, int z)
    {
        if (x < 0 || x >= nx || z < 0 || z >= nz) return float.PositiveInfinity;
        return goalDist[x + z * nx];
    }

    /// <summary>
    /// Distancia hasta la meta usada para el premio de progreso: por camino real si hay campo calculado
    /// (interpolada entre las 4 celdas más cercanas) y en línea recta si no.
    /// </summary>
    private float ProgressDistance()
    {
        if (!goalFieldReady) return PlanarDistanceToGoal();

        Vector3 p = transform.position;
        float fx = (p.x - gridOrigin.x) / cellSize - 0.5f;
        float fz = (p.z - gridOrigin.y) / cellSize - 0.5f;
        int x0 = Mathf.FloorToInt(fx);
        int z0 = Mathf.FloorToInt(fz);
        float tx = fx - x0;
        float tz = fz - z0;

        // Interpolación bilineal usando solo las celdas que tienen camino
        float sum = 0f, wsum = 0f;
        for (int k = 0; k < 4; k++)
        {
            int dx = k & 1;
            int dz = k >> 1;
            float d = CellGoalDist(x0 + dx, z0 + dz);
            if (float.IsPositiveInfinity(d)) continue;

            float w = (dx == 0 ? 1f - tx : tx) * (dz == 0 ? 1f - tz : tz);
            if (w <= 0f) continue;
            sum += w * d;
            wsum += w;
        }
        if (wsum > 1e-4f)
        {
            lastProgressDistance = sum / wsum;
            return lastProgressDistance;
        }

        // Robot pegado a un obstáculo: celda cercana con camino + distancia en línea recta hasta ella
        float best = float.PositiveInfinity;
        for (int dz = -2; dz <= 3; dz++)
        {
            for (int dx = -2; dx <= 3; dx++)
            {
                float d = CellGoalDist(x0 + dx, z0 + dz);
                if (float.IsPositiveInfinity(d)) continue;

                Vector3 c = CellCenter(x0 + dx, z0 + dz, p.y);
                float e = new Vector2(c.x - p.x, c.z - p.z).magnitude;
                best = Mathf.Min(best, d + e);
            }
        }
        if (!float.IsPositiveInfinity(best)) lastProgressDistance = best;
        return lastProgressDistance;
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
            float currentDistance = ProgressDistance();
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

        // Penalización por agotar el tiempo sin llegar (se aplica una sola vez, al final del episodio).
        // Debe ser >= collisionPenalty para que quedarse parado nunca salga más barato que arriesgarse.
        if (!timeoutPenaltyApplied && MaxStep > 0 && StepCount >= MaxStep - 2)
        {
            AddReward(-timeoutPenalty);
            timeoutPenaltyApplied = true;
        }
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

    // ---------- Depuración ----------

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        foreach (Vector3 p in spawnHistory)
        {
            Gizmos.DrawSphere(p, 0.15f);
        }

        if (debugDrawReachable && gridReady)
        {
            Gizmos.color = new Color(0f, 0.6f, 1f, 0.35f);
            for (int iz = 0; iz < nz; iz++)
            {
                for (int ix = 0; ix < nx; ix++)
                {
                    if (reachable[ix + iz * nx])
                    {
                        Gizmos.DrawCube(CellCenter(ix, iz, floorBounds.max.y + 0.02f),
                            new Vector3(cellSize * 0.9f, 0.02f, cellSize * 0.9f));
                    }
                }
            }
        }
    }
}
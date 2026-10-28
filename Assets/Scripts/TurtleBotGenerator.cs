using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class TurtleBotGenerator : MonoBehaviour
{
    [ContextMenu("Generar Robot Dummy")]
    public void GenerarRobot()
    {
        // 1. Crear el objeto padre en el inicio
        GameObject robot = new GameObject("TurtleBot_Agent");
        robot.transform.position = new Vector3(0, 0.05f, 0); // Ajustado al nivel del suelo

        // 2. Físicas (Rigidbody y Collider único)
        Rigidbody rb = robot.AddComponent<Rigidbody>();
        rb.mass = 1.5f;
        rb.useGravity = true;
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        BoxCollider col = robot.AddComponent<BoxCollider>();
        col.size = new Vector3(0.3f, 0.2f, 0.3f);
        col.center = new Vector3(0, 0.1f, 0);

        // 3. Estructura visual (Primitivas)
        GameObject chasis = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        chasis.name = "Visual_Chasis";
        chasis.transform.SetParent(robot.transform);
        chasis.transform.localPosition = new Vector3(0, 0.1f, 0);
        chasis.transform.localScale = new Vector3(0.3f, 0.1f, 0.3f);
        DestroyImmediate(chasis.GetComponent<Collider>()); // Quitamos colisionador visual

        GameObject morro = GameObject.CreatePrimitive(PrimitiveType.Cube);
        morro.name = "Visual_Direccion";
        morro.transform.SetParent(robot.transform);
        morro.transform.localPosition = new Vector3(0, 0.2f, 0.1f);
        morro.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
        DestroyImmediate(morro.GetComponent<Collider>());

        // 4. Componentes de ML-Agents
        BehaviorParameters bp = robot.AddComponent<BehaviorParameters>();
        bp.BehaviorName = "TurtleBotLearning";
        bp.BrainParameters.ActionSpec = ActionSpec.MakeContinuous(2); // Adelante/Atrás y Giro

        robot.AddComponent<DecisionRequester>().DecisionPeriod = 5;

        RayPerceptionSensorComponent3D sensor = robot.AddComponent<RayPerceptionSensorComponent3D>();
        sensor.SensorName = "LidarSensor";
        sensor.RaysPerDirection = 3;
        sensor.MaxRayDegrees = 90;
        sensor.RayLength = 10f;
        sensor.SphereCastRadius = 0.05f;
        
        Debug.Log("TurtleBot Dummy generado con éxito. Recuerda añadir el Tag 'Obstacle' en el sensor.");
    }
}
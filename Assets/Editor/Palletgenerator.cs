using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Herramienta de editor: crea una reserva de palets (obstáculos) en cada TrainingArea.
/// Colocar en Assets/Editor/. Menú: Tools > TurtleBot > Generador de palets.
///
/// Cada área recibe un objeto "Obstacles" (hijo de la TrainingArea) con N palets desactivados.
/// El script del agente los activará y colocará en cada episodio.
/// Cada palet es un objeto con tag "Obstacle" y un único BoxCollider que cubre toda su altura.
/// </summary>
public class PalletGenerator : EditorWindow
{
    private const string RootName = "Obstacles";
    private const string ObstacleTag = "Obstacle";

    private int count = 8;
    private float length = 1.2f;      // largo (m)
    private float width = 0.8f;       // ancho (m)
    private float baseHeight = 0.15f; // altura del palet de madera (m)
    private float cargoHeight = 0.6f; // altura de la carga encima (m); 0 = solo el palet

    [MenuItem("Tools/TurtleBot/Generador de palets")]
    private static void Open()
    {
        GetWindow<PalletGenerator>("Palets");
    }

    private void OnGUI()
    {
        EditorGUILayout.LabelField("Palets por TrainingArea", EditorStyles.boldLabel);
        count = Mathf.Max(1, EditorGUILayout.IntField("Cantidad", count));
        length = Mathf.Max(0.1f, EditorGUILayout.FloatField("Largo (m)", length));
        width = Mathf.Max(0.1f, EditorGUILayout.FloatField("Ancho (m)", width));
        baseHeight = Mathf.Max(0.05f, EditorGUILayout.FloatField("Altura palet (m)", baseHeight));
        cargoHeight = Mathf.Max(0f, EditorGUILayout.FloatField("Altura carga (m)", cargoHeight));

        EditorGUILayout.Space();
        EditorGUILayout.HelpBox(
            "Los palets se crean DESACTIVADOS. Usa Mostrar para comprobar su tamaño en el mapa " +
            "y Ocultar antes de entrenar (el agente los activará solo).", MessageType.Info);

        if (GUILayout.Button("Crear / regenerar en todas las TrainingArea"))
        {
            GenerateAll();
        }

        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("Mostrar")) SetVisible(true);
        if (GUILayout.Button("Ocultar")) SetVisible(false);
        EditorGUILayout.EndHorizontal();
    }

    private static List<Transform> FindAreas()
    {
        var list = new List<Transform>();
        foreach (Transform t in FindObjectsByType<Transform>(FindObjectsSortMode.None))
        {
            if (t.name.StartsWith("TrainingArea")) list.Add(t);
        }
        return list;
    }

    private void GenerateAll()
    {
        Material wood = GetOrCreateMaterial("Pallet_Wood", new Color(0.72f, 0.52f, 0.30f));
        Material cargo = GetOrCreateMaterial("Pallet_Cargo", new Color(0.80f, 0.70f, 0.45f));

        List<Transform> areas = FindAreas();
        foreach (Transform area in areas)
        {
            Transform old = area.Find(RootName);
            if (old != null) Undo.DestroyObjectImmediate(old.gameObject);

            GameObject root = new GameObject(RootName);
            Undo.RegisterCreatedObjectUndo(root, "Crear palets");
            root.transform.SetParent(area, false);

            for (int i = 0; i < count; i++)
            {
                GameObject pallet = BuildPallet(i, wood, cargo);
                pallet.transform.SetParent(root.transform, false);
                // Posición provisional en fila; el agente los recoloca en cada episodio
                pallet.transform.localPosition = new Vector3((i - (count - 1) / 2f) * (length + 0.3f), 0f, 0f);
                pallet.SetActive(false);
            }
        }

        EditorSceneManager.MarkAllScenesDirty();
        Debug.Log($"[PalletGenerator] {count} palets creados en {areas.Count} TrainingArea.");
    }

    private GameObject BuildPallet(int index, Material wood, Material cargo)
    {
        GameObject pallet = new GameObject("Pallet_" + (index + 1));
        pallet.tag = ObstacleTag;

        float total = baseHeight + cargoHeight;
        BoxCollider col = pallet.AddComponent<BoxCollider>();
        col.center = new Vector3(0f, total / 2f, 0f);
        col.size = new Vector3(length, total, width);

        // Tablero superior
        AddPart(pallet.transform, "Deck", wood,
            new Vector3(0f, baseHeight - 0.0125f, 0f),
            new Vector3(length, 0.025f, width));

        // Tres largueros
        float runnerHeight = baseHeight - 0.025f;
        for (int k = -1; k <= 1; k++)
        {
            AddPart(pallet.transform, "Runner_" + (k + 2), wood,
                new Vector3(0f, runnerHeight / 2f, k * (width / 2f - 0.05f)),
                new Vector3(length, runnerHeight, 0.1f));
        }

        // Carga encima
        if (cargoHeight > 0.01f)
        {
            AddPart(pallet.transform, "Cargo", cargo,
                new Vector3(0f, baseHeight + cargoHeight / 2f, 0f),
                new Vector3(length * 0.95f, cargoHeight, width * 0.95f));
        }

        return pallet;
    }

    private static void AddPart(Transform parent, string partName, Material mat, Vector3 localPos, Vector3 size)
    {
        GameObject part = GameObject.CreatePrimitive(PrimitiveType.Cube);
        part.name = partName;
        DestroyImmediate(part.GetComponent<Collider>()); // solo visual: el collider está en la raíz
        part.transform.SetParent(parent, false);
        part.transform.localPosition = localPos;
        part.transform.localScale = size;
        part.GetComponent<Renderer>().sharedMaterial = mat;
    }

    private static Material GetOrCreateMaterial(string matName, Color color)
    {
        string path = "Assets/Materials/" + matName + ".mat";
        Material m = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (m != null) return m;

        if (!AssetDatabase.IsValidFolder("Assets/Materials"))
            AssetDatabase.CreateFolder("Assets", "Materials");

        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null) shader = Shader.Find("Standard");

        m = new Material(shader);
        m.color = color;
        AssetDatabase.CreateAsset(m, path);
        AssetDatabase.SaveAssets();
        return m;
    }

    private static void SetVisible(bool visible)
    {
        foreach (Transform area in FindAreas())
        {
            Transform root = area.Find(RootName);
            if (root == null) continue;

            foreach (Transform pallet in root)
            {
                Undo.RecordObject(pallet.gameObject, "Mostrar/ocultar palets");
                pallet.gameObject.SetActive(visible);
            }
        }
        EditorSceneManager.MarkAllScenesDirty();
    }
}
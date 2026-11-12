# Navegación Autónoma de un TurtleBot en un Almacén (Unity 6 + ML-Agents PPO)

**Autor:** Ignacio Bas Tormo  
**Titulación:** Grado en Ingeniería en Informática Industrial y Robótica (UPV)  
**Asignatura:** Modelado y Control de Robots  
**Vídeo de demostración:** [https://youtu.be/nRx628asviw](https://youtu.be/nRx628asviw)

---

## 1. Descripción del proyecto

Este repositorio contiene el entorno de simulación en **Unity 6** y los modelos entrenados mediante **Aprendizaje por Refuerzo Profundo (PPO)** con **Unity ML-Agents** para que un robot diferencial tipo **TurtleBot** aprenda a navegar de forma autónoma por un almacén industrial de $20 \times 20\text{ m}$ con estanterías y obstáculos variables (palets), apoyándose únicamente en un sensor **LiDAR 2D** ($25$ rayos) y la posición relativa de la meta en coordenadas polares, sin emplear ningún mapa global ni planificador de rutas en inferencia.

El aprendizaje se ha estructurado en **6 fases progresivas**:
1. **Fase 1 (`Fase1_meta_cerca.onnx`):** Meta fija a corta distancia ($\sim 3\text{ m}$) y almacén sin palets.
2. **Fase 2 (`Fase2_mitad.onnx`):** Meta fija en el centro del almacén ($\sim 8{,}5\text{ m}$).
3. **Fase 3 (`Fase3_esquina.onnx`):** Meta fija en la esquina opuesta ($\sim 16\text{ m}$) transfiriendo los pesos de la Fase 2.
4. **Fase 4 (`Fase4c_91.onnx`):** Meta en cualquier punto aleatorio del almacén mediante *Curriculum Learning* de distancia ($3 \rightarrow 6 \rightarrow 10 \rightarrow 16 \rightarrow 30\text{ m}$) y red neuronal ampliada a $256 \times 256$. Alcanza un **$93{,}3\,\%$ de éxito**.
5. **Fase 5 (`Fase5_palets_5M.onnx`):** Inclusión de hasta $8$ palets aleatorios por episodio que bloquean pasillos ($0 \rightarrow 2 \rightarrow 4 \rightarrow 6 \rightarrow 8$ palets) con recompensa basada en distancia euclídea.
6. **Fase 6 (`Fase6_ruta_real_3M.onnx`):** Sustitución del premio de avance en línea recta por la distancia a través del camino libre real (calculada solo durante el entrenamiento para moldear la recompensa), evitando mínimos locales tras los palets y alcanzando un **$87{,}1\,\%$ de éxito** con $8$ palets activos.

---

## 2. Estructura del repositorio

```text
ProyectoTurtlebot/
├── Assets/
│   ├── Scenes/
│   │   └── SampleScene.unity        # Escena principal con las 8 áreas del almacén
│   ├── Scripts/
│   │   ├── TurtleBotAgent.cs        # Agente ML-Agents (sensores, acciones, recompensas y currículum)
│   │   └── TurtleBotGenerator.cs    # Herramienta para generar el modelo físico del TurtleBot
│   ├── Editor/
│   │   └── Palletgenerator.cs       # Herramienta de editor para clonar palets en las 8 áreas
│   └── Models/                      # Modelos .onnx entrenados listos para inferencia en Unity
│       ├── Fase0_inicial.onnx
│       ├── Fase1_meta_cerca.onnx
│       ├── Fase2_mitad.onnx
│       ├── Fase3_esquina.onnx
│       ├── Fase4c_91.onnx
│       ├── Fase5_palets_5M.onnx
│       └── Fase6_ruta_real_3M.onnx
├── Modelos_pt/                      # Pesos de PyTorch (.pt) para reanudar entrenamientos (--initialize-from)
├── config/                          # Archivos YAML de hiperparámetros PPO y Curriculum Learning (fase0_1 a fase6)
└── resumen.py                       # Script en Python para extraer métricas de TensorBoard
```

---

## 3. Instrucciones de Setup y prueba rápida en Unity (Modo Inferencia)

No es necesario instalar Python ni entrenar ningún modelo para probar el proyecto, ya que todos los modelos `.onnx` están incluidos y asignados en la escena.

1. **Requisitos:**
   - **Unity Hub** y **Unity 6** (versión recomendada: `6000.3.25f1` o compatible de la rama `6000.x`).
2. **Clonar el repositorio:**
   ```bash
   git clone https://github.com/nachobastormo/ProyectoTurtlebot.git
   ```
3. **Abrir el proyecto en Unity:**
   - En **Unity Hub**, pulsar en **Add -> Add project from disk** y seleccionar la carpeta `ProyectoTurtlebot`.
   - La primera vez que se abra, Unity importará automáticamente los paquetes de `Packages/manifest.json` (`com.unity.ml-agents 4.1.0`, URP, Input System) y generará la carpeta `Library/`.
4. **Cargar la escena y ejecutar la demostración:**
   - En la ventana **Project**, abrir la escena `Assets/Scenes/SampleScene.unity`.
   - En la escena se encuentran las **8 áreas de almacén** (`Warehouse_Area_0` a `Warehouse_Area_7`), configuradas con los modelos `.onnx` de las distintas fases para poder comparar su comportamiento simultáneamente.
   - Pulsar el botón **Play (▶)** en la parte superior del editor para ver a los robots navegar en tiempo real.

### Cómo probar un modelo o escenario concreto en el Inspector

Si se desea cambiar el modelo o la dificultad de cualquiera de las áreas (`Warehouse_Area_X`):
1. Desplegar `Warehouse_Area_X` en la **Hierarchy** y seleccionar el objeto **`TurtleBot`**.
2. En el componente **`Behavior Parameters`**:
   - En **Model**, arrastrar cualquiera de los archivos de `Assets/Models/` (por ejemplo, `Fase6_ruta_real_3M.onnx` o `Fase4c_91.onnx`).
   - Asegurarse de que **Behavior Type** está en `Default` (o `Inference Only`).
3. En el componente **`Turtle Bot Agent (Script)`**:
   - **`Num Obstacles`**: Número de palets aleatorios activos por episodio (`0` para almacén despejado en Fases 1–4, `8` para el escenario con obstáculos de las Fases 5–6).
   - **`Min Goal Distance` / `Max Goal Distance`**: Rango de distancia a la que aparece la meta aleatoria (por ejemplo, `1.5` y `30` para todo el almacén). *Nota: si se desea dejar la meta fija en la posición manual de la escena para probar las Fases 1–3, basta con poner `Min Goal Distance = 100`*.
   - **`Require Line Of Sight`**: Desmarcado (`false`) a partir de la Fase 4 para permitir metas detrás de estanterías y en otros pasillos.

---

## 4. Configuración del entorno Python para reentrenar modelos (Opcional)

Si se desea reproducir los entrenamientos con `mlagents-learn` o ejecutar el script de análisis `resumen.py`:

1. **Crear y activar un entorno virtual con Python 3.10.11:**
   ```powershell
   python -m venv venv
   .\venv\Scripts\Activate.ps1
   ```
2. **Instalar dependencias:**
   ```powershell
   pip install mlagents==1.1.0 torch==2.2.2+cu121 --extra-index-url https://download.pytorch.org/whl/cu121
   pip install protobuf==3.20.3 onnx==1.15.0 tensorboard
   ```
3. **Lanzar un entrenamiento de ejemplo ( y pulsar Play en Unity cuando lo indique la consola):**
   ```powershell
   # Fases con mapa y meta fijos (Fases 1, 2 y 3)
   mlagents-learn config/fase0_1.yaml --run-id=Fase1_v2
   mlagents-learn config/fase2.yaml --run-id=Fase2_v3
   mlagents-learn config/fase3.yaml --run-id=Fase3_esquina --initialize-from=Fase2_v3

   # Fase 4: Meta aleatoria por todo el almacén con currículum de distancias
   mlagents-learn config/fase4.yaml --run-id=Fase4c_Desde0

   # Fase 5: Obstáculos dinámicos (0 a 8 palets aleatorios)
   mlagents-learn config/fase5.yaml --run-id=Fase5_Obstaculos --initialize-from=Fase4c_Desde0

   # Fase 6: Recompensa de avance por el camino libre real con 8 palets
   mlagents-learn config/fase6.yaml --run-id=Fase6_RutaReal --initialize-from=Fase5_Obstaculos
   ```
4. **Visualizar gráficas en TensorBoard:**
   ```powershell
   tensorboard --logdir results
   ```

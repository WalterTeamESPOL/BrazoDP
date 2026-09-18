# PruebaBrazo — RealSense + MediaPipe + transformación a SETP para Universal Robots

## 1. Objetivo del proyecto

El objetivo actual es obtener en tiempo real la posición 3D de la **muñeca derecha** de una persona usando una cámara **Intel RealSense** y **MediaPipe Pose**, transformar esa posición desde el sistema de coordenadas de la cámara al sistema de coordenadas de la base de un robot Universal Robots y generar una pose tipo:

```text
SETP = [X, Y, Z, RX, RY, RZ]
```

Por ahora **NO se está moviendo el robot**. La prioridad actual es comprobar que las coordenadas obtenidas por la cámara y la transformación cámara → base del robot sean correctas antes de implementar el control del brazo.

## 2. Estado actual

Actualmente el sistema ya hace lo siguiente:

```text
RealSense RGB + Depth
        ↓
MediaPipe Pose
        ↓
Muñeca derecha (landmark 16)
        ↓
Píxel (u, v)
        ↓
Profundidad RealSense
        ↓
Coordenadas 3D de cámara [Xc, Yc, Zc]
        ↓
Transformación T_BASE_CAMARA
        ↓
Coordenadas respecto a la base [X, Y, Z]
        ↓
Filtro EMA
        ↓
SETP [X, Y, Z, RX, RY, RZ]
```

En esta etapa el `SETP` solamente se muestra en pantalla y en terminal.

## 3. Archivos del proyecto

La carpeta de trabajo actual es:

```text
C:\Users\usuario\Documents\PruebaBrazo
```

La estructura prevista es:

```text
PruebaBrazo/
│
├── realsense_rtde_mediapipetransform.py
├── realsense_rtde_configuration.xml
├── robot_rtde.py                  ← pendiente
└── README.md
```

### `realsense_rtde_mediapipetransform.py`

Es el archivo principal actual. Se encarga de iniciar la RealSense, obtener RGB y profundidad, ejecutar MediaPipe Pose, localizar la muñeca derecha, obtener su profundidad, convertir el píxel de la muñeca a coordenadas 3D, transformar esas coordenadas al sistema de la base del robot, filtrar las coordenadas y construir el `SETP`.

### `realsense_rtde_configuration.xml`

Archivo reservado para la configuración RTDE. Se utilizará cuando se implemente la comunicación con el Universal Robot.

### `robot_rtde.py`

Todavía no está implementado. La idea es separar toda la lógica del robot del archivo de visión. Este archivo deberá encargarse de conexión RTDE, registros de entrada, watchdog, envío de setpoints, control continuo del robot y desconexión segura.

## 4. Librerías utilizadas

El proyecto utiliza:

```text
numpy
opencv-python
pyrealsense2
mediapipe
```

Para la parte futura del robot:

```text
UrRtde
```

La librería RTDE oficial estudiada es:

```text
UniversalRobots/RTDE_Python_Client_Library
```

## 5. Python y MediaPipe

Se encontró un problema con Python 3.13 y versiones recientes de MediaPipe:

```text
AttributeError: module 'mediapipe' has no attribute 'solutions'
```

El código actual utiliza la API clásica:

```python
mp.solutions.pose
```

Por eso se decidió utilizar:

```text
Python 3.12
MediaPipe 0.10.21
```

Instalación recomendada:

```powershell
python3.12 -m pip install mediapipe==0.10.21 opencv-python numpy pyrealsense2
```

Si se utiliza el ejecutable completo de Python 3.12:

```powershell
& "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe" -m pip install mediapipe==0.10.21 opencv-python numpy pyrealsense2
```

Para ejecutar:

```powershell
python3.12 .\realsense_rtde_mediapipetransform.py
```

## 6. RealSense

Configuración actual:

```python
ANCHO = 640
ALTO = 480
FPS = 30
```

La cámara entrega imagen RGB e imagen de profundidad. La profundidad se alinea con la imagen RGB mediante:

```python
align = rs.align(rs.stream.color)
```

Esto permite utilizar directamente el píxel detectado por MediaPipe sobre la imagen RGB para consultar la profundidad correspondiente.

## 7. MediaPipe Pose

Actualmente se utiliza:

```python
MUNECA_DER = 16
VISIBILIDAD_MIN = 0.50
```

El landmark 16 corresponde a la muñeca derecha. MediaPipe entrega coordenadas normalizadas y se convierten a píxeles mediante:

```text
u = x × ancho
v = y × alto
```

## 8. Obtención de profundidad

Actualmente se utiliza:

```python
RADIO_PROFUNDIDAD = 5
```

Esto crea una región de `11 × 11 píxeles` alrededor del landmark de la muñeca. Se recopilan profundidades válidas dentro de:

```python
PROFUNDIDAD_MIN_M = 0.15
PROFUNDIDAD_MAX_M = 3.00
```

y se utiliza la **mediana** para reducir errores aislados.

Después la RealSense realiza la deproyección:

```text
píxel (u,v) + profundidad
        ↓
[Xc, Yc, Zc]
```

Estas coordenadas están expresadas respecto al sistema de coordenadas de la cámara.

## 9. Transformación cámara → base del robot

La cámara y el robot no utilizan el mismo sistema de referencia. Por eso se utiliza una transformación homogénea:

```text
T_BASE_CAMARA
```

La relación es:

```text
P_base = T_BASE_CAMARA × P_camara
```

con:

```text
P_camara = [Xc, Yc, Zc, 1]
P_base   = [X, Y, Z, 1]
```

### Configuración actual del desplazamiento

```python
TX = -0.35
TY = -0.66
TZ = 0.0
```

Estos valores todavía deben ser calibrados y validados físicamente.

### Rotación cámara → base

```python
ROLL_DEG = 0.0
PITCH_DEG = 0.0
YAW_DEG = 0.0
```

La matriz de rotación se construye como:

```text
R = Rz × Ry × Rx
```

### Matriz manual

También existe la opción:

```python
USAR_MATRIZ_4X4_MANUAL = True
```

para introducir directamente una matriz 4×4 obtenida mediante calibración.

## 10. Coordenadas mostradas

El programa muestra tres niveles principales:

```text
MUÑECA CAM = [Xc, Yc, Zc]
BASE       = [X, Y, Z]
FILTRADO   = [Xf, Yf, Zf]
```

`BASE` es la coordenada más importante para validar la calibración geométrica.

## 11. Filtro EMA

Actualmente se utiliza:

```python
ALPHA_FILTRO = 0.50
```

El filtro calcula:

```text
valor_filtrado = ALPHA × valor_nuevo + (1 - ALPHA) × valor_anterior
```

Interpretación:

```text
ALPHA = 1.00 → sin filtrado
ALPHA = 0.50 → filtrado medio
ALPHA = 0.25 → filtrado más fuerte
```

Para calibrar la transformación puede usarse temporalmente `ALPHA_FILTRO = 1.0` para observar la transformación sin suavizado.

## 12. Rechazo de saltos

Inicialmente se implementó:

```python
MAX_SALTO_M = 0.15
```

Durante las pruebas apareció:

```text
SALTO RECHAZADO: 0.596 m
```

Esta condición complicaba la validación porque una lectura errónea podía dejar el sistema comparando contra una coordenada antigua. Por ese motivo el rechazo de saltos fue **desactivado y dejado comentado**.

Actualmente:

```text
NO se rechazan saltos por distancia.
```

Cuando se conecte el robot deberá implementarse una protección mejor, probablemente dentro de `robot_rtde.py`.

## 13. Límites del espacio de trabajo

El programa ya contiene límites:

```python
ACTIVAR_LIMITES = False

X_MIN_M = -1.0
X_MAX_M = 1.0
Y_MIN_M = -1.0
Y_MAX_M = 1.0
Z_MIN_M = 0.0
Z_MAX_M = 1.5
```

Actualmente están desactivados porque primero se está validando la transformación. Antes de mover el robot será necesario definir límites reales y seguros.

## 14. Generación del SETP

Una vez obtenidas las coordenadas filtradas se construye:

```text
SETP = [X, Y, Z, RX, RY, RZ]
```

Actualmente la orientación es fija:

```python
RX_FIJO = 0.0
RY_FIJO = 3.14159
RZ_FIJO = 0.0
```

Por tanto:

```text
X, Y, Z → cambian con la muñeca
RX, RY, RZ → permanecen constantes
```

## 15. Qué NO hace todavía el sistema

Actualmente el programa:

```text
NO mueve el robot.
NO ejecuta MoveL.
NO ejecuta MoveJ.
NO ejecuta servoJ.
NO calcula una trayectoria del robot.
NO tiene control en tiempo real del UR.
NO utiliza todavía watchdog para movimiento.
NO envía todavía el SETP al robot.
```

Esto es intencional: primero se está validando la geometría.

## 16. RTDE

Se estudió el repositorio oficial:

```text
UniversalRobots/RTDE_Python_Client_Library
```

RTDE utiliza normalmente:

```text
TCP 30004
```

La propuesta actual es enviar:

```text
input_double_register_24 → X
input_double_register_25 → Y
input_double_register_26 → Z
input_double_register_27 → RX
input_double_register_28 → RY
input_double_register_29 → RZ
```

La comunicación con el robot se implementará en un archivo separado.

## 17. Arquitectura prevista

```text
realsense_rtde_mediapipetransform.py
        ↓
produce SETP
        ↓
robot_rtde.py
        ↓
RTDE
        ↓
Universal Robot
```

Así cada archivo tendrá una responsabilidad clara.

# 18. Qué falta

## Etapa 1 — terminar calibración cámara → robot

Hay que verificar correctamente:

```text
TX
TY
TZ
ROLL_DEG
PITCH_DEG
YAW_DEG
```

Debe comprobarse físicamente que al mover la muñeca a derecha/izquierda, adelante/atrás y arriba/abajo, las coordenadas `BASE` cambien en el eje y signo esperados por el robot.

## Etapa 2 — validar puntos conocidos

Conviene utilizar varios puntos físicos conocidos y comparar:

```text
posición física esperada
vs.
posición BASE calculada
```

Esto permitirá medir el error de transformación.

## Etapa 3 — definir espacio seguro

Antes del movimiento hay que definir y activar:

```text
X_MIN / X_MAX
Y_MIN / Y_MAX
Z_MIN / Z_MAX
velocidad máxima
aceleración máxima
máximo desplazamiento entre objetivos
acción cuando se pierde MediaPipe
acción cuando la profundidad no es válida
acción cuando se pierde RTDE
```

## Etapa 4 — crear `robot_rtde.py`

Este archivo deberá incluir:

```text
conexión al robot
configuración RTDE
lectura del estado
registros 24–29
watchdog
recepción del SETP
validaciones finales
envío de datos
cierre seguro
```

## Etapa 5 — conectar ambos archivos

El archivo de visión deberá entregar el `SETP` al módulo del robot.

Conceptualmente:

```text
RealSense/MediaPipe
        ↓
SETP
        ↓
robot_rtde.py
```

## Etapa 6 — control continuo

El `example_control_loop.py` del repositorio oficial utiliza dos setpoints y movimientos discretos. Eso no es suficiente para seguir una muñeca en tiempo real.

Para seguimiento continuo habrá que implementar una estrategia adecuada, por ejemplo:

```text
SETP cartesiano
        ↓
get_inverse_kin(...)
        ↓
q objetivo
        ↓
servoj(...)
```

Esta parte todavía **NO está implementada**.

## Etapa 7 — seguridad y robustez

Antes de una prueba física con movimiento se deberá implementar:

```text
workspace limitado
watchdog
pérdida de cámara
pérdida de MediaPipe
profundidad inválida
saltos de coordenadas
limitación de velocidad
limitación de aceleración
parada segura
```

# 19. Orden recomendado de trabajo

```text
1. RealSense funcionando                         ✅
2. MediaPipe Pose funcionando                    ✅
3. Detectar muñeca derecha                       ✅
4. Obtener profundidad                           ✅
5. Obtener XYZ cámara                            ✅
6. Transformar cámara → base                     ✅ código
7. Calibrar T_BASE_CAMARA                        ⏳
8. Validar coordenadas BASE                      ⏳
9. Definir workspace seguro                      ⏳
10. Crear robot_rtde.py                          ⏳
11. Probar RTDE sin movimiento                   ⏳
12. Verificar registros recibidos por UR         ⏳
13. Implementar control del robot                ⏳
14. Añadir watchdog y protecciones               ⏳
15. Pruebas lentas y controladas                 ⏳
16. Seguimiento de muñeca en tiempo real         ⏳
```

# 20. Situación actual resumida

```text
Persona
  ↓
MediaPipe
  ↓
muñeca derecha
  ↓
RealSense
  ↓
XYZ cámara
  ↓
T_BASE_CAMARA
  ↓
XYZ base
  ↓
filtro
  ↓
SETP
  ↓
[LISTO PARA VALIDAR]
```

La siguiente prioridad es:

```text
CALIBRAR Y VALIDAR T_BASE_CAMARA
```

No conviene implementar el movimiento del robot hasta confirmar que las coordenadas `BASE` representan correctamente posiciones físicas en el sistema de referencia del Universal Robot.

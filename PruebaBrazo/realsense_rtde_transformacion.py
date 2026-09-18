import math
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


# ============================================================
# RTDE
# ============================================================

ENVIAR_RTDE = False

ROBOT_HOST = "192.168.0.10"
ROBOT_PORT = 30004
CONFIG_XML = Path(__file__).resolve().parent / "realsense_rtde_configuration.xml"


# ============================================================
# REALSENSE
# ============================================================

ANCHO = 640
ALTO = 480
FPS = 30

PROFUNDIDAD_MIN_M = 0.15
PROFUNDIDAD_MAX_M = 3.00
RADIO_PROFUNDIDAD = 2


# ============================================================
# TRANSFORMACIÓN CÁMARA -> BASE ROBOT
# ============================================================

USAR_MATRIZ_4X4_MANUAL = False

# Desplazamiento cámara respecto a base del robot [m]
TX = -0.35
TY = -0.66
TZ = 0.0

# Rotación cámara respecto a base [grados]
ROLL_DEG = 0.0
PITCH_DEG = 0.0
YAW_DEG = 0.0

# Matriz manual 4x4
T_BASE_CAMARA_MANUAL = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=float)


# ============================================================
# ORIENTACIÓN TCP
# ============================================================

RX_FIJO = 0.0
RY_FIJO = 3.14159
RZ_FIJO = 0.0


# ============================================================
# FILTRO
# ============================================================

ALPHA_FILTRO = 0.25
MAX_SALTO_M = 0.15


# ============================================================
# LÍMITES
# ============================================================

ACTIVAR_LIMITES = False

X_MIN_M = -1.0
X_MAX_M = 1.0

Y_MIN_M = -1.0
Y_MAX_M = 1.0

Z_MIN_M = 0.0
Z_MAX_M = 1.5


# ============================================================
# IMPRESIÓN
# ============================================================

INTERVALO_IMPRESION_S = 0.20

pixel_seleccionado = None


# ============================================================
# MATRICES DE ROTACIÓN
# ============================================================

def rotacion_x(angulo_rad):
    c = math.cos(angulo_rad)
    s = math.sin(angulo_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def rotacion_y(angulo_rad):
    c = math.cos(angulo_rad)
    s = math.sin(angulo_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rotacion_z(angulo_rad):
    c = math.cos(angulo_rad)
    s = math.sin(angulo_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


# ============================================================
# CONSTRUIR MATRIZ HOMOGÉNEA
# ============================================================

def construir_matriz_transformacion(tx, ty, tz, roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    rx = rotacion_x(roll)
    ry = rotacion_y(pitch)
    rz = rotacion_z(yaw)

    rotacion = rz @ ry @ rx

    transformacion = np.eye(4, dtype=float)
    transformacion[0:3, 0:3] = rotacion
    transformacion[0:3, 3] = [tx, ty, tz]

    return transformacion


# ============================================================
# OBTENER MATRIZ CÁMARA -> BASE
# ============================================================

def obtener_T_base_camara():
    if USAR_MATRIZ_4X4_MANUAL: return T_BASE_CAMARA_MANUAL.copy()
    return construir_matriz_transformacion(TX, TY, TZ, ROLL_DEG, PITCH_DEG, YAW_DEG)


# ============================================================
# TRANSFORMAR PUNTO
# ============================================================

def transformar_camara_a_base(punto_camara, T_base_camara):
    x_camara = punto_camara[0]
    y_camara = punto_camara[1]
    z_camara = punto_camara[2]

    punto_homogeneo = np.array([x_camara, y_camara, z_camara, 1.0], dtype=float)
    punto_base_h = T_base_camara @ punto_homogeneo

    return punto_base_h[0:3]


# ============================================================
# PROFUNDIDAD MEDIANA
# ============================================================

def obtener_profundidad_mediana(depth_frame, u, v):
    valores = []

    ancho = depth_frame.get_width()
    alto = depth_frame.get_height()

    for dy in range(-RADIO_PROFUNDIDAD, RADIO_PROFUNDIDAD + 1):
        for dx in range(-RADIO_PROFUNDIDAD, RADIO_PROFUNDIDAD + 1):
            px = u + dx
            py = v + dy

            if not (0 <= px < ancho and 0 <= py < alto): continue

            profundidad = depth_frame.get_distance(px, py)

            if PROFUNDIDAD_MIN_M <= profundidad <= PROFUNDIDAD_MAX_M: valores.append(profundidad)

    if len(valores) == 0: return None

    return float(np.median(valores))


# ============================================================
# LÍMITES
# ============================================================

def punto_dentro_limites(punto):
    if not ACTIVAR_LIMITES: return True

    x = punto[0]
    y = punto[1]
    z = punto[2]

    return X_MIN_M <= x <= X_MAX_M and Y_MIN_M <= y <= Y_MAX_M and Z_MIN_M <= z <= Z_MAX_M


# ============================================================
# FILTRO EMA
# ============================================================

def aplicar_filtro_ema(valor_nuevo, valor_anterior):
    if valor_anterior is None: return valor_nuevo.copy()
    return ALPHA_FILTRO * valor_nuevo + (1.0 - ALPHA_FILTRO) * valor_anterior


# ============================================================
# MOUSE
# ============================================================

def evento_mouse(evento, x, y, flags, param):
    global pixel_seleccionado

    if evento == cv2.EVENT_LBUTTONDOWN:
        pixel_seleccionado = (x, y)
        print()
        print("Nuevo pixel seleccionado:", pixel_seleccionado)


# ============================================================
# RTDE
# ============================================================

class ClienteRTDE:

    def __init__(self, host, port, config_xml):
        self.host = host
        self.port = port
        self.config_xml = config_xml
        self.con = None
        self.setp = None
        self.iniciado = False

    def conectar(self):
        try:
            import rtde.rtde as rtde
            import rtde.rtde_config as rtde_config
        except ImportError as exc:
            raise RuntimeError("No está instalada la librería RTDE.") from exc

        print()
        print("Conectando RTDE a:", self.host)

        conf = rtde_config.ConfigFile(str(self.config_xml))

        state_names, state_types = conf.get_recipe("state")
        setp_names, setp_types = conf.get_recipe("setp")

        self.con = rtde.RTDE(self.host, self.port)
        self.con.connect()

        version = self.con.get_controller_version()
        print("Versión controlador:", version)

        self.con.send_output_setup(state_names, state_types, frequency=30.0)
        self.setp = self.con.send_input_setup(setp_names, setp_types)

        if self.setp is None: raise RuntimeError("No se pudo configurar la receta RTDE 'setp'.")

        for registro in range(24, 30):
            setattr(self.setp, f"input_double_register_{registro}", 0.0)

        if not self.con.send_start(): raise RuntimeError("No se pudo iniciar la sincronización RTDE.")

        self.iniciado = True
        print("RTDE conectado correctamente.")

    def enviar_pose(self, pose):
        if not self.iniciado: return

        for i in range(6):
            registro = 24 + i
            setattr(self.setp, f"input_double_register_{registro}", float(pose[i]))

        self.con.send(self.setp)
        self.con.receive_buffered(buffer_limit=4096)

    def cerrar(self):
        if self.con is None: return

        try:
            if self.iniciado: self.con.send_pause()
        except Exception:
            pass

        try:
            self.con.disconnect()
        except Exception:
            pass

        print("RTDE desconectado.")


# ============================================================
# TEXTO
# ============================================================

def escribir_texto(imagen, texto, fila):
    y = 25 + fila * 25
    cv2.putText(imagen, texto, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)


# ============================================================
# MAIN
# ============================================================

def main():
    global pixel_seleccionado

    T_base_camara = obtener_T_base_camara()

    print()
    print("========================================")
    print("MATRIZ T_BASE_CAMARA")
    print("========================================")
    print(T_base_camara)
    print("========================================")
    print()

    if ENVIAR_RTDE: print("RTDE: ACTIVADO")
    else:
        print("RTDE: DESACTIVADO")
        print("Modo de validación: el programa NO enviará nada al robot.")

    cliente_rtde = None

    if ENVIAR_RTDE:
        cliente_rtde = ClienteRTDE(ROBOT_HOST, ROBOT_PORT, CONFIG_XML)
        cliente_rtde.conectar()

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, ANCHO, ALTO, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, ANCHO, ALTO, rs.format.bgr8, FPS)

    pipeline.start(config)

    align = rs.align(rs.stream.color)

    nombre_ventana = "RealSense - Transformacion Camara a Robot"

    cv2.namedWindow(nombre_ventana)
    cv2.setMouseCallback(nombre_ventana, evento_mouse)

    punto_base_filtrado = None
    ultimo_punto_base = None
    pixel_anterior = None
    ultimo_print = 0.0

    print()
    print("Haz CLICK sobre un punto de la imagen.")
    print("ESC = salir")
    print("R = reiniciar filtro")
    print()

    try:
        while True:

            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame: continue

            imagen = np.asanyarray(color_frame.get_data())

            if pixel_seleccionado != pixel_anterior:
                punto_base_filtrado = None
                ultimo_punto_base = None
                pixel_anterior = pixel_seleccionado

            if pixel_seleccionado is not None:

                u = int(pixel_seleccionado[0])
                v = int(pixel_seleccionado[1])

                cv2.circle(imagen, (u, v), 6, (0, 0, 255), -1)

                profundidad = obtener_profundidad_mediana(depth_frame, u, v)

                if profundidad is not None:

                    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

                    punto_camara = np.array(rs.rs2_deproject_pixel_to_point(intrinsics, [float(u), float(v)], profundidad), dtype=float)

                    punto_base = transformar_camara_a_base(punto_camara, T_base_camara)

                    salto_valido = True
                    distancia_salto = 0.0

                    if ultimo_punto_base is not None:
                        distancia_salto = float(np.linalg.norm(punto_base - ultimo_punto_base))
                        if distancia_salto > MAX_SALTO_M: salto_valido = False

                    if salto_valido:

                        ultimo_punto_base = punto_base.copy()
                        punto_base_filtrado = aplicar_filtro_ema(punto_base, punto_base_filtrado)

                        dentro_limites = punto_dentro_limites(punto_base_filtrado)

                        if dentro_limites:

                            x_robot = float(punto_base_filtrado[0])
                            y_robot = float(punto_base_filtrado[1])
                            z_robot = float(punto_base_filtrado[2])

                            pose = np.array([x_robot, y_robot, z_robot, RX_FIJO, RY_FIJO, RZ_FIJO], dtype=float)

                            if ENVIAR_RTDE and cliente_rtde is not None: cliente_rtde.enviar_pose(pose)

                            ahora = time.time()

                            if ahora - ultimo_print >= INTERVALO_IMPRESION_S:
                                #print("CAMARA:", np.round(punto_camara, 4), " -> BASE:", np.round(punto_base, 4), " -> FILTRADO:", np.round(punto_base_filtrado, 4), " -> SETP:", np.round(pose, 4))
                                print("SETP:", np.round(pose, 4))
                                ultimo_print = ahora

                            escribir_texto(imagen, f"Pixel: ({u}, {v})", 0)
                            escribir_texto(imagen, f"Depth: {profundidad:.3f} m", 1)
                            escribir_texto(imagen, f"CAM: X={punto_camara[0]:+.3f} Y={punto_camara[1]:+.3f} Z={punto_camara[2]:+.3f}", 2)
                            escribir_texto(imagen, f"BASE raw: X={punto_base[0]:+.3f} Y={punto_base[1]:+.3f} Z={punto_base[2]:+.3f}", 3)
                            escribir_texto(imagen, f"BASE filt: X={punto_base_filtrado[0]:+.3f} Y={punto_base_filtrado[1]:+.3f} Z={punto_base_filtrado[2]:+.3f}", 4)
                            escribir_texto(imagen, f"SETP: [{pose[0]:+.3f}, {pose[1]:+.3f}, {pose[2]:+.3f}, {pose[3]:+.3f}, {pose[4]:+.3f}, {pose[5]:+.3f}]", 5)

                            if ENVIAR_RTDE: escribir_texto(imagen, "RTDE: ENVIANDO registros 24-29", 6)
                            else: escribir_texto(imagen, "RTDE: DESACTIVADO", 6)

                        else:
                            escribir_texto(imagen, "PUNTO FUERA DE LIMITES", 5)

                    else:
                        escribir_texto(imagen, f"SALTO RECHAZADO: {distancia_salto:.3f} m", 5)

                else:
                    escribir_texto(imagen, "PROFUNDIDAD NO VALIDA", 1)

            else:
                escribir_texto(imagen, "Haz CLICK sobre un punto", 0)

            cv2.imshow(nombre_ventana, imagen)

            tecla = cv2.waitKey(1) & 0xFF

            if tecla == 27: break

            if tecla == ord("r"):
                punto_base_filtrado = None
                ultimo_punto_base = None
                print("Filtro reiniciado.")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

        if cliente_rtde is not None: cliente_rtde.cerrar()


# ============================================================
# EJECUCIÓN
# ============================================================

if __name__ == "__main__":
    main()
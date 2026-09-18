import math
import time

import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs


# ============================================================
# REALSENSE
# ============================================================

ANCHO = 640
ALTO = 480
FPS = 30

PROFUNDIDAD_MIN_M = 0.15
PROFUNDIDAD_MAX_M = 3.00
RADIO_PROFUNDIDAD = 5


# ============================================================
# MEDIAPIPE
# ============================================================

MUNECA_DER = 16
VISIBILIDAD_MIN = 0.50


# ============================================================
# TRANSFORMACIÓN CÁMARA -> BASE ROBOT
# ============================================================

USAR_MATRIZ_4X4_MANUAL = False

# Desplazamiento de la cámara respecto a la base del robot [m]
TX = -0.35
TY = -0.66
TZ = 0.0

# Rotación de la cámara respecto a la base [grados]
ROLL_DEG = 0.0
PITCH_DEG = 0.0
YAW_DEG = 0.0

# Matriz manual opcional
T_BASE_CAMARA_MANUAL = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=float)


# ============================================================
# ORIENTACIÓN DEL TCP PARA SETP
# ============================================================

RX_FIJO = 0.0
RY_FIJO = 3.14159
RZ_FIJO = 0.0


# ============================================================
# FILTRO
# ============================================================

# 1.00 = sin suavizado
# 0.50 = suavizado medio
# 0.25 = bastante suavizado
ALPHA_FILTRO = 0.50


# ============================================================
# RECHAZO DE SALTOS - DESACTIVADO POR AHORA
# ============================================================

# MAX_SALTO_M = 0.15


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
# CONSTRUIR MATRIZ DE TRANSFORMACIÓN
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
# TRANSFORMAR XYZ CÁMARA -> XYZ BASE
# ============================================================

def transformar_camara_a_base(punto_camara, T_base_camara):
    punto_h = np.array([punto_camara[0], punto_camara[1], punto_camara[2], 1.0], dtype=float)
    punto_base_h = T_base_camara @ punto_h
    return punto_base_h[0:3]


# ============================================================
# PROFUNDIDAD DE LA MUÑECA
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

    if len(valores) < 3: return None

    return float(np.median(valores))


# ============================================================
# FILTRO EMA
# ============================================================

def aplicar_filtro_ema(valor_nuevo, valor_anterior):
    if valor_anterior is None: return valor_nuevo.copy()
    return ALPHA_FILTRO * valor_nuevo + (1.0 - ALPHA_FILTRO) * valor_anterior


# ============================================================
# COMPROBAR LÍMITES
# ============================================================

def punto_dentro_limites(punto):
    if not ACTIVAR_LIMITES: return True

    x = punto[0]
    y = punto[1]
    z = punto[2]

    return X_MIN_M <= x <= X_MAX_M and Y_MIN_M <= y <= Y_MAX_M and Z_MIN_M <= z <= Z_MAX_M


# ============================================================
# TEXTO EN PANTALLA
# ============================================================

def escribir_texto(imagen, texto, fila):
    y = 25 + fila * 25
    cv2.putText(imagen, texto, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)


# ============================================================
# MAIN
# ============================================================

def main():
    T_base_camara = obtener_T_base_camara()

    print()
    print("========================================")
    print("T_BASE_CAMARA")
    print("========================================")
    print(T_base_camara)
    print("========================================")
    print()

    # ========================================================
    # REALSENSE
    # ========================================================

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, ANCHO, ALTO, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, ANCHO, ALTO, rs.format.bgr8, FPS)

    pipeline.start(config)

    # Profundidad alineada con RGB
    align = rs.align(rs.stream.color)

    # ========================================================
    # MEDIAPIPE POSE
    # ========================================================

    mp_pose = mp.solutions.pose

    pose_detector = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        smooth_landmarks=True,
        enable_segmentation=False,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    # ========================================================
    # VARIABLES
    # ========================================================

    punto_base_filtrado = None
    ultimo_punto_base = None
    ultimo_print = 0.0

    try:
        while True:

            # =================================================
            # REALSENSE
            # =================================================

            frames = pipeline.wait_for_frames()
            frames_alineados = align.process(frames)

            depth_frame = frames_alineados.get_depth_frame()
            color_frame = frames_alineados.get_color_frame()

            if not depth_frame or not color_frame: continue

            imagen = np.asanyarray(color_frame.get_data())

            # =================================================
            # MEDIAPIPE
            # =================================================

            rgb = cv2.cvtColor(imagen, cv2.COLOR_BGR2RGB)
            resultados = pose_detector.process(rgb)

            if resultados.pose_landmarks:

                muñeca = resultados.pose_landmarks.landmark[MUNECA_DER]

                if muñeca.visibility >= VISIBILIDAD_MIN:

                    ancho_imagen = imagen.shape[1]
                    alto_imagen = imagen.shape[0]

                    u = int(muñeca.x * ancho_imagen)
                    v = int(muñeca.y * alto_imagen)

                    if 0 <= u < ancho_imagen and 0 <= v < alto_imagen:

                        # Punto rojo sobre la muñeca
                        cv2.circle(imagen, (u, v), 7, (0, 0, 255), -1)

                        # =====================================
                        # PROFUNDIDAD REALSENSE
                        # =====================================

                        profundidad = obtener_profundidad_mediana(depth_frame, u, v)

                        if profundidad is not None:

                            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

                            # =================================
                            # PIXEL + DEPTH -> XYZ CÁMARA
                            # =================================

                            punto_camara = np.array(rs.rs2_deproject_pixel_to_point(intrinsics, [float(u), float(v)], profundidad), dtype=float)

                            # =================================
                            # XYZ CÁMARA -> XYZ BASE ROBOT
                            # =================================

                            punto_base = transformar_camara_a_base(punto_camara, T_base_camara)


                            # ============================================================
                            # RECHAZO DE SALTOS - DESACTIVADO
                            # ============================================================
                            #
                            # Esta protección se deja comentada por ahora porque estamos
                            # comprobando las coordenadas de la cámara y la transformación.
                            #
                            # Más adelante, cuando conectemos el robot, podemos recuperar
                            # esta protección o implementar una mejor.
                            #
                            # salto_valido = True
                            # distancia_salto = 0.0
                            #
                            # if ultimo_punto_base is not None:
                            #     distancia_salto = float(np.linalg.norm(punto_base - ultimo_punto_base))
                            #
                            #     if distancia_salto > MAX_SALTO_M:
                            #         salto_valido = False
                            #
                            # if not salto_valido:
                            #     escribir_texto(imagen, f"SALTO RECHAZADO: {distancia_salto:.3f} m", 5)
                            #     cv2.imshow("RealSense + MediaPipe - Muneca derecha", imagen)
                            #
                            #     tecla = cv2.waitKey(1) & 0xFF
                            #
                            #     if tecla == 27: break
                            #
                            #     continue
                            #
                            # ============================================================


                            # =================================
                            # GUARDAR ÚLTIMO PUNTO
                            # =================================

                            ultimo_punto_base = punto_base.copy()


                            # =================================
                            # FILTRAR XYZ DE LA BASE
                            # =================================

                            punto_base_filtrado = aplicar_filtro_ema(punto_base, punto_base_filtrado)


                            # =================================
                            # COMPROBAR LÍMITES
                            # =================================

                            if punto_dentro_limites(punto_base_filtrado):

                                x_robot = float(punto_base_filtrado[0])
                                y_robot = float(punto_base_filtrado[1])
                                z_robot = float(punto_base_filtrado[2])

                                # =============================
                                # CREAR SETP
                                # =============================

                                setp = np.array([x_robot, y_robot, z_robot, RX_FIJO, RY_FIJO, RZ_FIJO], dtype=float)


                                # =============================
                                # MOSTRAR EN TERMINAL
                                # =============================

                                ahora = time.time()

                                if ahora - ultimo_print >= INTERVALO_IMPRESION_S:
                                    print("MUÑECA CAM:", np.round(punto_camara, 4), " -> BASE:", np.round(punto_base, 4), " -> FILTRADO:", np.round(punto_base_filtrado, 4), " -> SETP:", np.round(setp, 4))
                                    ultimo_print = ahora


                                # =============================
                                # MOSTRAR EN PANTALLA
                                # =============================

                                escribir_texto(imagen, f"Muneca derecha pixel: ({u}, {v})", 0)
                                escribir_texto(imagen, f"Visibility: {muñeca.visibility:.2f}", 1)
                                escribir_texto(imagen, f"Depth: {profundidad:.3f} m", 2)
                                escribir_texto(imagen, f"CAM: X={punto_camara[0]:+.3f} Y={punto_camara[1]:+.3f} Z={punto_camara[2]:+.3f}", 3)
                                escribir_texto(imagen, f"BASE: X={punto_base[0]:+.3f} Y={punto_base[1]:+.3f} Z={punto_base[2]:+.3f}", 4)
                                escribir_texto(imagen, f"FILT: X={punto_base_filtrado[0]:+.3f} Y={punto_base_filtrado[1]:+.3f} Z={punto_base_filtrado[2]:+.3f}", 5)
                                escribir_texto(imagen, f"SETP: [{setp[0]:+.3f}, {setp[1]:+.3f}, {setp[2]:+.3f}, {setp[3]:+.3f}, {setp[4]:+.3f}, {setp[5]:+.3f}]", 6)

                            else:
                                escribir_texto(imagen, "PUNTO FUERA DE LIMITES", 5)

                        else:
                            escribir_texto(imagen, "PROFUNDIDAD MUNECA NO VALIDA", 2)

                else:
                    escribir_texto(imagen, f"MUNECA NO VISIBLE: {muñeca.visibility:.2f}", 0)

            else:
                punto_base_filtrado = None
                ultimo_punto_base = None
                escribir_texto(imagen, "MEDIAPIPE: PERSONA NO DETECTADA", 0)


            # =================================================
            # VENTANA
            # =================================================

            cv2.imshow("RealSense + MediaPipe - Muneca derecha", imagen)

            tecla = cv2.waitKey(1) & 0xFF

            if tecla == 27: break

            if tecla == ord("r"):
                punto_base_filtrado = None
                ultimo_punto_base = None
                print("Filtro reiniciado.")

    finally:
        pose_detector.close()
        pipeline.stop()
        cv2.destroyAllWindows()


# ============================================================
# EJECUCIÓN
# ============================================================

if __name__ == "__main__":
    main()
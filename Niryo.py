import time,json,serial,cv2
import numpy as np
import paho.mqtt.client as mqtt
from pyniryo import NiryoRobot

# ==========================================
# 1. CONFIGURACIÓN Y VARIABLES
# ==========================================
robot_ip = "200.126.13.186"
robot = NiryoRobot(robot_ip)
heltec = serial.Serial('COM3', 115200, timeout=1)

# Configuración MQTT para Ignition
MQTT_BROKER = "localhost"        
MQTT_PORT = 1883
MQTT_USER = "admin"
MQTT_PASS = "admin123"

TOPIC_ESTADO = "fabrica/linea1/estado"
TOPIC_COMANDOS = "fabrica/linea1/comando"

sistema_activo = False

cont_small = 0
cont_large = 0

# Poses
home = [0, 0, 0, 0, 0, 0]
aproxpick = [0.044, -0.215, -0.585, 0.167, -0.695, 0.005]
pick = [0.006, -0.743, -0.431, 0.192, -0.494, -0.005]
repisa_alta = [0.839, -0.227, 0.054, 0.141, -1.121, -1.411]
repisa_baja = [0.906, -0.757, -0.497, 0.166, 0.594, -0.250]


# 2. FUNCIONES DEL ROBOT

def home_position():
    print("[ROBOT] Volviendo a la posicion de home...")
    robot.move_joints(home)

def alta():
    print("[ROBOT] Yendo a repisa ALTA...")
    robot.move_joints(*repisa_alta)

def baja():
    print("[ROBOT] Yendo a repisa BAJA...")
    robot.move_joints(*repisa_baja)

def tomar_caja():
    print("[ROBOT] Acercandose a la caja...")
    robot.move_joints(*aproxpick)
    time.sleep(0.5)
    robot.move_joints(*pick)
    time.sleep(0.5)

    print("[ROBOT] Agarrando...")
    try:
        robot.close_gripper(500)
    except Exception as e:
        print(f"[Aviso Gripper]: {e}")
       
    time.sleep(0.5)
    print("[ROBOT] Elevando la caja...")
    robot.move_joints(*aproxpick)
    time.sleep(0.5)

def soltar_caja():
    print("[ROBOT] Soltando caja...")
    try:
        robot.open_gripper(500)
    except Exception as e:
        print(f"[Aviso Gripper]: {e}")
    time.sleep(0.5)


# 3. VISIÓN Y HARDWARE (HELTEC)
# ==========================================
def clasificacion_cajas(captura):
    ret, frame = captura.read()
    if not ret or frame is None:
        return "NONE", 0.0

    # Escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Segmentacion
    _, thresh = cv2.threshold(blurred, 120, 255, cv2.THRESH_BINARY_INV)

    # Encontrar contornos
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = [c for c in contours if cv2.contourArea(c) > 100]

    if not valid_contours:
        return "NONE", 0.0

    # Seleccionar contorno principal
    main_contour = max(valid_contours, key=cv2.contourArea)
    area = cv2.contourArea(main_contour)
    
    print(f"[OpenCV] Área de la caja detectada: {area:.1f} píxeles")

    UMBRAL_CORTE = 3000  
    categoria = "SMALL" if area < UMBRAL_CORTE else "LARGE"
    return categoria, area

def banda_transportadora():
    print("[BANDA] Iniciando la banda transportadora...")
    heltec.flushInput()
    heltec.write(b'START\n')
    
    while sistema_activo:
        if heltec.in_waiting > 0:
            respuesta = heltec.readline().decode('utf-8').strip()
            if respuesta == "caja_detectada":
                print("[BANDA] Caja detectada en la banda transportadora")
                return True
        time.sleep(0.02)
    return False


# 4. COMUNICACIÓN MQTT (IGNITION)
# ==========================================
def coneccion_mqtt(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] Conexión MQTT exitosa con Ignition")
        client.subscribe(TOPIC_COMANDOS)
    else:
        print(f"[MQTT ERROR] Error de conexión MQTT: {rc}")

def on_message(client, userdata, msg):
    global sistema_activo
    payload = msg.payload.decode('utf-8').strip()
    print(f"[MQTT ◄ HMI] Comando recibido desde Ignition: {payload}")
    
    if payload == "START":
        sistema_activo = True
    elif payload == "STOP":
        sistema_activo = False

mqtt_client = mqtt.Client(client_id="Python_Niryo_Controller")
mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
mqtt_client.on_connect = coneccion_mqtt
mqtt_client.on_message = on_message

try:
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start() 
except Exception as e:
    print(f"[MQTT ERROR] No se pudo conectar a Ignition: {e}")

def publicar_datos(banda, caja, area_px, robot_estado):
    """ Envia el estado actual del proceso a los Tags de Ignition """
    payload = {
        "banda": banda,              
        "caja": caja,                 
        "area_px": round(area_px, 1), 
        "robot": robot_estado,
        "cont_small": cont_small,
        "cont_large": cont_large,       
    }
    mqtt_client.publish(TOPIC_ESTADO, json.dumps(payload))


# 5. BUCLE PRINCIPAL
# ==========================================
def main():
    print("Iniciando el programa... ")
    captura = cv2.VideoCapture(0)

    # Asegurar modo automático en el robot
    robot.set_learning_mode(False)

    # Mover el robot a la posición de inicio
    home_position()

    print("Esperando a que el sistema se active desde Ignition...")

    while True:
        # Bucle de espera si está detenido en Ignition
        if not sistema_activo:
            publicar_datos("Stopped", "Ninguna", 0.0, "Standby")
            time.sleep(0.5)
            continue  # Ahora sí está bien identado dentro del if

        # 1. Encender banda y esperar llegada
        publicar_datos("Running", "Ninguna", 0.0, "Esperando caja")
        caja_en_posicion = banda_transportadora()

        if not caja_en_posicion or not sistema_activo:
            heltec.write(b'STOP\n')
            continue

        # 2. Clasificar la caja usando la cámara
        categoria, area_px = clasificacion_cajas(captura)
        publicar_datos("Stopped", categoria, area_px, "Inspeccionando")

        # 3. Mover robot según la clasificación
        if categoria == "SMALL":    
            print("[PROCESO] Caja PEQUEÑA detectada")
            cont_small += 1  # Incrementamos contador de cajas pequeñas
            print(f"[CONTEO] Caja pequeña clasificada. Total: {cont_small}")
            publicar_datos("Running", "SMALL", area_px, "Moviendo a Repisa Alta")
            #publicar_datos("Stopped", categoria, area_px, "Tomando caja")

            tomar_caja()
            alta()
            soltar_caja()
            home_position()

        elif categoria == "LARGE":
            print(f"[CONTEO] Caja LARGE clasificada. Total: {cont_large}")
            publicar_datos("Running", "LARGE", area_px, "Moviendo a Repisa Baja")
            tomar_caja()
            
            publicar_datos("Stopped", categoria, area_px, "Llevando a Repisa Baja")
            baja()
            soltar_caja()
            home_position() 
            
        else:
            print("[PROCESO] No se detectó ninguna caja válida. Volviendo a home.")
            home_position()

publicar_datos("Running", "Ninguna", 0, "Esperando caja")
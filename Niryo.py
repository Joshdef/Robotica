from pyniryo import NiryoRobot
import numpy as np
import cv2, time, serial, json
import paho.mqtt.client as mqtt




robot_ip= "200.126.13.186"
robot= NiryoRobot(robot_ip)
heltec=serial.Serial('COM3', 115200, timeout=1)


# Configuración MQTT para Ignition
MQTT_BROKER = "localhost"        
MQTT_PORT = 1883
MQTT_USER = "admin"
MQTT_PASS = "admin123"

TOPIC_ESTADO = "fabrica/linea1/estado"
TOPIC_COMANDOS = "fabrica/linea1/comando"

sistema_activo = False

home=[0,0,0,0,0,0]
aproxpick=[0.044,-0.215,-0.585,0.167,-0.695,0.005]
pick=[0.006,-0.743,-0.431,0.192,-0.494,-0.005]


#robot.set_learning_mode(True)

def home_position():
    print("Volviendo a la posicion de home")
    robot.move_joints(home)



def clasificacion_cajas(img_compressed):
    img_array = np.frombuffer(img_compressed, np.uint8)
    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

    if img is None:
        return "Ninguna"

   #Escala de grises
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    #Segmentacion
    _, thresh = cv2.threshold(blurred, 120, 255, cv2.THRESH_BINARY_INV)

    # 4. Encontrar contornos de los objetos
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filtrar pequeñas sombras o ruido (contornos menores a 100 píxeles)
    valid_contours = [c for c in contours if cv2.contourArea(c) > 100]

    if not valid_contours:
        return "NONE"

    # 5. Seleccionar la caja principal (el contorno más grande visible)
    main_contour = max(valid_contours, key=cv2.contourArea)
    area = cv2.contourArea(main_contour)
    
    print(f"[OpenCV] Área de la caja detectada: {area:.1f} píxeles")

    # 6. Umbral de corte (Ajustar según tus cajas en la práctica)
    UMBRAL_CORTE = 3000  

    if area < UMBRAL_CORTE:
        return "SMALL"
    else:
        return "LARGE"




def alta():
    robot.move_joints(0.839,-0.227,0.054,0.141,-1.121,-.1411)
    print("Yendo a repisa alta")



def baja():
    robot.move_joints(0.906,-0.757,-0.497,0.166,0.594,-0.250)
    print("Yendo a repisa alta")


def tomar_caja():
    print("Acercandose a la caja")
    robot.move_joints(*aproxpick)
    time.sleep(0.5)
    robot.move_joints(*pick)
    time.sleep(0.5)

    print("Agarrando...")
    try:
        robot.close_gripper(500)
    except Exception as e:
        print(f"Aviso de la pinza (se ignora para continuar): {e}")
       
    time.sleep(0.5)

    print("4. Elevando la caja...")
    robot.move_joints(*aproxpick)
    time.sleep(0.5)


def soltar_caja():
    robot.open_gripper(1000)
    time.sleep(0.5)


def banda_transportadora():
    print("Iniciando la banda transportadora...")
    heltec.flushInput()
    heltec.write(b'START\n')
    while True:
        if heltec.in_waiting > 0:
            respuesta=heltec.readline().decode('utf-8').strip()
            if respuesta == "caja_detectada":
                print("Caja detectada en la banda transportadora")
                return True
        time.sleep(0.02)
    return False


def coneccion_mqtt(client, userdata, flags, rc):
    if rc == 0:
        print("Conexión MQTT exitosa")
        client.subscribe(TOPIC_COMANDOS)
    else:
        print(f"Error de conexión MQTT: {rc}")


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
    mqtt_client.loop_start()  # Hilo en segundo plano para escuchar al HMI
except Exception as e:
    print(f"[MQTT ERROR] No se pudo conectar a Ignition: {e}")

def publicar_datos(banda, caja, area_px, robot_estado):
    """ Envia el estado actual del proceso a los Tags de Ignition """
    payload = {
        "banda": banda,              
        "caja": caja,                 
        "area_px": round(area_px, 1), 
        "robot": robot_estado         
    }
    mqtt_client.publish(TOPIC_ESTADO, json.dumps(payload))


def main():
    print("Iniciando el programa... ")
    captura=cv2.VideoCapture(0)

    # 1. Mover el robot a la posición de inicio
    home_position()

    print("Esperando a que el sistema se active desde Ignition...")

    while True:
        if  not sistema_activo:
            publicar_datos("Stopped", "Ninguna", 0, "Standby")
        time.sleep(0.5)
        continue

    publicar_datos("Running", "Ninguna", 0, "Esperando caja")
    caja_en_posicion = banda_transportadora(heltec)

    # 2. Clasificar la caja usando la cámara
    categoria= clasificacion_cajas(captura)



    if categoria == "SMALL":    
        print("Caja pequeña detectada")
        tomar_caja()
        alta()
        soltar_caja()
        home_position()

    elif categoria == "LARGE":
        print("Caja grande detectada")
        tomar_caja()
        baja()
        soltar_caja()
        home_position() 
    else:
        print("No se detectó ninguna caja válida. Volviendo a la posición de inicio.")
        home_position()




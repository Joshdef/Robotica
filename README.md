# 🤖 Proyecto de Clasificación de Cajas con Niryo One

Sistema automatizado de inspección y clasificación de cajas por tamaño (*Pick & Place*) utilizando visión por computadora y una banda transportadora integrada.

---

## 🛠️ Arquitectura del Sistema

* **Controlador Principal:** Python 3 (Entorno local con PyNiryo v1.2.5 y OpenCV).
* **Robot Colaborativo:** Niryo One (Conexión TCP/IP).
* **Control de Banda Transportadora:** Tarjeta Heltec ESP32 (Comunicación USB Serial).
* **Sistema de Visión:** Cámara USB HD.

---

## 🚀 Flujo de Trabajo

1. **Detección Inicial:** Python envía comando `START` a la placa Heltec para activar el motor de la banda transportadora.
2. **Parada Automática:** El sensor óptico/infrarrojo detecta la llegada de la caja al punto fijo; la Heltec apaga el motor de inmediato y notifica `BOX_READY` vía USB.
3. **Procesamiento de Imagen:** La cámara USB captura la foto del objeto. OpenCV calcula el área en píxeles y clasifica la caja en `SMALL` o `LARGE`.
4. **Manipulación Físico-Robótica:** El robot Niryo One ejecuta la secuencia de agarre (*Pick*) y traslada el objeto hacia la **Repisa Alta** (`SMALL`) o **Repisa Baja** (`LARGE`).

---

## ⚙️ Ejecución del Proyecto

1. Subir el código C++ a la placa Heltec usando **PlatformIO** (`src/main.cpp`).
2. Conectar la cámara USB y verificar el puerto COM de la Heltec.
3. Ejecutar el script principal:
   ```bash
   python Mainpy.py
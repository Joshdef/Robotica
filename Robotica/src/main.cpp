#include <Arduino.h>

//Pines placa
const int pin_puenteh = 22;  
const int pin_sensor_ir = 23;  

// Inicialización de variables
bool caja_detectada = false;
bool banda_activa = false;

void setup() {
  
  Serial.begin(115200);

  pinMode(pin_puenteh, OUTPUT);
  pinMode(pin_sensor_ir, INPUT); // Usar INPUT_PULLUP si tu sensor lo requiere

  // Estado inicial: Banda apagada
  digitalWrite(pin_puenteh, LOW);
}

void loop() {
  // 1. LEER COMANDOS DESDE PYTHON
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando == "START") {
      digitalWrite(pin_puenteh, HIGH);
      banda_activa = true;
      caja_detectada = false;
      Serial.println("START");
    } 
    else if (comando == "STOP") {
      digitalWrite(pin_puenteh, LOW);
      banda_activa = false;
      Serial.println("STOP");
    }
  }

  // 2. MONITOREAR EL SENSOR DE PRESENCIA
  if (banda_activa) {
    int estado_sensor = digitalRead(pin_sensor_ir);

    // Si el sensor detecta la caja (habitualmente LOW al interrumpir el haz)
    if (estado_sensor == LOW && !caja_detectada) {
      digitalWrite(pin_puenteh, LOW); // Detener la banda
      banda_activa = false;
      caja_detectada = true;

      // Enviar confirmación a Python
      Serial.println("caja_detectada");
    }
  }

  delay(20);
}
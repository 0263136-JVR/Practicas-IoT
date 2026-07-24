#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ── Configuración WiFi y MQTT ───────────────────────────────────────────────
const char* WIFI_SSID     = "Jaime_S24_FE";
const char* WIFI_PASSWORD = "abcd1234";
const char* MQTT_BROKER = "10.198.97.186";
const int   MQTT_PORT     = 1883;

const char* TOPIC_TELEMETRIA = "iot/motor/telemetria";
const char* TOPIC_SETPOINT   = "iot/motor/setpoint";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
bool mqttListo = false;

unsigned long ultimoTiempoMQTT = 0;
const unsigned long INTERVALO_MQTT = 500; // Publicar a MQTT cada 500 ms

// ── Pines de Control del Motor (DRV8833 en Paralelo) ────────────────────────
#define MOTOR_PWM_A  6
#define MOTOR_PWM_B  7

// ── Pines del Encoder del JGA25-370 ─────────────────────────────────────────
#define ENCODER_A    9  // Canal A 
#define ENCODER_B    10  // Canal B 

// ── Configuración Física del Motor y Zona Muerta ────────────────────────────
const float PPR = 374.0; 
const int PWM_MINIMO = 60; // Umbral mínimo para vencer la fricción estática

// ── Configuración de Rampas (Suavizado de velocidad) ────────────────────────
const double PASO_ACELERACION    = 5.0; // Suavidad al subir velocidad
const double PASO_DESACELERACION = 5.0; // Suavidad al bajar velocidad

// ── Variables de Control de Estado ──────────────────────────────────────────
volatile bool sentidoHorario = true;  
volatile bool paroEmergencia = false; 
volatile bool motorEncendido = false;  // Guarda el estado del motor (Inicia apagado)

bool modoTestActivo = false;           // Bandera para el Modo Test
unsigned long tiempoImpresoTest = 0;   // Contador de tiempo del test (hasta 5000ms)

double setpointDestinoRPM = 0.0;       // La velocidad final que el usuario desea (Comando S o MQTT)
double setpointActualRPM = 0.0;        // La velocidad transitoria que usa el PID (para las rampas)

// Variables globales para lectura asíncrona de MQTT
double rpmActualGlobal = 0.0;
int pwmGlobal = 0;

// ── Variables del PID ───────────────────────────────────────────────────────
double kp = 1.5;  // 1.5
double ki = 2.1;  // 0.6
double kd = 0.05; // 0.15

double errorAnterior = 0;
double integral = 0;
double derivativaFiltrada = 0; // Para suavizar picos en la derivada

// ── Variables de Tiempo y Encoder ───────────────────────────────────────────
volatile long encoderPulsos = 0;
unsigned long ultimoTiempoPID = 0;
const unsigned long INTERVALO_PID = 50; 

// ── Funciones de Conectividad ───────────────────────────────────────────────

void conectarWiFi() {
  Serial.printf("\n>> [WIFI] Conectando a '%s'", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos++ < 40) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n>> [WIFI] OK - IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n>> [WIFI] FALLO - Continuando offline (Modo Local)");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg = "";
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];

  if (String(topic) == TOPIC_SETPOINT) {
    float sp = msg.toFloat();
    if (sp >= 0) {
      setpointDestinoRPM = sp;
      Serial.printf("\n>> [MQTT] Nuevo objetivo recibido: %.2f RPM\n", setpointDestinoRPM);
    }
  }
}

void conectarMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  int intentos = 0;
  while (!mqttClient.connected() && intentos++ < 3) {
    Serial.print(">> [MQTT] Conectando al broker...");
    if (mqttClient.connect("ESP32-Motor-Original")) {
      mqttClient.subscribe(TOPIC_SETPOINT);
      mqttListo = true;
      Serial.println(" OK");
    } else {
      Serial.printf(" fallo (rc=%d)\n", mqttClient.state());
      delay(1000);
    }
  }
}

void publicarTelemetria() {
  if (!mqttListo || !mqttClient.connected()) return;

  StaticJsonDocument<200> doc;
  doc["rpm"]      = round(rpmActualGlobal * 10.0f) / 10.0f;
  doc["setpoint"] = setpointActualRPM; 
  doc["error"]    = round(abs(setpointActualRPM - rpmActualGlobal) * 10.0f) / 10.0f;
  doc["pwm"]      = pwmGlobal;
  doc["nodo"]     = "ESP32-Motor-PID";

  char buf[200];
  serializeJson(doc, buf);
  mqttClient.publish(TOPIC_TELEMETRIA, buf);
}

// ── Interrupciones ──────────────────────────────────────────────────────────

void IRAM_ATTR analizarEncoder() {
  int estadoB = digitalRead(ENCODER_B);
  if (estadoB == HIGH) { encoderPulsos++; } else { encoderPulsos--; }
}

// ── Setup ───────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } 

  pinMode(MOTOR_PWM_A, OUTPUT);
  pinMode(MOTOR_PWM_B, OUTPUT);
  
  frenarMotor(); // Seguridad inicial

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), analizarEncoder, RISING);

  // Inicialización de Red
  conectarWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);
    conectarMQTT();
  }

  // ── Filtro de Arranque ──
  delay(200);                 // Espera a que la tensión de la fuente se estabilice
  encoderPulsos = 0;          // Limpia cualquier pulso falso del interruptor de encendido
  ultimoTiempoPID = millis(); // Sincroniza el reloj del PID
  ultimoTiempoMQTT = millis();

  Serial.println("\n--- ¡Sistema PID IoT con Modo Test Listo! ---");
  imprimirInstrucciones();
}

// ── Loop ────────────────────────────────────────────────────────────────────

void loop() {
  if (mqttListo) mqttClient.loop();
  procesarConsola();

  unsigned long tiempoActual = millis();

  if (tiempoActual - ultimoTiempoPID >= INTERVALO_PID) {
    double dt = (tiempoActual - ultimoTiempoPID) / 1000.0; 
    ultimoTiempoPID = tiempoActual;

    noInterrupts();
    long pulsosAcumulados = encoderPulsos;
    encoderPulsos = 0; 
    interrupts();

    double rps = abs((double)pulsosAcumulados) / (PPR * dt);
    double rpmActual = rps * 60.0;
    rpmActualGlobal = rpmActual;

    bool requierePID = true;

    // ── GESTIÓN DE ESTADOS Y RAMPAS ──
    if (modoTestActivo) {
      if (setpointActualRPM < setpointDestinoRPM) {
        setpointActualRPM += PASO_ACELERACION;
        if (setpointActualRPM > setpointDestinoRPM) setpointActualRPM = setpointDestinoRPM;
      }
    }
    else if (paroEmergencia) {
      frenarMotor();
      setpointActualRPM = 0.0; 
      integral = 0;
      errorAnterior = 0;
      requierePID = false;
      pwmGlobal = 0;
      Serial.print("Target:0.00,Actual:"); Serial.print(rpmActual); Serial.println(",PWM:EMERGENCIA_ESTOP");
    } 
    else if (!motorEncendido) {
      if (setpointActualRPM > 0.0) {
        setpointActualRPM -= PASO_DESACELERACION;
        if (setpointActualRPM < 0.0) setpointActualRPM = 0.0;
      }
      
      if (setpointActualRPM == 0.0) {
        analogWrite(MOTOR_PWM_A, 0);
        analogWrite(MOTOR_PWM_B, 0);
        integral = 0;
        errorAnterior = 0;
        requierePID = false;
        pwmGlobal = 0;
        Serial.print("Target:0.00,Actual:"); Serial.print(rpmActual); Serial.println(",PWM:0_APAGADO");
      }
    } 
    else {
      if (setpointActualRPM < setpointDestinoRPM) {
        setpointActualRPM += PASO_ACELERACION;
        if (setpointActualRPM > setpointDestinoRPM) setpointActualRPM = setpointDestinoRPM;
      } 
      else if (setpointActualRPM > setpointDestinoRPM) {
        setpointActualRPM -= PASO_DESACELERACION;
        if (setpointActualRPM < setpointDestinoRPM) setpointActualRPM = setpointDestinoRPM;
      }
      
      if (setpointActualRPM == 0.0 && setpointDestinoRPM == 0.0) {
        analogWrite(MOTOR_PWM_A, 0);
        analogWrite(MOTOR_PWM_B, 0);
        integral = 0;
        errorAnterior = 0;
        requierePID = false;
        pwmGlobal = 0;
        Serial.print("Target:0.00,Actual:"); Serial.print(rpmActual); Serial.println(",PWM:0_REPOSO");
      }
    }

    // ── EJECUCIÓN DEL CONTROLADOR PID ──
    if (requierePID) {
      double error = setpointActualRPM - rpmActual;
      
      // 1. Banda Muerta (Deadband): Previene que el motor "vibre" cazando errores microscópicos
      const double BANDA_MUERTA = 1.0; // Si el error es menor a 1 RPM, lo damos por bueno
      if (abs(error) < BANDA_MUERTA) {
          error = 0; 
      }

      // 2. Cálculo Proporcional y Derivativo (con Filtro Paso Bajo)
      double P = kp * error;
      
      double derivativaCruda = (error - errorAnterior) / dt;
      // Filtro paso bajo para la derivada (alfa = 0.3)
      derivativaFiltrada = 0.3 * derivativaCruda + 0.7 * derivativaFiltrada;
      double D = kd * derivativaFiltrada;
      
      errorAnterior = error;

      // 3. Integración Condicional (Anti-Windup Avanzado)
      // Calculamos cuánto control aplicaríamos en este ciclo
      double controlTeorico = P + D + (ki * (integral + error * dt));
      
      // Límites efectivos de control (descontando el mínimo para vencer la inercia)
      double limiteSup = 255.0 - PWM_MINIMO;
      double limiteInf = 0.0;
      
      // Solo sumamos a la integral si NO estamos saturados, o si el error tiende a reducir la saturación
      bool saturadoArriba = (controlTeorico >= limiteSup && error > 0);
      bool saturadoAbajo  = (controlTeorico <= limiteInf && error < 0);
      
      if (!saturadoArriba && !saturadoAbajo) {
          integral += error * dt;
      }

      // Mantenemos tu límite estático de seguridad
      if (ki != 0) { integral = constrain(integral, -255.0 / ki, 255.0 / ki); }
      
      // 4. Cálculo final del esfuerzo de control
      double controlU = P + (ki * integral) + D;
      
      int pwmControlado = 0;
      if (controlU > 0) {
        pwmControlado = (int)controlU + PWM_MINIMO;
      } else {
        pwmControlado = PWM_MINIMO; 
      }
      pwmControlado = constrain(pwmControlado, PWM_MINIMO, 255);
      
      pwmGlobal = pwmControlado;
      aplicarMovimiento(pwmControlado);

      // Impresión en formato estándar
      Serial.print("Target:"); Serial.print(setpointActualRPM); Serial.print(",");
      Serial.print("Actual:"); Serial.print(rpmActual); Serial.print(",");
      Serial.print("Error:"); Serial.print(abs(setpointActualRPM - rpmActual)); Serial.print(",");
      Serial.print("PWM:"); Serial.println(pwmControlado);

      // Lógica de finalización del Modo Test
      if (modoTestActivo) {
        tiempoImpresoTest += INTERVALO_PID;
        if (tiempoImpresoTest >= 5000) {
          analogWrite(MOTOR_PWM_A, 0);
          analogWrite(MOTOR_PWM_B, 0);
          
          Serial.println("--- FIN MODO TEST (Motor Apagado) ---");
          Serial.println(">> Esperando 5 segundos de pausa final...");
          Serial.flush();
          
          delay(5000);
          
          modoTestActivo = false;
          motorEncendido = false;
          setpointActualRPM = 0.0;
          integral = 0;
          errorAnterior = 0;
          
          Serial.println(">> Sistema listo para nuevas instrucciones.\n");
        }
      }
    }
  }

  // 3. Bucle de transmisión MQTT
  if (tiempoActual - ultimoTiempoMQTT >= INTERVALO_MQTT) {
    ultimoTiempoMQTT = tiempoActual;
    publicarTelemetria();
    if (mqttListo && !mqttClient.connected()) conectarMQTT();
  }
}

// ── Funciones de Control del Driver DRV8833 ─────────────────────────────────

void aplicarMovimiento(int pwm) {
  if (sentidoHorario) {
    analogWrite(MOTOR_PWM_A, pwm);
    analogWrite(MOTOR_PWM_B, 0);
  } else {
    analogWrite(MOTOR_PWM_A, 0);
    analogWrite(MOTOR_PWM_B, pwm);
  }
}

void frenarMotor() {
  analogWrite(MOTOR_PWM_A, 255);
  analogWrite(MOTOR_PWM_B, 255);
}

// ── Utilidades de Consola Interactiva ───────────────────────────────────────

void imprimirInstrucciones() {
  Serial.println("\n--- COMANDOS DE CONTROL DISPONIBLES ---");
  Serial.println("  S<valor>  : Definir RPM objetivo (Ej: S120)");
  Serial.println("  KP<valor> : Modificar Kp (Ej: KP2.5)");
  Serial.println("  KI<valor> : Modificar Ki (Ej: KI1.8)");
  Serial.println("  KD<valor> : Modificar Kd (Ej: KD0.05)");
  Serial.println("  C         : Ver constantes PID actuales");
  Serial.println("  T         : EJECUTAR MODO TEST (5s con impresiones normalizadas)");
  Serial.println("  R         : ENCENDER / REANUDAR (Modo continuo)");
  Serial.println("  A         : APAGADO GRADUAL (Bajada suave y progresiva)");
  Serial.println("  E         : PARO DE EMERGENCIA (Freno total instantáneo)");
  Serial.println("  D         : Cambiar sentido de giro");
  Serial.println("---------------------------------------\n");
}

void procesarConsola() {
  if (Serial.available() > 0) {
    char comando = Serial.read();
    
    switch (comando) {
      case 'S': case 's': {
        float nuevaVelocidad = Serial.parseFloat();
        if (nuevaVelocidad >= 0) {
          setpointDestinoRPM = nuevaVelocidad;
          Serial.print(">> Objetivo guardado: ");
          Serial.print(setpointDestinoRPM);
          Serial.println(" RPM. Usa 'T' para test de 5s o 'R' para modo continuo.");
        }
        break;
      }
      case 'K': case 'k': {
        delay(5); 
        char subComando = Serial.read();
        float nuevoValor = Serial.parseFloat();
        
        if (nuevoValor >= 0) {
          if (subComando == 'P' || subComando == 'p') {
            kp = nuevoValor;
            Serial.print(">> [PID] Kp actualizado a: "); Serial.println(kp, 4);
          } else if (subComando == 'I' || subComando == 'i') {
            ki = nuevoValor;
            integral = 0; 
            Serial.print(">> [PID] Ki actualizado a: "); Serial.println(ki, 4);
          } else if (subComando == 'D' || subComando == 'd') {
            kd = nuevoValor;
            Serial.print(">> [PID] Kd actualizado a: "); Serial.println(kd, 4);
          }
        }
        break;
      }
      case 'C': case 'c': {
        Serial.println("\n--- VALORES ACTUALES DEL PID ---");
        Serial.print("  Kp = "); Serial.println(kp, 4);
        Serial.print("  Ki = "); Serial.println(ki, 4);
        Serial.print("  Kd = "); Serial.println(kd, 4);
        Serial.println("--------------------------------\n");
        break;
      }
      case 'T': case 't': {
        if (setpointDestinoRPM <= 0) {
          Serial.println(">> [AVISO] Define primero una velocidad objetivo con S<valor> (Ej: S100)");
        } else {
          Serial.println("\n>> [INICIO] Preparando Modo Test...");
          Serial.println(">> Silenciando e iniciando test en 5 segundos...");
          
          for (int i = 5; i > 0; i--) {
            Serial.print(">> "); Serial.print(i); Serial.println("...");
            delay(1000);
          }

          paroEmergencia = false;
          motorEncendido = true;
          modoTestActivo = true;
          
          setpointActualRPM = 0.0;
          integral = 0;
          errorAnterior = 0;
          tiempoImpresoTest = 0;
          
          noInterrupts();
          encoderPulsos = 0;
          interrupts();
          
          ultimoTiempoPID = millis();
          
          Serial.println("\n--- INICIO MODO TEST (5 Segundos) ---");
        }
        break;
      }
      case 'R': case 'r': {
        paroEmergencia = false;
        modoTestActivo = false;
        motorEncendido = true; 
        Serial.println(">> [ESTADO] Sistema ENCENDIDO en modo continuo...");
        break;
      }
      case 'A': case 'a': {
        modoTestActivo = false;
        motorEncendido = false; 
        Serial.println(">> [ESTADO] Iniciando desaceleración controlada...");
        break;
      }
      case 'E': case 'e': {
        paroEmergencia = true;   
        modoTestActivo = false;
        motorEncendido = false;  
        Serial.println(">> [ALERTA] ¡PARO DE EMERGENCIA ACTIVADO! Freno en seco aplicado.");
        break;
      }
      case 'D': case 'd': {
        sentidoHorario = !sentidoHorario;
        integral = 0; 
        Serial.print(">> Sentido cambiado a: ");
        Serial.println(sentidoHorario ? "HORARIO" : "ANTIHORARIO");
        break;
      }
    }
  }
}

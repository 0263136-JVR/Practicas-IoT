// ── Pines de Control del Motor (DRV8833 en Paralelo) ────────────────────────
#define MOTOR_PWM_A  7
#define MOTOR_PWM_B  6

// ── Pines del Encoder del JGA25-370 ─────────────────────────────────────────
#define ENCODER_A    5  // Canal A 
#define ENCODER_B    4  // Canal B 

// ── Configuración Física del Motor y Zona Muerta ────────────────────────────
const float PPR = 374.0; 
const int PWM_MINIMO = 20; // Umbral mínimo para vencer la fricción estática

// ── Configuración de Rampas (Suavizado de velocidad) ────────────────────────
// Ajusta cuántas RPM cambia por ciclo de 50ms (5.0 RPM / 50ms = 100 RPM por segundo)
const double PASO_ACELERACION   = 5.0; // Suavidad al subir velocidad
const double PASO_DESACELERACION = 5.0; // Suavidad al bajar velocidad

// ── Variables de Control de Estado ──────────────────────────────────────────
volatile bool sentidoHorario = true;  
volatile bool paroEmergencia = false; 
volatile bool motorEncendido = false;  // Guarda el estado del motor (Inicia apagado)

double setpointDestinoRPM = 0.0;       // La velocidad final que el usuario desea
double setpointActualRPM = 0.0;        // La velocidad transitoria que usa el PID (para las rampas)

// ── Variables del PID ───────────────────────────────────────────────────────
double kp = 1.5;   
double ki = 2;   
double kd = 0.04;  

double errorAnterior = 0;
double integral = 0;

// ── Variables de Tiempo y Encoder ───────────────────────────────────────────
volatile long encoderPulsos = 0;
unsigned long ultimoTiempoPID = 0;
const unsigned long INTERVALO_PID = 50; 

void IRAM_ATTR analizarEncoder() {
  int estadoB = digitalRead(ENCODER_B);
  if (estadoB == HIGH) { encoderPulsos++; } else { encoderPulsos--; }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } 

  pinMode(MOTOR_PWM_A, OUTPUT);
  pinMode(MOTOR_PWM_B, OUTPUT);
  
  frenarMotor(); // Seguridad inicial

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), analizarEncoder, RISING);

  // ── Filtro de Arranque ──
  delay(200);                 // Espera a que la tensión de la fuente se estabilice
  encoderPulsos = 0;          // Limpia cualquier pulso falso del interruptor de encendido
  ultimoTiempoPID = millis(); // Sincroniza el reloj del PID

  Serial.println("--- ¡Sistema PID con Doble Rampa (Aceleración/Desaceleración) Listo! ---");
  imprimirInstrucciones();
}

void loop() {
  // 1. Escuchar comandos por el Monitor Serie
  procesarConsola();

  // 2. Bucle de control PID periódico
  unsigned long tiempoActual = millis();
  if (tiempoActual - ultimoTiempoPID >= INTERVALO_PID) {
    double dt = (tiempoActual - ultimoTiempoPID) / 1000.0; 
    ultimoTiempoPID = tiempoActual;

    // Captura segura de pulsos
    noInterrupts();
    long pulsosAcumulados = encoderPulsos;
    encoderPulsos = 0; 
    interrupts();

    // Calcular RPM actuales
    double rps = abs((double)pulsosAcumulados) / (PPR * dt);
    double rpmActual = rps * 60.0;

    bool requierePID = true;

    // ── MÁQUINA DE ESTADOS DE CONTROL Y RAMPAS ──
    
    if (paroEmergencia) {
      // ESTADO 1: PARO DE EMERGENCIA (Frena en seco de inmediato)
      frenarMotor();
      setpointActualRPM = 0.0; // Reseteamos la rampa a cero para cuando se reanude
      integral = 0;
      errorAnterior = 0;
      requierePID = false;
      
      Serial.print("Target:0.00,Actual:"); Serial.print(rpmActual); Serial.println(",PWM:EMERGENCIA_ESTOP");
    } 
    else if (!motorEncendido) {
      // ESTADO 2: APAGADO GRADUAL (Rampa de bajada controlada)
      if (setpointActualRPM > 0.0) {
        setpointActualRPM -= PASO_DESACELERACION;
        if (setpointActualRPM < 0.0) setpointActualRPM = 0.0;
      }
      
      // Si la rampa llegó a cero, apagamos por completo los pines del driver
      if (setpointActualRPM == 0.0) {
        analogWrite(MOTOR_PWM_A, 0);
        analogWrite(MOTOR_PWM_B, 0);
        integral = 0;
        errorAnterior = 0;
        requierePID = false;
        
        Serial.print("Target:0.00,Actual:"); Serial.print(rpmActual); Serial.println(",PWM:0_APAGADO");
      }
    } 
    else {
      // ESTADO 3: MOTOR ENCENDIDO (Rampas dinámicas de funcionamiento)
      
      // Si el objetivo actual es menor que el deseado -> Aceleramos progresivamente
      if (setpointActualRPM < setpointDestinoRPM) {
        setpointActualRPM += PASO_ACELERACION;
        if (setpointActualRPM > setpointDestinoRPM) setpointActualRPM = setpointDestinoRPM;
      } 
      // Si el objetivo actual es mayor que el deseado -> Desaceleramos progresivamente
      else if (setpointActualRPM > setpointDestinoRPM) {
        setpointActualRPM -= PASO_DESACELERACION;
        if (setpointActualRPM < setpointDestinoRPM) setpointActualRPM = setpointDestinoRPM;
      }
      
      // Si ambos setpoints son cero, apagamos por completo para evitar zumbidos residuales
      if (setpointActualRPM == 0.0 && setpointDestinoRPM == 0.0) {
        analogWrite(MOTOR_PWM_A, 0);
        analogWrite(MOTOR_PWM_B, 0);
        integral = 0;
        errorAnterior = 0;
        requierePID = false;
        
        Serial.print("Target:0.00,Actual:"); Serial.print(rpmActual); Serial.println(",PWM:0_REPOSO");
      }
    }

    // ── EJECUCIÓN DEL CONTROLADOR PID ──
    if (requierePID) {
      double error = setpointActualRPM - rpmActual;
      
      // Anti-Windup
      integral += error * dt;
      if (ki != 0) { integral = constrain(integral, -255.0 / ki, 255.0 / ki); }
      
      double derivativa = (error - errorAnterior) / dt;
      errorAnterior = error;

      double controlU = (kp * error) + (ki * integral) + (kd * derivativa);
      
      int pwmControlado = 0;
      if (controlU > 0) {
        pwmControlado = (int)controlU + PWM_MINIMO;
      } else {
        pwmControlado = PWM_MINIMO; 
      }
      pwmControlado = constrain(pwmControlado, PWM_MINIMO, 255);

      aplicarMovimiento(pwmControlado);

      // Telemetría para el Monitor Serie / Serial Plotter
      Serial.print("Target:"); Serial.print(setpointActualRPM); Serial.print(",");
      Serial.print("Actual:"); Serial.print(rpmActual); Serial.print(",");
      Serial.print("Error:"); Serial.print(abs(setpointActualRPM-rpmActual)); Serial.print(",");
      Serial.print("PWM:"); Serial.println(pwmControlado);
    }
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
  Serial.println("  S<valor> : Definir RPM objetivo (Ej: S300)");
  Serial.println("  R        : ENCENDER / REANUDAR (Subida suave y progresiva)");
  Serial.println("  A        : APAGADO GRADUAL (Bajada suave y progresiva)");
  Serial.println("  E        : PARO DE EMERGENCIA (Freno total instantáneo)");
  Serial.println("  D        : Cambiar sentido de giro");
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
          Serial.println(" RPM. (Presiona 'R' para aplicar si está apagado)");
        }
        break;
      }
      case 'R': case 'r': {
        paroEmergencia = false;
        motorEncendido = true; 
        Serial.println(">> [ESTADO] Sistema ENCENDIDO. Acelerando progresivamente...");
        break;
      }
      case 'A': case 'a': {
        motorEncendido = false; 
        Serial.println(">> [ESTADO] Iniciando desaceleración controlada...");
        break;
      }
      case 'E': case 'e': {
        paroEmergencia = true;   
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

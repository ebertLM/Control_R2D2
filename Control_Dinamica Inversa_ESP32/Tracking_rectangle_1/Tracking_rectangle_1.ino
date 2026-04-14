/*
  Tracking_rectangle_V5.ino
  -------------------------
  Seguimiento en tiempo real en espacio operacional para un robot 3-DOF siguiendo un recorrido rectangular
  definido con 4 puntos en el espacio de trabajo.

  Entradas:
    • Conteos de encoders de cada articulación (cuadratura, x4)
    • Comando "start" vía Serial para comenzar la trayectoria

  Salidas:
    • Señales PWM hacia cada driver de motor
    • Telemetría (ángulos de articulación) enviada por Serial al host

  Dependencias:
    • ESP32Encoder
    • BasicLinearAlgebra
    • ElementStorage
    • CIK_Robot3DOF      – dinámica inversa + controlador PD
    • TrajectoryPlanning – planificador de trayectorias rectangulares
    • FuncionesAuxiliares  – rutinas auxiliares para Serial y PWM

  Software version:
      Arduino IDE
      Version: 2.3.6
      Date: 2025-04-09T11:26:55.498Z
      CLI Version: 1.2.0
      Copyright © 2025 Arduino SA  
*/

#include <ESP32Encoder.h>
#include "CIK_Robot3DOF.h"
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include "TrajectoryPlanning.h"
#include "FuncionesAuxiliares.h"
#include "SingleEMAFilterLib.h"

using namespace BLA;

//=== Configuración de control (ganancias PD) ===
BLA::Matrix<3,3> Kp_3DOF = { 100, 0,   0,
                             0,   100, 0,
                             0,   0,   100 };   // Ganancia proporcional
BLA::Matrix<3,3> Kd_3DOF = { 20,  0,   0,
                             0,   20,  0,
                             0,   0,   20 };    // Ganancia derivativa

//=== Configuración de trayectoria (perfil trapezoidal) ===
const float factor = 0.35f;   // ds_c = factor * ds_max
const float ds_max = 1.0f;    // Velocidad máxima (m/s)
const float dds_max = 1.2f;   // Aceleración máxima (m/s²)
const float tadd   = 1.0f;    // Tiempo de reposo al final (s)

//=== Definición de puntos del rectángulo (metros) ===
const float so = 0.0f;  // Longitud inicial de perfil en cada lado
BLA::Matrix<3,1> p1 = {0.149f,  0.110f, 0.200f};  // Vértice 1
BLA::Matrix<3,1> p2 = {0.149f,  0.110f, 0.400f}; // Vértice 2
BLA::Matrix<3,1> p3 = {0.149f, -0.110f, 0.400f}; // Vértice 3
BLA::Matrix<3,1> p4 = {0.149f, -0.110f, 0.200f}; // Vértice 4
BLA::Matrix<3, 1> pd, dpd, ddpd; // posición, velocidad y aceleración deseada
const char *currentSide; // lado del rectángulo en tiempo real

//=== Estado de trayectoria ===
float s, ds, dds; // Posición, velocidad y aceleración del perfil trapezoidal

//=== Estado de temporización ===
float tf=0.0f, tf1=0.0f, tf2=0.0f, tf3=0.0f, tf4=0.0f; // Tiempos de navegación por lado y total
float t=0.0f, tc=0.0f, td=0.0f;                       // Tiempo muestreado, tiempo de crucero  y total
long  k=1;                             // Índice de muestra
const float Tms = 5.0f;
const float Ts  = Tms/1000.0f;         // Periodo de muestreo (s)

//=== Estado de encoders/juntas ===
BLA::Matrix<3,1> q, dq;          // q: posiciones; dq: velocidades
float posQ1Initial = 0.0f,       // Posición articular inicial (home)
      posQ2Initial = PI/2,
      posQ3Initial = -PI/2;

// Contadores de pulsos de los encoders
int64_t M1_counter_encoder=0, M2_counter_encoder=0, M3_counter_encoder=0;
// Ángulos y velocidades angulares calculados
float M1_ang=0, M2_ang=0, M3_ang=0;
float M1_ang_old=posQ1Initial, M2_ang_old=posQ2Initial, M3_ang_old=posQ3Initial;
float M1_w=0, M2_w=0, M3_w=0;

//=== Configuración de hardware: encoders ===
ESP32Encoder M1_encoder; 
const int M1_encoderA = 39, M1_encoderB = 36; 
const float M1_CPR = 161280; 

ESP32Encoder M2_encoder; 
const int M2_encoderA = 35, M2_encoderB = 34; 
const float M2_CPR = 161280; 

ESP32Encoder M3_encoder; 
const int M3_encoderA = 33, M3_encoderB = 32; 
const float M3_CPR = 107712; 

//=== Configuración de hardware: PWM motorización ===
const int PWM_RESOLUTION = 1023;  // 10 bits

typedef struct {
  int pin;         // GPIO pin
  int frecuencia;  // Frecuencia PWM (Hz)
  int resolucion;  // Bits de resolución
  int canal;       // Canal de hardware  
} PWM_t;

// PWM motor 1
PWM_t PWM1 = { .pin=22, .frecuencia=10000, .resolucion=10, .canal=0 };
PWM_t PWM2 = { .pin=23, .frecuencia=10000, .resolucion=10, .canal=1 };
// PWM motor 2
PWM_t PWM3 = { .pin=21, .frecuencia=10000, .resolucion=10, .canal=2 };
PWM_t PWM4 = { .pin=19, .frecuencia=10000, .resolucion=10, .canal=3 };
// PWM motor 3
PWM_t PWM5 = { .pin=17, .frecuencia=10000, .resolucion=10, .canal=4 };
PWM_t PWM6 = { .pin=18, .frecuencia=10000, .resolucion=10, .canal=5 };

//=== Límites de control ===
const float outMaxVoltage   = 3.3f;   // Vcc del ESP32
const float outMaxTorque_M1 = 3.5f;  // Torque máximo por motor
const float outMaxTorque_M2 = 3.5f;  
const float outMaxTorque_M3 = 0.98f;

//=== Variables de control de señal PWM ===
int M1_pwm_motor=0, M2_pwm_motor=0, M3_pwm_motor=0;
BLA::Matrix<3,1> u, VuC;  // Torque y voltaje de control

//=== Buffers de telemetría ===
int    lengthColumn = 0;     // Número de muestras totales
float **Q, **Vc;             // Buffer para q1,q2,q3(Q) y voltaje de control(Vc)
const int nVector2Send = 2;  // Número de vectores a enviar
const int timeDelay = 1;     // Delay tras cada envío (ms)
String inputString = "", // Variable para almacenar la cadena recibida 
       nameMatrixQ = "vectorQ",
       nameMatrixVc = "vectorVc",
       message2start = "Starting...",
       message2finished = "Finished...";

//=== Temporizador por hardware ===
hw_timer_t *Timer = NULL;
volatile byte FLAG_TIMER = 0;
byte START      = 0;

//=== Rutina de interrupción del timer ===
void IRAM_ATTR Timer_ISR() 
{
  FLAG_TIMER = 1;
}

//=== Objetos de control ===
CIK_Robot3DOF CCI_R2D2(Kp_3DOF, Kd_3DOF);
TrajectoryPlanning rectanglePlanner(p1,p2,p3,p4, so, Ts, tadd,
                                    factor, ds_max, dds_max);

//=== Filtros ===    
const float alpha_dQ = 0.34f;                
SingleEMAFilter<float> EMAFilterM1(alpha_dQ);          
SingleEMAFilter<float> EMAFilterM2(alpha_dQ);  
SingleEMAFilter<float> EMAFilterM3(alpha_dQ);  

//=== Setup: inicialización de periféricos y buffers ===
void setup() {
  //--- Serial para telemetría y comandos ---
  Serial.begin(115200);
  inputString.reserve(10);

  //--- Inicializar encoders en modo x4 ---
  M1_encoder.attachFullQuad(M1_encoderA, M1_encoderB);
  M1_encoder.clearCount();
  M2_encoder.attachFullQuad(M2_encoderA, M2_encoderB);
  M2_encoder.clearCount();
  M3_encoder.attachFullQuad(M3_encoderA, M3_encoderB);
  M3_encoder.clearCount();

  //--- Configurar canales PWM ---
  ledcAttachChannel(PWM1.pin, PWM1.frecuencia, PWM1.resolucion, PWM1.canal); // motor 1
  ledcAttachChannel(PWM2.pin, PWM2.frecuencia, PWM2.resolucion, PWM2.canal);
  ledcAttachChannel(PWM3.pin, PWM3.frecuencia, PWM3.resolucion, PWM3.canal); // motor 2
  ledcAttachChannel(PWM4.pin, PWM4.frecuencia, PWM4.resolucion, PWM4.canal);
  ledcAttachChannel(PWM5.pin, PWM5.frecuencia, PWM5.resolucion, PWM5.canal); // motor 3
  ledcAttachChannel(PWM6.pin, PWM6.frecuencia, PWM6.resolucion, PWM6.canal);

  //--- Configurar timer a 1MHz con alarma cada Ts ms ---
  uint32_t freq_sec = 1000000;             // Timer a 1MHz
  Timer = timerBegin(freq_sec);            // Creación y configuración del temporizador
  timerAttachInterrupt(Timer, &Timer_ISR); // Rutina de interrupción Timer_ISR
  uint64_t alarm_value = Ts*freq_sec;      // Valor de la alarma (micro segundos)
  timerAlarm(Timer,alarm_value,true,0);    // Configura la alarma cada Ts ms(alarm_value),  
                                           // y reinicia automáticamente la alarma(true)

  //--- Ajustar límites de control ---
  CCI_R2D2.set_outputMaxTorque(outMaxTorque_M1, outMaxTorque_M2, outMaxTorque_M3);
  CCI_R2D2.set_outputMaxVoltage(outMaxVoltage, outMaxVoltage, outMaxVoltage);

  //--- Calcular los parámetros del perfil trapezoidal
  rectanglePlanner.computeParameters();
  tf  = rectanglePlanner.getTotalTime();
  tf1 = rectanglePlanner.getTf1();
  tf2 = rectanglePlanner.getTf2();
  tf3 = rectanglePlanner.getTf3();
  tf4 = rectanglePlanner.getTf4();
  td  = tf + tadd;
  lengthColumn = (int)(td / Ts);

  //--- Buffers ---
  Q = new float*[3];
  Vc = new float*[3];
  for (int i = 0; i < 3; ++i) 
  {
    Q[i] = new float[lengthColumn];
    Vc[i] = new float[lengthColumn];
  }
}

//=== Bucle principal: lectura, control y telemetría ===
void loop() {
  // Espera el evento de timer y señal de inicio
  if (START == 1) 
  {
    M1_encoder.clearCount();
    M2_encoder.clearCount();
    M3_encoder.clearCount();
    START = 2;
  }
  if (!(FLAG_TIMER==1 && START == 2)) return;
  FLAG_TIMER = 0;

  //--- A. Leer pulsos de los encoders ---
  M1_counter_encoder = M1_encoder.getCount();
  M2_counter_encoder = M2_encoder.getCount();
  M3_counter_encoder = M3_encoder.getCount();

  //--- B. Calcular q y dq (posición y velocidad angular en tiempo real) ---
  M1_ang = (M1_counter_encoder / M1_CPR) * 2*PI + posQ1Initial;
  M2_ang = (M2_counter_encoder / M2_CPR) * 2*PI + posQ2Initial;
  M3_ang = (M3_counter_encoder / M3_CPR) * 2*PI + posQ3Initial;
  q = {M1_ang, M2_ang, M3_ang}; // Posición angular en tiempo real

  M1_w = (M1_ang - M1_ang_old) / Ts;
  M2_w = (M2_ang - M2_ang_old) / Ts;
  M3_w = (M3_ang - M3_ang_old) / Ts;

  // Calcular filtro a la velocidad angular
  EMAFilterM1.AddValue(M1_w);  
  EMAFilterM2.AddValue(M2_w);  
  EMAFilterM3.AddValue(M3_w);  

  // Valores filtrados  
  M1_w = EMAFilterM1.GetLowPass();
  M2_w = EMAFilterM2.GetLowPass();
  M3_w = EMAFilterM3.GetLowPass();

  // Velocidad angular en tiempo real
  dq = {M1_w, M2_w, M3_w};     

  M1_ang_old = M1_ang;
  M2_ang_old = M2_ang;
  M3_ang_old = M3_ang;

  //--- C. Verifica los límites establecidos de las posiciones angulares ---
  RobotCinematica::verifyAngleQ(q);

  //--- D. Generar perfil y trayectoria según el lado actual ---
  t = k * Ts;
  if      (t <= tf1)                                 currentSide = "lado1";
  else if (t <= tf1 + tf2)        t-= tf1,           currentSide = "lado2";
  else if (t <= tf1 + tf2 + tf3)  t-= (tf1+tf2),     currentSide = "lado3";
  else if (t <= tf)               t-= (tf1+tf2+tf3), currentSide = "lado4";
  else                            t = tf,            currentSide = "lado4";

  auto profileNow    = rectanglePlanner.profile(t, currentSide);
  auto trajectoryNow = rectanglePlanner.trajectory(profileNow, currentSide);
  pd   = trajectoryNow.pd;    // Posición deseada
  dpd  = trajectoryNow.dpd;   // Velocidad deseada
  ddpd = trajectoryNow.ddpd;  // Aceleración deseada

  //--- E. Cálculo de torque y voltaje de control ---
  u   = CCI_R2D2.get_torque(pd, dpd, ddpd, q, dq);
  VuC = CCI_R2D2.get_voltage_uC(u);

  //--- F. Aplicar PWM a los motores ---
  M1_pwm_motor = VuC(0)*(PWM_RESOLUTION/outMaxVoltage);
  M2_pwm_motor = VuC(1)*(PWM_RESOLUTION/outMaxVoltage);
  M3_pwm_motor = VuC(2)*(PWM_RESOLUTION/outMaxVoltage);
  enviar_pwm_motor(M1_pwm_motor, PWM1.pin, PWM2.pin);
  enviar_pwm_motor(M2_pwm_motor, PWM3.pin, PWM4.pin);
  enviar_pwm_motor(M3_pwm_motor, PWM5.pin, PWM6.pin);

  //--- G. Almacenar datos para telemetría ---
  if (k <= lengthColumn) 
  {
    // Envío de los angulos de las juntas en tiempo real
    Q[0][k-1] = M1_ang;
    Q[1][k-1] = M2_ang;
    Q[2][k-1] = M3_ang;

    // Envío de los voltajes de control en tiempo real
    Vc[0][k-1] = VuC(0);
    Vc[1][k-1] = VuC(1);
    Vc[2][k-1] = VuC(2);    
  }

  //--- H. Envío de datos vía serial al terminar la recopilación ---
  else 
  {
    //Apagamos los motores
    // enviar_pwm_motor(0, PWM1.pin, PWM2.pin);
    // enviar_pwm_motor(0, PWM3.pin, PWM4.pin);
    // enviar_pwm_motor(0, PWM5.pin, PWM6.pin); 
    
    // Enviar el mensaje para comenzar a enviar la data
    sendString(message2start, timeDelay);  

    // Numero de vectores a enviar
    send_int(nVector2Send, timeDelay);

    // Enviamos los vectores por puerto serial
    sendVectorByPortSerial(Q, lengthColumn, nameMatrixQ, timeDelay);
    sendVectorByPortSerial(Vc, lengthColumn, nameMatrixVc, timeDelay);
    
    // Enviar el mensaje del finalización de envío de data
    sendString(message2finished, timeDelay);  // Enviar el mensaje
    START = 0;
  }
  k++;
}

//=== Funciones auxiliares ===
// Gestiona la recepción de comandos por serial ("start")
void serialEvent() {
  handleSerialEvent();
}

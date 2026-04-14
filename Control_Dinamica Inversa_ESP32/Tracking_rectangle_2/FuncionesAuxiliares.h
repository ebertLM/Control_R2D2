#pragma once
#include <Arduino.h>

// SerialEvent necesita acceder a estas dos variables globales:
// inputString (String) y START (byte)
// así que las declaramos extern aquí:
extern String inputString;
extern byte   START;

// Manejo de recepción de comandos por serie
void handleSerialEvent();

// Envío de datos por el puerto serie
void sendString(String message, int timeDelay);
void send_float(float value,   int timeDelay);
void send_int(int value,     int timeDelay);
void sendVectorByPortSerial(float** vector2send, const int &lengthColumn, const String &nameVector, int timeDelay);

// Control de motores por PWM
void enviar_pwm_motor(const int &pwm, const int &PWM_pin1, const int &PWM_pin2);

#include "FuncionesAuxiliares.h"

// Lectura y procesamiento de la cadena recibida
void handleSerialEvent() 
{
  while (Serial.available()) 
  {
    char inChar = (char)Serial.read(); // Leer el caracter entrante
    inputString += inChar; // Añadirlo a la cadena
    inputString.toLowerCase();
    // Verificar si la cadena recibida es "start"
    if (inputString.equals("start") && START==0)   START=1;
  }
  inputString = "";
}

// Función para enviar datos tipo string por el puerto serial
void sendString(String message, int timeDelay) 
{
  // Primero enviamos la longitud del mensaje (para que MATLAB sepa cuántos bytes esperar)
  byte messageLength = message.length(); 
  Serial.write(messageLength);  // Enviar la longitud del mensaje

  // Luego enviamos el contenido del mensaje en una sola llamada a Serial.write()
  Serial.write(message.c_str());  // Enviar el string como una secuencia de bytes (char array)
  delay(timeDelay);
}

// Función para enviar datos tipo float por el puerto serial
void send_float(float value, int timeDelay)
{
  byte * data = (byte *) &value;
  Serial.write(data, sizeof (value));
  delay(timeDelay);
}

// Función para enviar datos tipo entero por el puerto serial
void send_int(int value, int timeDelay)
{
  byte * data = (byte *) &value;
  Serial.write(data, sizeof(value));
  delay(timeDelay);
}

// Función para enviar datos tipo vectores por el puerto serial
void sendVectorByPortSerial(float** vector2send, const int &lengthColumn, 
                            const String &nameVector, int timeDelay)
{
  // Enviamos el nombre del vector
  sendString(nameVector, timeDelay);

  // Enviamos el númeo de columnas del vector
  send_int(lengthColumn, timeDelay);

  // Enviamos cada uno de los valores del vector
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<lengthColumn; j++)
    {
      send_float(vector2send[i][j], timeDelay);    
    }
  }
}

// ENVIA EL PWM A LOS MOTORES
void enviar_pwm_motor(const int &pwm, const int &PWM_pin1, const int &PWM_pin2)
{
  if(pwm>=0)
  {
    ledcWrite(PWM_pin1, pwm); // PWM sale por IN1
    ledcWrite(PWM_pin2, 0);
  }
  else
  {
    ledcWrite(PWM_pin1, 0);
    ledcWrite(PWM_pin2, -pwm); // PWM sale por IN2
  }
}
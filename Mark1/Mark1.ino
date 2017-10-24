#include <QTRSensors.h>             // Librería para los sensores
#include <OpenTB6612FNG.h>          // Librería para los motores
#include <EEPROMex.h>               // Librería para EEPROM
 
#define BOTON  12
#define BUZZER  10
#define NUM_SENSORS 8               // Números de sensores
#define NUM_SAMPLES_PER_SENSOR 4    
#define EMITTER_PIN 11     

#define addrCalibratedMinimumOn 0
#define addrCalibratedMaximumOn 100

OpenTB6612FNG OpenTB6612FNG(0);

QTRSensorsAnalog qtra((unsigned char[]) {
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7
},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


/* PID Neural network¨*/
// Constantes para PID_NN

float Kp = 12;
float Ki = 0.00025;
float Kd = 10.5;

float a =  1;
float b =  1;
float c =  1;
float d = -1;
float e = -1;
float f = -1;

float theta1[2][3] = {

          {  a,  b,  c},
          {  e,  f,  g}
          
                      };

float theta2[3][1] = 
                 {

          {Kp},
          {Ki}, 
          {Kd}
                    
                 };                      

/*  ------------------------------ */

/* Variables */

float x = 0;             // Valor de entrada el PID (error)

float y = 0;             // Valor de salida del PID

float r = 1.5;           // Referencia

float x_prev = 0;        // Salida previa de Ineuron 
float u_prev = 0;        // Entrada previa de Dneuron

float Vel = 25;          // Velocidad actual    

float M1 = 0;
float M2 = 0;

int DETECT = 4;
          
/* ------------------------------ */

void setup()
{
  Serial.begin(9600);
  pinMode(BOTON, INPUT);
  pinMode(BUZZER, OUTPUT);
  WaitBoton();
  recallQTR();
  delay(1000);
}

void loop(){

  tiempo = millis();

  /* Sensores */
  unsigned int position = qtra.readLine(sensorValues);

  x = pos(sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3], sensorValues[4], sensorValues[5]);

  /* Detección de meta */
  if (sensorValues[6] > 500 && sensorValues[7] < 500)
  {
    meta_suma = meta_suma + 1;
  }

  if (meta_suma == DETECT)
  {
    meta = meta + 1;
    tone(BUZZER, 2500, 100); 
    delay(100);
    meta_suma = 0;
  }
  
  /*  ---------------------------------------------   */

  /* Controlador PID con estructura neuronal */
  float X[2] = {r, x};

  /* Función de neurona P */
  float a1[2];
  for (int i = 0; i < 2; i++)
  {
    a1[i] = Pneuron(X[i]);
  }

  /*  Multiplicación theta1*a1   */
  float z2[3];
  for (int i = 0; i < 3; i++)
  {
    z2[i] = 0;
    for(int j = 0; j < 2 ; j++)
    {    
      z2[i] = z2[i] + theta1[j][i]*a1[j];
    }
  }

  float a2[3];
  a2[0] = Pneuron(z2[0]);
  a2[1] = Ineuron(z2[1], x_prev);
  a2[2] = Dneuron(z2[2], u_prev);

  /* Ajuste de entradas y salidad de Ineuron y Dneuron */
  x_prev = a2[1];
  u_prev = z2[2];

  /*  Multiplicación a2*theta2   */
  float z3[1];
  for (int i = 0; i < 1; i++)
  {
    z3[i] = 0;
    for(int j = 0; j < 3; j++)
    {    
      z3[i] = z3[i] + a2[j]*theta2[j][i];
    }
  }
  
  y = Pneuron(z3[0]);  
  M1 = Vel + y;
  M2 = Vel - y;
  
  OpenTB6612FNG.Motores(M1, -M2);  

  /* Frenado en la zona de meta*/
  if (meta >= 2)
  {
   OpenTB6612FNG.ACTFrenoDe(1);
   OpenTB6612FNG.ACTFrenoIz(1); 
  }

}

/*  FUNCIONES   */
void IfBoton()
{
  if (digitalRead(BOTON) == HIGH) {
    tone(BUZZER, 600, 50);
    delay(200);
    WaitBoton();
    delay(200);
  }
}

void WaitBoton()
{
  /*  Función wait boton
   *  Espera la entrada de un 1 en la entrada
   *  del boton.
   */
  while (!digitalRead(BOTON));
  tone(BUZZER, 1800, 100);
}

void recallQTR()
{
  Serial.println();
  Serial.println("Recall de datos de calibración desde EEPROM...");

  qtra.calibrate(); 
  EEPROM.readBlock<unsigned int>(addrCalibratedMinimumOn, qtra.calibratedMinimumOn, 8);
  EEPROM.readBlock<unsigned int>(addrCalibratedMaximumOn, qtra.calibratedMaximumOn, 8);

  Serial.println("Recall completo");
}

/* Funciones de neuronas tipo P I D */
float threshold = 2000;
float Pneuron (float u){
  /* Neurona tipo P
   * La función limita el valor de la salida dentro del umbral (threshold)
   *  - u = neuron input
   *  - x = neuron output
   */
  float x;
  x = u;
  if (x > threshold){
    x = threshold;
  }
  if (x < -threshold){
    x = -threshold;
  }
  return x;
}

float Ineuron(float u , float x_prev1){
  /* Neurona tipo I
   * La función limita el valor de la salida dentro del umbral (threshold).
   * Ademas la salida corresponde a la entrada + la salida previa.
   *  - u = neuron input
   *  - x = neuron output 
   *  - x_prev = previous output
   */
  float x;
  x = u + x_prev1;
  if (x > threshold){
    x = threshold;
  }
  if (x < -threshold){
    x = -threshold;
  }
  return x;  
}

float Dneuron(float u , float u_prev){ 
  /* Neurona tipo D
   * La función limita el valor de la salida dentro del umbral (threshold).
   * Ademas la salida corresponde a la entrada - la entrada previa.
   *  - u = neuron input
   *  - x = neuron output 
   *  - u_prev = previous input
   */
  float x;
  x = u - u_prev;
  if (x > threshold){
    x = threshold;
  }
  if (x < -threshold){
    x = -threshold;
  }
  return x;  
}

int limit = 300;
float pos(float p1 , float p2 , float p3 , float p4 , float p5 , float p6 ){ 
  /* Función posición
   * La función convierte la entradas analogas de los sensores por entradas digitales
   * Ademas calcula el valor de la salida entre 0 y 3 en funciónde la
   * posición.
   *  - pi= input sensor 
   *  - y = output
   */
  float y;
  if(p1 > limit)
  {
    p1 = 1;
  }
  else
  {
    p1 = 0;
  }
  
  if(p2 > limit)
  {
    p2 = 1;
  }
  else
  {
    p2 = 0;
  }
  
  if(p3 > limit)
  {
    p3 = 1;
  }
  else
  {
    p3 = 0;
  }
  
  if(p4 > limit)
  {
    p4 = 1;
  }
  else
  {
    p4 = 0;
  }
  
  if(p5 > limit)
  {
    p5 = 1;
  }
  else
  {
    p5 = 0;
  }
  if(p6 > limit)
  {
    p6 = 1;
  }
  else
  {
    p6 = 0;
  }
  y = (1.5*p1 - 0.5*p2 + p3 - p4 + 0.5*p5 - 1.5*p6) + 1.5;
  if (y > 3)
  {
    y = 2.5;
  }
  if (y < 0)
  {
    y = 1;
  }
  return y;
}

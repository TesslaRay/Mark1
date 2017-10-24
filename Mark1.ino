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

int DETECT = 3;

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

float Kp = 3.5;
float Ki = -0.00005;
float Kd = 30;

/*
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
*/

/*
float Kp = 12;
float Ki = 0.00026;
float Kd = 11;
*/

float theta1[2][3] = {

          {  1,  1,  1},
          { -1, -1, -1}
          
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

float y_anterior = 1.5;

float r = 1.5;           // Referencia

float x_prev = 0;        // Salida previa de Ineuron 
float u_prev = 0;        // Entrada previa de Dneuron

float Vel_base = 40;     // Velocidad base
float Vel_giro_max = 35; // Velocidad de giro máxima
float Vel_giro_min = 30; // Velocidad de giro mínima
float Vel = 25;          // Velocidad actual    

float M1 = 0;
float M2 = 0;

float A_c = 0.5;
float A_r = 15;
          
/* ------------------------------ */

/* Variables adicionales */

int meta = 0;
int curva = 0;
int bonus = 0;

int curva_suma = 0;
int cruce_suma = 0;
int meta_suma = 0;

unsigned long tiempo = 0;
unsigned long t_actualizado = 0;
unsigned long t_actualizado2 = 0;

unsigned long t_delay = 2000/Vel_base;

/* ------------------------------ */

float M_count = 0;

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

  x = error(sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3], sensorValues[4], sensorValues[5], y_anterior);
  y_anterior = x;
  
  /*  Detección de cruce */
  int negro = 500;
  if (sensorValues[6] < negro && sensorValues[7] < negro)
  {
    //curva_suma = 0;
    cruce_suma = cruce_suma + 1;
    //meta_suma = 0;
  }

  if (cruce_suma == DETECT) 
  {
    //tone(BUZZER, 1500, 100); 
  }

  /* Detección de meta */
  if (sensorValues[6] > negro && sensorValues[7] < negro)
  {
    curva_suma = 0;
    cruce_suma = 0;
    meta_suma = meta_suma + 1;
  }

  if (meta_suma == DETECT)
  {
    meta = meta + 1;
    tone(BUZZER, 2500, 100); 
    delay(80);
    meta_suma = 0;
  }

  /* Detección de curva */
  if (sensorValues[6] < negro && sensorValues[7] > negro)
  {
    curva_suma = curva_suma + 1;
    cruce_suma = 0;
    meta_suma = 0;
  }
  
  if (curva_suma == DETECT)
  {
    Vel = Vel_giro_min;
    curva = curva + 1;  
    tone(BUZZER, 3000, 100);
    curva_suma = 0;       

    DETECT = 1;
    
    Kp = 14;
    Ki = 0.0000;
    Kd = 75;  
    
    /*
    Kp = 16;
    Ki = 0.000005;
    Kd = 82;  
    */
    M_count = 0;
      
  }

  if (detectar_curvatura(y, &M_count))
    {
      tone(BUZZER, 1300, 100); 
      Kp = 3.5;
      Ki = -0.00005;
      Kd = 30;
      Vel = 35;

      DETECT = 4;
    }

  /* Función de aceleración */
  /* 
   *  Acelera en línea recta con una aceleración 
   *  y en curva con otra.
   *    A_r: Aceleración en recta.
   *    A_c: Aceleración en curva.
   */
  if (tiempo > t_actualizado + t_delay)
  {
    t_actualizado = tiempo;
    if (detectar_curvatura(y, &M_count))
    {
      if (Vel <= Vel_base)
      {
        Vel = Vel + A_r;
      }
    }
    else
    {
      if (Vel <= Vel_giro_max)
      {
        Vel = Vel + A_c;
      }
    }  
    
  }
    
  /*  ---------------------------------------------   */

  float theta2[3][1] = 
                 {

          {Kp},
          {Ki}, 
          {Kd}
                    
                 };                                         


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

  Serial.println(x_prev);
  Serial.println(u_prev);

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
   OpenTB6612FNG.ACTFrenoDe(10);
   OpenTB6612FNG.ACTFrenoIz(10); 
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
float threshold = 500;
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
float error(float p1 , float p2 , float p3 , float p4 , float p5 , float p6, float y_ant){ /*funcion de velocidad*/
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
  
  if ((p1 == 1) && (p2 == 1) && (p3 == 1) && (p4 == 1) && (p5 == 1) && (p6 == 1) && (y_ant < 1.5))
  {
    y = -3.5;
  }
  if ((p1 == 1) && (p2 == 1) && (p3 == 1) && (p4 == 1) && (p5 == 1) && (p6 == 1) && (y_ant > 1.5))
  {
    y = 6.5;
  }
  if (y_ant == -3.5)
  {
    y = -4.5;
  }
  if (y_ant == 6.5)
  {
    y = 7.5;
  }
  
  /*if (y > 3)
  {
    y = 2.5;
  }
  if (y < 0)
  {
    y = 1;
  }
  */
  return y;
}

int detectar_curvatura(float y, float *count)
{
  if (abs(y) < 10)
  {
    *count = *count + 1;
  }
  if (*count == 25)
  {
    return 1;
  }
  return 0;
}



# Mark1
Código en C++ (Arduino) para un controlador PID basado en una red neuronal de estrucra 2-3-1.

* Introducción
* Ruteo de pines
* Próximas mejoras

### Introducción
El siguiente código muestra el funcionamiento de un controlador PID basado en una red neuronal de estructura 2-3-1.

![Una imagen cualquiera](http://molefrog.com/pidnn-talk/images/pidnn-system.png "De 150 x 150 píxeles")

* La primera entrada **r** corresponde al valor de referencia a seguir en este caso ** x - 1.5** *(x corresponde a la salida de los sensores)*


* La segunda entrada **(y - 1)** corresponde el valor de salida previa. 

### Ruteo de pines
 
**0**   TX Bluetooth   
**1**	  RX Bluetooth   
**2**	  Button   
**3**	  Buzzer   
**4**	  TB6612FNG PWMA   
**5**	  TB6612FNG PWMB  
**10**	TB6612FNG AIN1	(Izquierda)   
**11**  TB6612FNG AIN2	(Izquierda)   
**12**	TB6612FNG BIN1	(Derecha)   
**13**	TB6612FNG BIN2	(Derecha)   
**14**	QTR -> 1   
**15**	QTR -> 2   
**16**	QTR -> 3  
**17**	QTR -> 4   
**18**	QTR -> 5    
**19**	QTR -> 6   
**20**	QTR-1A --> Left   
**21**	QTR-1A --> Right    
**22**	QTR LED ON   
**23**	TB6612FNG STBY  

## Próximas mejoras
* Implementación de algoritmo genético para calcular los pesos.

## Referencias
```
[1] PID Neural Networks for Time-Delay Systems — H.L. Shu, Y. Pi (2000)
[2] Decoupled Temperature Control System Based on PID Neural Network — H.L. Shu, Y. Pi (2005)  
[3] Adaptive System Control with PID Neural Networks — F. Shahrakia, M.A. Fanaeib, A.R. Arjomandzadeha (2009)  
[4] Control System Design (Chapter 6) — Karl Johan Åström (2002)
```

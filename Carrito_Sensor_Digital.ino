/*****************************************************************
Este sketch requiere de la libreria del sensor QTR-8RC Reflectance Sensor Array, la cual se puede descargar en:
https://github.com/pololu/qtr-sensors-arduino.git
 
FUNCIONMIENTO DEL CARRITO
El carrito cuenta con dos botones, los cuales se les asignará un nombre de acuerdo a la posición que tienen con respecto al frente del carrito,
es decir que se tiene un botón que se encuentra a la derecha y otro botón que está a la izquierda. Aplica de la misma manera para los leds.
El BOTON_DER cuando sea presionado por mas de 1 seg permitirá entrar en modo CALIBRACION, cuando  se inicie esta rutina se encenderá
el LED_IZQ, y acontinuación se debe de colocar el carrito sobre la linea que se encuentre en la pista. Mover el carrito de derecha a izquierda
para lograr que todos los sensores se calibren. El fin de la calibración se podrá identificar cuando el LED_IZQ se apague.
El BOTON_IZQ permite colocar el carrito en Modo CARRERA, y solo podrá salir de esa rutina hasta que se presione el botón de reset del arduino.
Cuando se encuentre en Modo CARRERA, el LED_DER y LED_IZQ estarán prendidos.
EL lED_DER, indica cuando el carrito esta listo parar entra en cualquiera de los modos descritos anteriormente.
5V PWM RECTAS 60, PWM CURVAS 1, PWM_FRENADO = 60, GANANCIAS. Ganancias curvas PWM = 1, kp = 2, kd = 0.6, Ganancias Rectas PWM = 60, kp = 2, kd = 0.06;
5V PWM RECTAS 90, PWM CURVAS 1, PWM_FRENADO = 90, GANANCIAS. Ganancias curvas PWM = 1, kp = 2, kd = 0.6, Ganancias Rectas PWM = 60, kp = 2, kd = 0.06;
*****************************************************************/
#define DEBUG         true  //Modo de DEBUG, activelo si desea imprimir los valores de calibración y del control en el monitor serial
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   22    // emitter is controlled by digital pi, pin 11 para el arduino nano
#define MBIN1         4     //Pines para motor 1 que es el motor IZQUIERDO 5arduino nano
#define MBIN2         5     //Pines para motor 1                           6arduino nano
#define MAIN1         7    //Pines para motor 2 que es el motor DERECHO   10arduino nano
#define MAIN2         6     //Pines para motor 2                           9arduino nano
#define SEGUNDO       1000  //1000 milisegundos
#define BTN_DER       2     //Pin de botón Derecho
#define BTN_IZQ       3     //Pin de botón Izquierdo
#define LED_DER       8     //Pin de led para indicar carrito listo 
#define LED_IZQ       13    //Pin de led para indicar calibración, cambie el pin 7 en el arduino nano por el 13 el mega
#define DIRECCION     0xDC  //Direccion de inicial donde se guardan los valores de la calibración de los sensores

#include <QTRSensors.h>
#include <EEPROM.h>

QTRSensorsRC qtr((unsigned char[]) {
  24, 26, 28, 30, 32, 34, 36, 38}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
//QTRSensorsRC qtr((unsigned char[]) {4, 12, 14, 15, 16, 17, 18, 19}, //para arduino nano
//  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned long t_ant = 0, t_act = 0;             //Variables para medir intervalos de tiempo
byte seg = 0;
float kp, kd;                                   //Ganancias del controlador PD, modificar las ganancias en la función de control
byte PWM, PWM_FRENADO = 255;                     //PWM en velocidad deseado (modificar en la función de control) y PWM de frenado para apoyo en las curvas. 
int Posicion = 0;                               //Variable para almacenar la posicion actul de la línea
const int Referencia = 3500;                    //Posicion donde queremos que el carrito mantenga
float Proporcional = 0;                         //Control proporcional
float Derivativo = 0;                           //Control derivativo
int error_Ant = 0;                              //Error Anterior

void setup() {
  Inicializar_Btn_Leds();
  Inicializar_Motores();
  Serial1.begin(115200);
  Asignar_Calibracion(DIRECCION);
  digitalWrite(LED_DER, HIGH);
}
////////////////////////////////////////////////
//CÓDIGO PRINCIPAL
////////////////////////////////////////////////
void loop() {
  while (!digitalRead(BTN_DER)) {               //Mientras se presione por mas de 1 seg el BOTON DERECHO, entraremos en modo de CALIBRACION DE SENSORES
    t_act = millis();
    if (t_act - t_ant > SEGUNDO) {
      seg++;
      if (seg > 1) {
        Serial1.println("Modo Calibracion");
        Calibracion_Sensores();                 //En esta función se lleva a cabo la calibración
        seg = 0;
      }
      t_ant = t_act;
    }
  }
  while (!digitalRead(BTN_IZQ)) {              //Mientras se presiona el BOTON IZQUIERDO, entramos en modo CARRERA, el programa se cicla en ese punto, si desea
    while (true) {                             //detener el carrito, presione el botón de reset del arduino
      digitalWrite(LED_IZQ, HIGH);
      Control_PD();                            //Consultar la función si desea modificar el CONTROLADOR
    }
  }
  seg = 0;
}
////////////////////////////////////////////////
//FUNCIONES
////////////////////////////////////////////////
void Inicializar_Btn_Leds() {                  //Se configuran los leds y los botones como salidas y entradas digitales respectivamente
  pinMode(LED_DER, OUTPUT);
  pinMode(LED_IZQ, OUTPUT);
  pinMode(BTN_DER, INPUT_PULLUP);
  pinMode(BTN_IZQ, INPUT_PULLUP);
}

void Inicializar_Motores() {                  //Se configuran los pines como salidas digitales para los motores
  pinMode(MBIN1, OUTPUT);
  pinMode(MBIN2, OUTPUT);
  pinMode(MAIN2, OUTPUT);
  pinMode(MAIN1, OUTPUT);
}

void Motor_PWM(byte _pwm_M1, byte _pwm_M2) {  //Se configura el pwm deseado para que el carrito avance
  analogWrite(MBIN2, _pwm_M1); //Motor Izquierdo
  analogWrite(MBIN1, LOW);
  analogWrite(MAIN2, LOW);
  analogWrite(MAIN1, _pwm_M2); //Motor Derecho
}

void Frenado(byte _pwm_M1, byte _pwm_M2) {    //Configura que el motor que funcione como eje, gire en sentido inverso para facilitar los giros en las curvas
  if (Posicion >= 7000) {
    analogWrite(MBIN2, LOW); //Motor Izquierdo
    analogWrite(MBIN1, PWM_FRENADO);
    analogWrite(MAIN2, LOW);
    analogWrite(MAIN1, _pwm_M2); //Motor Derecho
  }
  if (Posicion <= 0) {
    analogWrite(MBIN2, _pwm_M1); //Motor Izquierdo
    analogWrite(MBIN1, LOW);
    analogWrite(MAIN2, PWM_FRENADO);
    analogWrite(MAIN1, LOW); //Motor Derecho
  }
}

void Calibracion_Sensores() {                 //Se calibra los máximos y mínimos de las lecturas de los sensores
  digitalWrite(LED_DER, LOW);
  delay(500);
  for (int i = 0; i < NUM_SENSORS; i++) {     //Se hace un reset a los valores de calibracion para permitir modificarlos
    qtr.calibratedMinimumOn[i] = 2500;
    qtr.calibratedMaximumOn[i] = 0;
  }
  digitalWrite(LED_IZQ, HIGH);                //Encendemos LED para indicar que inicio la calibración
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();                         //Función de calibración de los sensores
  }
  Guardar_Calibracion_EEPROM(DIRECCION);
  Leer_Calibracion_EEPROM(DIRECCION);
  digitalWrite(LED_IZQ, LOW);                 //Apagamos LED para indicar que terminó la calibración
  delay(500);
  digitalWrite(LED_DER, HIGH);
}

void Guardar_Calibracion_EEPROM(byte _Direccion) { //Se guardan los valores de calibración en la EEPROM
  for (int i = 0; i < NUM_SENSORS; i++) {
    int _Dato = qtr.calibratedMinimumOn[i];
    byte Byte_Alto = _Dato >> 8;
    byte Byte_Bajo = _Dato;
    EEPROM.write(_Direccion, Byte_Alto);
    delay(10);
    _Direccion++;
    EEPROM.write(_Direccion, Byte_Bajo);
    _Direccion++;
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    int _Dato = qtr.calibratedMaximumOn[i];
    byte Byte_Alto = _Dato >> 8;
    byte Byte_Bajo = _Dato;
    EEPROM.write(_Direccion, Byte_Alto);
    delay(10);
    _Direccion++;
    EEPROM.write(_Direccion, Byte_Bajo);
    _Direccion++;
  }
}

void Leer_Calibracion_EEPROM(byte _Direccion) {  //Se leen los valores de calibración de la EEPROM
  for (int i = 0; i < NUM_SENSORS; i++) {
    int Valor = EEPROM.read(_Direccion);
    Valor <<=  8;
    _Direccion++;
    Valor += EEPROM.read(_Direccion);;
    _Direccion++;
    if (DEBUG == true) {
      Serial1.print(Valor);
      Serial1.print(" ");
    }
  }
  if (DEBUG == true)   {
    Serial1.println("");
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    int Valor = EEPROM.read(_Direccion);
    Valor <<=  8;
    _Direccion++;
    Valor += EEPROM.read(_Direccion);;
    _Direccion++;
    if (DEBUG == true) {
      Serial1.print(Valor);
      Serial1.print(" ");
    }
  }
  if (DEBUG == true) {
    Serial1.println("");
  }
}

void Asignar_Calibracion(byte _Direccion) {   //Se asignan los valores de calibración almacenados en la EEPROM a las variables de calibración en RAM
  qtr.calibrate();
  if (DEBUG == true) {
    Serial1.println("Calibracion Actual");
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    int Valor = EEPROM.read(_Direccion);
    Valor <<=  8;
    _Direccion++;
    Valor += EEPROM.read(_Direccion);;
    _Direccion++;
    qtr.calibratedMinimumOn[i] = Valor;
    if (DEBUG == true) {
      Serial1.print(qtr.calibratedMinimumOn[i]);
      Serial1.print(" ");
    }
  }
  if (DEBUG == true) {
    Serial1.println("");
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    int Valor = EEPROM.read(_Direccion);
    Valor <<=  8;
    _Direccion++;
    Valor += EEPROM.read(_Direccion);;
    _Direccion++;
    qtr.calibratedMaximumOn[i] = Valor;
    if (DEBUG == true) {
      Serial1.print(qtr.calibratedMaximumOn[i]);
      Serial1.print(" ");
    }
  }
  if (DEBUG == true)  {
    Serial1.println("");
  }
}

void Control_PD() {                             //Función de CONTROLADOR implementado
  bool edo = false;
  Posicion = qtr.readLine(sensorValues);  
  if (Posicion > 6999 || Posicion < 1) {
    PWM = 1, kp = 0, kd = 0;                //Modificar la variable PWM si desea aumentar la velocidad del carrito en curvas
    edo = true;
  }
  else {
    PWM = 125, kp = 3, kd = 5;                //Modificar la variable PWM si desea aumentar la velocidad del carrito en las rectas
  }

  float error = ((Posicion - Referencia) * 70.0) / 7000.0;
  Proporcional = error * kp;
  Derivativo = (error - error_Ant) * kd;
  int Salida_M1 = PWM - (Proporcional + Derivativo);
  int Salida_M2 = PWM + (Proporcional + Derivativo);

  if (DEBUG == true) {                        //Coloque a true el macro "DEBUG" para visualizar los valores en el monitor serial
    Serial1.print(Posicion);
    Serial1.print("\t");
    Serial1.print(error);
    Serial1.print("\t");
    Serial1.print(Proporcional);
    Serial1.print("\t");
    Serial1.print(Salida_M1);
    Serial1.print("\t");
    Serial1.println(Salida_M2);
  }
  if (edo == true) {
    Frenado(Salida_M1, Salida_M2);
  }
  else {
    Motor_PWM(Salida_M1 , Salida_M2);
  }
  error_Ant = error;
}


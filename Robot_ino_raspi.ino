/*************************************************************
 * Datos Entorno HUD del Robot 
 ************************************************************
Autor: Castillo Alvardo, David
Tema: Toma de Datos para el HUD del Robot asi como su control de mov
Finalidad: implementar arduino con Raspberry usando OpenCV en python 
Funcionamiento:
1. Toma de datos de los sensores para ser enviados al robot
2. Recepcion de datos del raspberry para dar ordenes al robot

NOTAS_MPU:
==========
Librerias I2C para controlar el mpu6050
la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.

NOTAS_HCRC04:
=============
0.034 cm/us = 340/1000000*100 Velocidad del sonido
entonces la distancia para un tiempo T con disparo y eco

D = t*0.034/2 = t/58.823

NOTA Motor : L298N
=========
      Forwards
L                 R
 ____^^^^^^^^^___
o|              |o
o|              |o
o| M[0]   M[1]  |o
o|              |o
o|______________|o
      Backwards

Nota WDT :
==========
15mS                           WDTO_15MS
30mS                           WDTO_30MS
60mS                           WDTO_60MS
120mS                          WDTO_120MS
250mS                          WDTO_250MS
500mS                          WDTO_500MS
1S                             WDTO_1S            
2S                             WDTO_2S
4S                             WDTO_4S
8S                             WDTO_8S

*/
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "math.h"
#include <avr/wdt.h>
/*--------------------------MPU 6050---------------------------------*/
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor_ang;
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;
double  ang_x, ang_y;
float ang_x_prev, ang_y_prev;
long tiempo_prev;
float dt;
int d_max;
/*---------------------------HC RC04---------------------------------*/
const int Trigger = 2;   //Pin digital 2 para el Trigger del sensor
const int Echo    = 3;   //Pin digital 3 para el Echo del sensor
long t;                  //timpo que demora en llegar el eco
long dist_obst;          //distancia en centimetros
int sum_d = 0;
int i = 0;
int j = 0;
int dist_obst_0;
int dis_val[5]={0,0,0,0,0}; //pos[0,1,2,3,4] 
long time_out;
/*---------------------------Control de Motor------------------------*/
//                        M1     M2  COnfiguracion de Pines L298N
int IN1[2]={0,0};       // 9     7
int IN2[2]={0,0};       // 8     6
int PWMp[2]={0,0};      // 11    5
int forWARDS  = 1; 
int backWARDS = 0;
int dir1 =0;
int dir2=0;
/*-----------------------------Serial JSON format---------------------*/
String data_send;
String status_dir1 = "\"wait\"";
String status_dir2 = "\"wait\"";
boolean read_up = false;
String data;
long tiempo_prev2;

/*=============================Interrupción Serial====================*/
void serialEvent(){
  while(Serial.available()){
    data = Serial.readString();
  }
  read_up = true;
}
/*==============================SETUP=================================*/
void setup(){
  wdt_disable();
  Serial.begin(115200);    //Iniciando puerto serial
  Serial.setTimeout(50);
  init_MPU_sensor(); 
  init_HC_RC04();
  init_Motor_control();
  wdt_enable(WDTO_1S);     // Iniciando el WDT
}
/*==============================LOOP==================================*/
void loop(){
  MPU_sensor();
  HC_RC04();
  Read_serial();
  Send_serial();


  /*
    Serial.print(ang_x); Serial.print("\t");
    Serial.print(ang_y); Serial.print("\t");
    //Enviar la distancia detectada cm
    Serial.print(dist_obst); Serial.println("\t");

    //Resetear el contador del WDT*/
    wdt_reset();
}
/*====================================================================*/

void init_MPU_sensor(){
  Wire.begin();           //Iniciando I2C  
  sensor_ang.initialize();    //Iniciando el sensor_ang
  if (sensor_ang.testConnection()) Serial.println("sensor_ang iniciado correctamente");
  else Serial.println("Error al iniciar el sensor_ang");
}

void init_HC_RC04(){
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
  d_max = 150; // distancia maxima detectada
  time_out = d_max*58.823;
}

void init_Motor_control(){
  //Motor 1
  IN1[0] = 9;
  IN2[0] = 8;
  PWMp[0] = 11;
  pinMode(IN1[0], OUTPUT);
  pinMode(IN2[0], OUTPUT);
  pinMode(PWMp[0], OUTPUT);
  // Motor 2
  IN1[1] = 7;
  IN2[1] = 6;
  PWMp[1] = 5;
  pinMode(IN1[1], OUTPUT);
  pinMode(IN2[1], OUTPUT);
  pinMode(PWMp[1], OUTPUT);
}

void MPU_sensor(){
  // Leer las aceleraciones y velocidades angulares
  sensor_ang.getAcceleration(&ax, &ay, &az);
  sensor_ang.getRotation(&gx, &gy, &gz);
  // Correxion de Offsets
  ax=ax-1277;
  ay=ay+428;
  az=az+1723;
  gx=gx+637;
  gy=gy-319;
  gz=gz-224;
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan2(ay,sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan2(-ax,sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complementario  
  ang_x = 0.98*(ang_x_prev+(gx/127)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/160)*dt) + 0.02*accel_ang_y;
   
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;
}

void HC_RC04(){
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);      //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);
  
  t = pulseIn(Echo, HIGH,time_out );    //obtenemos el ancho del pulso
  if (t == 0 ){ 
    dist_obst = d_max; // Tiempo mayor al esperado =  más distancia Dmáx
    } 
    else{
          dist_obst = t/58.823;       // Distancia en centimetros
        } 
  
  if (i <5){                  // Acumulamos 5 lecturas 
    dis_val[i] = dist_obst;   // para sacar la media
    i++;
  }
  if (i == 5){
    for (j = 0; j < 5; j++){  // calcula el promedio de mediciones
      sum_d += dis_val[j];
      dist_obst=sum_d/5;
    }
    for (j = 0; j < 4; j++){  // Desplaza los valores dentro del array
      dis_val[j] = dis_val[j+1]; 
    } 
    i--;
    sum_d = 0;
  }
  if (dist_obst > d_max ){ dist_obst = d_max; }
}

void Read_serial(){
 if(read_up){
  //delay(10);
  //String data = Serial.readString();
  // Primer valor / Direccion: forw[1] back[2] left[3] right[4]
  int index1= data.indexOf("=");
  index1 = index1 + 1;
  char value1 = data.charAt(index1);
  dir1= int(value1)-'0';

  // Segundo valor para establecer la velocidad
  int index2= data.lastIndexOf("=");
  index2 = index2 + 1;
  char value2 = data.charAt(index2);
  dir2= int(value2)-'0';

  /*Muestra valores de forma provisional*/
  Serial.print("Direccion"); Serial.print("\t");Serial.println(dir1); // adelante / atras
  Serial.print("PWM"); Serial.print("\t");Serial.println(dir2); // left / right
  
  // Solo entra a la funcion cuando hay una orden, ahorro de recursos
  Motor_action(dir1, dir2 );
  read_up = false;
 } 

}

void Motor_action(int direct,int pwmS ){
  pwmS=pwmS*255/9;
  status_dir2 = String(pwmS/255.0*100.0);
  switch (direct) {
      case 1: // FOrward
        shaftrev(IN1[0],IN2[0],PWMp[0],forWARDS, pwmS);
        shaftrev(IN1[1],IN2[1],PWMp[1],forWARDS, pwmS);
        status_dir1 = "\"FORWARD\"";
        break;
      case 2: // BackWard
        shaftrev(IN1[0],IN2[0],PWMp[0],backWARDS, pwmS);
        shaftrev(IN1[1],IN2[1],PWMp[1],backWARDS, pwmS);
        status_dir1 = "\"BACKWARD\"";
        break;
      case 4: // Left
        shaftrev(IN1[0],IN2[0],PWMp[0],backWARDS,pwmS);
        shaftrev(IN1[1],IN2[1],PWMp[1],forWARDS, pwmS);
        status_dir1 = "\"LEFT\"";
        break;
      case 3: // Right
        shaftrev(IN1[0],IN2[0],PWMp[0],forWARDS, pwmS);
        shaftrev(IN1[1],IN2[1],PWMp[1],backWARDS, pwmS);
        status_dir1 = "\"RIGHT\"";
        break;
      case 0: // Stop
        shaftrev(IN1[0],IN2[0],PWMp[0],3, 0);
        shaftrev(IN1[1],IN2[1],PWMp[1],3, 0);
        status_dir1 = "\"STOP\"";
        status_dir2 = "\"0\"";
        break;
      default: // Stop
        shaftrev(IN1[0],IN2[0],PWMp[0],3, 0);
        shaftrev(IN1[1],IN2[1],PWMp[1],3, 0);
        status_dir1 = "\"STOP\"";
        status_dir2 = "\"0\"";
        break;
  }
}

void shaftrev(int in1, int in2, int PWM, int sentido,int Wpulse){  
  switch (sentido) {
      case 0:
        digitalWrite(in2, HIGH);
        digitalWrite(in1, LOW);
        analogWrite(PWM,Wpulse);
        break;
      case 1:
        digitalWrite(in2, LOW);
        digitalWrite(in1, HIGH);
        analogWrite(PWM,Wpulse);
        break;
      case 3:
        digitalWrite(in2, LOW);
        digitalWrite(in1, LOW);
        analogWrite(PWM,Wpulse);
        break;
  }
}

void Send_serial(){
  // Envía datos cada 500ms 
  if ((millis()  - tiempo_prev2) > 500){
    //Enviando datos en formato JSON para el python
    data_send = String("{")+String("'ang_x':") + String(ang_x) + String(",") + String("'ang_y':") + String(ang_y) + String(",") + String("'dist':") + String(dist_obst) + String(",")+ String("'st1':") + status_dir1 + String(",")+ String("'st2':") + status_dir2 + String("}");
    Serial.println(data_send);
    tiempo_prev2 = millis();
  }
}
#include "TRSensors.h"
#define NUM_SENSORS 5

const int LEFT_B = A0;
const int LEFT_F = A1;
const int RIGHT_B = A3;
const int RIGHT_F = A2;

const float K_p = 0.2; 
const float K_i = 0.0001; //0.005;
const float K_d = 0.05; //-0.2;

int err_t_1 = 0; 
int err = 0; // 2000 <-> -2000 po prawej
int e_i = 0;
int d_err = 0;
int speed = 160;
TRSensors sens = TRSensors();

unsigned int sensorValues[NUM_SENSORS];

int PID = K_p*err + K_i*e_i + K_d*d_err;

void setup() {
  Serial.begin(9600);
  

  digitalWrite(RIGHT_B, 0);
  digitalWrite(LEFT_B, 0);
  digitalWrite(RIGHT_F, 1);
  digitalWrite(LEFT_F, 1);

  analogWrite(5, 0);
  analogWrite(6, 0);

  Serial.println("CALIBRATION...");
  for(int i= 0; i < 800 + 1; i++) {
    sens.calibrate();
  }
  Serial.println("CALIBRATON FINISHED");
  //for(int i= 0; i < 10 + 1; i++) {
  //  delay(200);
  //} 


  analogWrite(5, speed);
  analogWrite(6, speed);
} 
int Vr =0; 
int Vl =0;

void loop() {
  //Serial.println(err);
  

  e_i += err;
  err = sens.readLine(sensorValues) -2000;
  d_err = err - err_t_1;
    err_t_1 = err;

  PID = K_p * err + K_i * e_i + K_d * d_err;
  Vr = constrain(speed-PID, 0, 255);
  Vl = constrain(speed+PID, 0, 255);
  analogWrite(5, Vr);
  analogWrite(6, Vl);
  
}

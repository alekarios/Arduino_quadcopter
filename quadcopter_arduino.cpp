#include <Wire.h>
#include <Servo.h>
#include <WiFi.h>
#include <BlynkSimpleWifi.h>


int gyroResult[3], accelResult[3];
float timeStep = 0.02;
float biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ;
float pitchGyro = 0;
float pitchAccel = 0;
float pitchPrediction = 0; //Output of Kalman filter
float rollGyro = 0;
float rollAccel = 0;
float rollPrediction = 0;  //Output of Kalman filter
float giroVar = 0.1;
float deltaGiroVar = 0.1;
float accelVar = 5;
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float kx, kv;
unsigned long timer;

 void readFrom(byte device, byte fromAddress, int num, byte result[]) {
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
  }
  
  void getGyroscopeReadings(int gyroResult[]) {
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer);
  gyroResult[0] = (((int)buffer[0]) << 8 ) | buffer[1];
  gyroResult[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  gyroResult[2] = (((int)buffer[4]) << 8 ) | buffer[5];
} 

void getAccelerometerReadings(int accelResult[]) {
  byte buffer[6];
  readFrom(0x53,0x32,6,buffer);
  accelResult[0] = (((int)buffer[1]) << 8 ) | buffer[0];
  accelResult[1] = (((int)buffer[3]) << 8 ) | buffer[2];
    accelResult[2] = (((int)buffer[5]) << 8 ) | buffer[4];
}

float kp_pitch_control = 0.8;
float kp_roll_control = 0.8;
float ki_pitch_control = 0.03;
float ki_roll_control = 0.03;
float kd_pitch_control = 35;
float kd_roll_control = 35;
float  d_errory,last_errory,last_errorx,setpointy,setpointx,pitch_control,roll_control;
float   
y_filter_offset,x_filter_offset,total_pitchPrediction,last_error_pitch_rate,total_rollPre
diction,last_error_roll_rate;
double xRate, yRate, zRate,xRate_offset,yRate_offset,zRate_offset,y_filter,x_filter,j;
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
//

int x,C_x,C_y;
int values = 0;
char ssid[] = "alekarios";      //  ΤΟ ΔΙΚΤΥΟ ΣΥΝΔΕΣΗΣ ΤΗΣ WIFISHIELD
char pass[] = "oi5filoi";   // ΚΩΔΙΚΟΣ WPA TOY DIKTYOY
int keyIndex = 0;  
byte inchar,last_inchar;
int c;
char auth[] = "ccb7c53dd6d644928cd8dd71d0d2724e";

void setup() {
Wire.begin(); 
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  myservo1.attach(2);
  myservo2.attach(3);
  myservo3.attach(5);
  myservo4.attach(8);
  int totalGyroXValues = 0;
  int totalGyroYValues = 0;
  int totalGyroZValues = 0;
  int totalAccelXValues = 0;
  int totalAccelYValues = 0;
  int totalAccelZValues = 0;
  int i;
  
  writeTo(0x53,0x31,0x09); //Set accelerometer to 11bit, +/-4g
  writeTo(0x53,0x2D,0x08); //Set accelerometer to measure mode
  writeTo(0x68,0x16,0x1A); //Set gyro to +/-2000deg/sec and 98Hz low pass filter
  writeTo(0x68,0x15,0x09); //Set gyro to 100Hz sample rate
  
    for (i = 0; i < 50; i += 1) {
    getGyroscopeReadings(gyroResult);
    getAccelerometerReadings(accelResult);
    totalGyroXValues += gyroResult[0];
    totalGyroYValues += gyroResult[1];
    totalGyroZValues += gyroResult[2];
    totalAccelXValues += accelResult[0];
    totalAccelYValues += accelResult[1];
    totalAccelZValues += accelResult[2];
    delay(50);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;
  biasAccelX = totalAccelXValues / 50;
  biasAccelY = totalAccelYValues / 50;
  biasAccelZ = (totalAccelZValues / 50) – 256;
  
  for (j = 0; j < 50; j++){
  getGyroscopeReadings(gyroResult);
  getAccelerometerReadings(accelResult);
  
    pitchAccel = atan2((accelResult[1] - biasAccelY) / 256, (accelResult[2] -      
    biasAccelZ) / 256) * 360.0 / (2*PI);
    pitchGyro = pitchGyro + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
    pitchPrediction = pitchPrediction + ((gyroResult[0] - biasGyroX) / 14.375) * 
    timeStep;
	
	rollAccel = atan2((accelResult[0] - biasAccelX) / 256, (accelResult[2] - 
    biasAccelZ) / 256) * 360.0 / (2*PI);
    rollGyro = rollGyro - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep; 
    rollPrediction = rollPrediction - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep;
	
    Pxx += timeStep * (2 * Pxv + timeStep * Pvv);
    Pxv += timeStep * Pvv;
    Pxx += timeStep * giroVar;
    Pvv += timeStep * deltaGiroVar;
    kx = Pxx * (1 / (Pxx + accelVar));
    kv = Pxv * (1 / (Pxx + accelVar));
    pitchPrediction += (pitchAccel - pitchPrediction) * kx;
    rollPrediction += (rollAccel - rollPrediction) * kx;
    total_pitchPrediction += pitchPrediction;
    total_rollPrediction += rollPrediction;
    Pxx *= (1 - kx);
    Pxv *= (1 - kx);
    Pvv -= kv * Pxv;
    total_pitchPrediction /= 50;
	
	void loop() {
    Blynk.run(); 
    BLYNK_WRITE(V1);  
    compute_angle();
	
	void compute_angle(){
    timer = millis();
    getGyroscopeReadings(gyroResult);
    getAccelerometerReadings(accelResult);
	
	  pitchAccel = atan2((accelResult[1] - biasAccelY) / 256, (accelResult[2] - 
      biasAccelZ) / 256) * 360.0 / (2*PI);
      pitchGyro = pitchGyro + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
      pitchPrediction = pitchPrediction + ((gyroResult[0] - biasGyroX) / 14.375) * 
      timeStep;
	  
	  rollAccel = atan2((accelResult[0] - biasAccelX) / 256, (accelResult[2] - 
      biasAccelZ) / 256) * 360.0 / (2*PI);
      rollGyro = rollGyro - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep; 
      rollPrediction = rollPrediction - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep;
	  
	  Pxx += timeStep * (2 * Pxv + timeStep * Pvv);
      Pxv += timeStep * Pvv;
      Pxx += timeStep * giroVar;
      Pvv += timeStep * deltaGiroVar;
      kx = Pxx * (1 / (Pxx + accelVar));
      kv = Pxv * (1 / (Pxx + accelVar));
	  
	  pitchPrediction += (pitchAccel - pitchPrediction) * kx;
      rollPrediction += (rollAccel - rollPrediction) * kx;
	  
	  Pxx *= (1 - kx);
      Pxv *= (1 - kx);
      Pvv -= kv * Pxv;
	  
	   //xeisrismos gia pitch
      float input_pitch_rate = pitchPrediction + C_x ;
      float setpoint_pitch_rate = total_pitchPrediction;
      float error_pitch_rate = input_pitch_rate - setpoint_pitch_rate;
      float P_pitch_rate = kp_pitch_control * error_pitch_rate;
      I_pitch_rate += ki_pitch_control * error_pitch_rate;
      float  d_error_pitch_rate = (error_pitch_rate - last_error_pitch_rate) ;
      float  D_pitch_rate = kd_pitch_control * d_error_pitch_rate;
      float  C_pitch_rate = P_pitch_rate + I_pitch_rate + D_pitch_rate;
      last_error_pitch_rate = error_pitch_rate;
	  
	   //xeisrismos gia pitch
      float input_pitch_rate = pitchPrediction + C_x ;
      float setpoint_pitch_rate = total_pitchPrediction;
      float error_pitch_rate = input_pitch_rate - setpoint_pitch_rate;
      float P_pitch_rate = kp_pitch_control * error_pitch_rate;
      I_pitch_rate += ki_pitch_control * error_pitch_rate;
      float  d_error_pitch_rate = (error_pitch_rate - last_error_pitch_rate) ;
      float  D_pitch_rate = kd_pitch_control * d_error_pitch_rate;
      float  C_pitch_rate = P_pitch_rate + I_pitch_rate + D_pitch_rate;
      last_error_pitch_rate = error_pitch_rate;
	  
	  float  C_roll_rate = P_roll_rate + I_roll_rate + D_roll_rate  ;
      last_error_roll_rate = error_roll_rate
	  
	  myservo1.write(x-C_pitch_rate+C_roll_rate );  
      myservo2.write(x+C_pitch_rate+C_roll_rate );
      myservo3.write(x-C_pitch_rate-C_roll_rate );
      myservo4.write(x+C_pitch_rate-C_roll_rate );
	  
	  BLYNK_WRITE(V2) {
       C_x = param[0].asInt();
        C_y = param[1].asInt();
     }
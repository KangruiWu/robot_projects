#include <JY901.h>

/*
Test on Uno R3.
JY901    UnoR3
SDA <---> SDA
SCL <---> SCL
*/

//PID constants
double Kp = 5;
double Ki = 0;// 0.0003;
double Kd = 2;

// PID
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input1, output1,input2, output2, setPoint;
double cumError, rateError;


// IMU
double accX,accY,accZ;
double Setpoint;

// Motor A
int motor1Pin1 = 16;        // out1   18 -> 16
int motor1Pin2 = 4;        // out2  5 -> 4
int enable1Pin1= 17;   //19 -> 17

// Motor B
int motor1Pin3 = 27;        // out1
int motor1Pin4 = 14;        // out2
int enable1Pin2 = 12;       

//enable1 = 19, pin1 18, pin2 5
//pin3 = 27, pin4 = 14 , enable2=12

// Setting PWM properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int resolution = 8;
int dutyCycle = 200; // 0-255

void setup() {
  Setpoint = 179;
  
  //imu setup
  Serial.begin(115200);
  JY901.startIIC();
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);    // out1
  pinMode(motor1Pin2, OUTPUT);    // out2
  pinMode(enable1Pin1, OUTPUT);
  
  pinMode(motor1Pin3, OUTPUT);    // out1
  pinMode(motor1Pin4, OUTPUT);    // out2
  pinMode(enable1Pin2, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel0, freq, resolution);
  ledcSetup(pwmChannel1, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin1, pwmChannel0);
  ledcAttachPin(enable1Pin2, pwmChannel1);

  Serial.begin(115200);

  ledcWrite(pwmChannel0, dutyCycle); 
  ledcWrite(pwmChannel1, dutyCycle); 

  input1 = JY901.getRoll();  
  input2 = JY901.getRoll();  
  

}

void loop() {


  double gyroX = JY901.getGyroX();


  input1 = JY901.getRoll();               
  output1 = computePID(input1, gyroX);
  output1 = map(output1, 1300, 700, 150, 255);
  output1 = min(output1, 255.0);

  

  delay(1);

 
  
  // motor 1
  if ((input1 > 90) && (input1 < 180)){
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH); 
  }
  else if ((input1 > -180) && (input1 < -90)){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW); 
  }
  else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
  }

  
  ledcWrite(pwmChannel0, output1); 
  ledcWrite(pwmChannel1, output1);
  Serial.println(output1);
  delay(2);

}

double computePID(double inp, double vel){
  // Source: https://www.teachmemicro.com/arduino-pid-control-tutorial/
  
  currentTime = millis();                                    //get current time

  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation 
        
  error = setPoint - abs(inp);                               // determine error
  cumError += error * elapsedTime;                           // compute integral
  rateError = (0-vel);               // compute derivative

  double out = abs(Kp*error + Ki*cumError + Kd*rateError);        //PID output               

  lastError = error;                                         //remember current error
  previousTime = currentTime;                                //remember current time
  
  return out;                                                //have function return the PID output




  
}

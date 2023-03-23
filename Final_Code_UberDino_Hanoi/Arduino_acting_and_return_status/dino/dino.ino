// Communication buffers lengths
#define MAX_BUFF_LEN        255 
#define CMD_BUFF_LEN        6

// Satates of the LED
#define S_STATE             0       // Stay state
#define R_STATE             1       // Right state
#define L_STATE             2       // Left state
#define F_STATE             3       // Forward state
#define G_STATE             4       // Go to giver state
#define P_STATE             5       // pick up state
#define D_STATE             6       // drop gift state
#define B_STATE             7       // Back-up state
#define T_STATE             8       // turn PID state
#define DEFAULT_DELAY       100

// Motor A
int motor1Pin1 = 16;         // out1   18 -> 16
int motor1Pin2 = 4;         // out2  5 -> 4
int enable1Pin= 17;        //19 -> 17

// Motor B
int motor2Pin1 = 27;        // out1
int motor2Pin2 = 14;       // out2
int enable2Pin = 12;       

// Setting PWM properties
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 50; // 0-255

// Globals

int angle =0;
int angleStep = 5;

int angleMin =0;
int angleMax = 180;

// PID
#include <sstream>
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input1, output1,input2, output2, setPoint;
double cumError, rateError;
double Kp = 1;
double Ki = 0;// 0.0003;
double Kd = 8;
int output;

 

char c; // IN char
char str[CMD_BUFF_LEN];
uint8_t idx = 0; // Reading index

uint8_t state = S_STATE; // Default state
int delay_t = DEFAULT_DELAY; // Default blinking delay
unsigned long prev_time;

// IMU and angle vars
int cam_angle = 0; // Default blinking delay
int imu_angle;
int prev_imu_angle;
int desired_angle;
int difference;
float angle_error;
int counter;


// Func prototypes
uint8_t interpret(char);

// IMU
#include <JY901.h>
double Setpoint;

// HCSR04
const int trigPin = 5;
const int echoPin = 18;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;


// Lidar
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();


void setup() {
  // Config serial port
  Serial.begin(115200);

  // HCSR04 setup
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  //imu setup
  JY901.startIIC(); 
  //angle = JY901.getYaw();//forward: -107; left:-35; right: -160

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  // set Pin Mode
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);
  
  // put your setup code here, to run once:
  imu_angle = int(JY901.getYaw());

  ledcWrite(pwmChannel1, dutyCycle); 
  ledcWrite(pwmChannel2, dutyCycle); 
  // 
  prev_time = millis();

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // HCSR04
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  
  // Parse incoming command
  if(Serial.available() > 0){ // There's a command
    Serial.println("There's a command");
    
    c = Serial.read(); // Read one byte

    while(c != '\n'){
      str[idx++] = c; // Parse the string byte (char) by byte
      c = Serial.read(); // Read one byte
    }
    // Done reading
    Serial.println("Done reading");
    str[idx] = '\0'; // Convert it to a string
    
    // Determine nature of the command
    state = interpret(str[0]);
    
    Serial.println(state);
    // strtol(char*, Ref pointer, Base[Decimal-->10, Hex-->16, ...])
    cam_angle = strtoul(str+1, NULL, 10); // str+1 --> exclude the first char
    /* Some input checking could've been done here (like b15f2 --> invalid) */

    // Reset reading index 
    idx = 0;
  }
  else{ // No input from commander
    state = S_STATE;
  }
  
  int current_angle = 0;
  Serial.println("cam_angle, imu_angle, desired_angle");
  Serial.println(cam_angle);
  imu_angle = int(JY901.getYaw());
  Serial.println(imu_angle);
  current_angle = imu_angle + cam_angle + 180;
  desired_angle = current_angle % 360;
  if (cam_angle < 0){desired_angle = (desired_angle + 180) % 360;}
  else{desired_angle = desired_angle - 180;}
  Serial.println(desired_angle);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  
  // Main state machine
  switch(state){
    case S_STATE: // This is the default do nothing state
      //Serial.println("writting stay");
      dutyCycle = 0;
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, LOW); 
      ledcWrite(pwmChannel1, dutyCycle); 
      ledcWrite(pwmChannel2, dutyCycle);
      delay(delay_t);

      break;

    case T_STATE: // turn PID
      Serial.println("writting turn");

      Serial.println("Angle info");
      Serial.println(imu_angle);
      Serial.println(desired_angle);

      difference = computeError(imu_angle, desired_angle);

      counter = 0;

      while (abs(difference) > 5){
      difference = computeError(imu_angle, desired_angle);
      // PID
      Serial.println("difference");
      Serial.println(difference);
      Serial.println("In loop");
      output = computePID(desired_angle);
      //if (cam_angle < 0){difference = -difference;}
      if (difference > 0){
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      }
      else{
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor2Pin2, HIGH);
      digitalWrite(motor2Pin1, LOW);  
      }
      output = min(abs(output), 255);
      dutyCycle = map(output, 0,255,110,220);

      Serial.println("dutyCycle");
      Serial.println(dutyCycle);

      ledcWrite(pwmChannel1, dutyCycle); 
      ledcWrite(pwmChannel2, dutyCycle);
      delay(200);
      Serial.println("rotate");

      ledcWrite(pwmChannel1, 0); 
      ledcWrite(pwmChannel2, 0);
      if (counter >= 8){break;}
      counter +=1;

      delay(200);

      imu_angle = int(JY901.getYaw());
      Serial.println(imu_angle);
      }
      Serial.println("turned");
      break;

    case R_STATE: // turn right
      Serial.println("writting right");
      dutyCycle = map(cam_angle, -1,90,80,110);
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);

      ledcWrite(pwmChannel1, dutyCycle); 
      ledcWrite(pwmChannel2, dutyCycle);
      delay(delay_t);
      break;

    case L_STATE: // turn left
      Serial.println("writting left");
      dutyCycle = map(cam_angle, -1,90,80,110);
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);  

      ledcWrite(pwmChannel1, dutyCycle); 
      ledcWrite(pwmChannel2, dutyCycle);
      delay(delay_t);
      break;

    case F_STATE: // go forward
      Serial.println("writting forward");
      dutyCycle = cam_angle;       
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      
      ledcWrite(pwmChannel1, dutyCycle); 
      ledcWrite(pwmChannel2, dutyCycle);
      delay(delay_t);
      break;
      
    case G_STATE: // go to giver
      Serial.println("writting goTo");
      // Clears the trigPin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // HCSR04
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculate the distance
      distanceCm = duration * SOUND_SPEED/2;

      Serial.println(distanceCm);
      if (distanceCm <25){
      state = S_STATE;
      Serial.println("done");
      //Serial.println("Stop; state="); 
      //
      } else {
      dutyCycle = 140;       
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);

      ledcWrite(pwmChannel1, dutyCycle); 
      ledcWrite(pwmChannel2, dutyCycle);
      delay(400);
      ledcWrite(pwmChannel1, 0); 
      ledcWrite(pwmChannel2, 0);
      delay(100);
      Serial.println("go");
      //  
      }
      break;

    case B_STATE: // go to giver
      Serial.println("writting back-up");
      //*
      // Clears the trigPin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // HCSR04
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculate the distance
      distanceCm = duration * SOUND_SPEED/2;
      //*/

      //Serial.println(distanceCm);
      if (distanceCm > 50){
      state = S_STATE;
      Serial.println("done");
      ////Serial.println("Stop; state="); 
      //
      } else {
      dutyCycle = 140;  
      digitalWrite(motor1Pin1, LOW);     
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      

      ledcWrite(pwmChannel1, dutyCycle); 
      ledcWrite(pwmChannel2, dutyCycle);
      delay(400);
      ledcWrite(pwmChannel1, 0); 
      ledcWrite(pwmChannel2, 0);
      delay(100);
      Serial.println("back");
      //  
      }
      break;

    case P_STATE: // go to giver
      Serial.println("Check Lidar, pick-up");
      // Lidar
      VL53L0X_RangingMeasurementData_t measure;
      
      //Serial.print("Reading a measurement... ");
      //lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      Serial.println(lox.readRange());
      if (lox.readRange() < 60) {  // phase failures have incorrect data
      //Serial.println("Dectected a gift");
      //Serial.print("Distance (mm): "); 
      //Serial.println(measure.RangeMilliMeter);
      Serial.print("done"); 
      }
      else{Serial.println("pick");}
      break;
      
    case D_STATE: // drop gift
      Serial.println("Check Lidar, drop");
      Serial.println(lox.readRange());
      // Lidar
      //VL53L0X_RangingMeasurementData_t measure;
      //Serial.print("Reading a measurement... ");
      //lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      if (lox.readRange() > 60) {  // phase failures have incorrect data
      //Serial.println("Dectected a gift");
      //Serial.print("Distance (mm): "); 
      Serial.print("done"); 
      }
      else{Serial.println("drop");}
      break;
  }
}

// Func definition

uint8_t interpret(char c){
  switch(c){
    case 's': return S_STATE;

    case 't': return T_STATE;

    case 'r': return R_STATE;

    case 'l': return L_STATE;

    case 'f': return F_STATE;

    case 'g': return G_STATE;

    case 'p':  return P_STATE;

    case 'd':  return D_STATE;

    case 'b':  return B_STATE;
    
    default: Serial.println("UNKNOWN");
  }
  return state; // Don't change anything
}

double computePID(double angle){
  // Source: https://www.teachmemicro.com/arduino-pid-control-tutorial/
  
  double vel = 0;

  currentTime = millis();                                    //get current time

  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation 

  int prev_imu_amgle = imu_angle;
  imu_angle = int(JY901.getYaw());
  vel = computeError(imu_angle, prev_imu_angle)/elapsedTime;

  angle_error = computeError(angle, imu_angle);                               // determine error
  //angle_error -= 360*floor(0.5+angle_error/360);
  cumError += angle_error * elapsedTime;                           // compute integral
  rateError = (0-vel);               // compute derivative

  double out = Kp*angle_error + Ki*cumError + Kd*rateError;        //PID output               

  lastError = angle_error;                                         //remember current error
  previousTime = currentTime;                                //remember current time
  
  return out;       
}       

int computeError(int angle, int imu_angle){

  int copy_imu = imu_angle;
  int copy_angle = angle;

  copy_angle = copy_angle +180;
  copy_imu = copy_imu + 180;

  if(copy_angle < copy_imu){
      if(abs(copy_angle - copy_imu)<180){
        error = copy_angle - copy_imu;
      }
      else{
          error = copy_angle - copy_imu +360;}
  }
  else{
      if(abs(copy_angle - copy_imu)<180){
        error = copy_angle - copy_imu;
      }
      else{
          error = copy_angle - copy_imu - 360;}
  }

  return error;
}
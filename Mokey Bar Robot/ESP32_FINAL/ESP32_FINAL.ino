#include "ESPAsyncWebServer.h"
#include <WiFi.h>
#include <Servo.h>
#include <HCSR04.h> //a third-party lib for sonar
#include <Wire.h>
#include <JY901.h>

#define left_claw_pin 5
#define right_claw_pin 18
#define base_pin 19
#define tail_pin 23
#define pwmA 14
#define in1A 27
#define in2A 26
#define pwmB 32
#define in1B 25
#define in2B 33
#define trigger_pin 13
#define buttonL_pin 15
#define buttonR_pin 4

#define pi 3.1415
//SDA 21 SCL 22 for ESP
Servo left_claw, right_claw, base, tail;
AsyncWebServer server(80);
TaskHandle_t Task_0;
const char* ssid = "ESP32_FINAL";
const char* password = "123456789" ;
int handshake = 0, left_angle = 0, right_angle = 0, base_angle = 0, tail_angle = 0, left_LA_status = 0, right_LA_status = 0, bump_triggered = 0;
float left_distance = -1, right_distance = -1, IMU_angle = 0;
byte echoCount = 2;
byte* echoPins = new byte[echoCount] { 35, 34 }; //35left 34right

void setup() {
  pinMode(pwmA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  pinMode(buttonL_pin, INPUT);
  pinMode(buttonR_pin, INPUT);
  digitalWrite(pwmA, LOW);
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, HIGH);
  digitalWrite(pwmA, LOW);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, HIGH);
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  left_claw.attach(left_claw_pin);
  right_claw.attach(right_claw_pin);
  base.attach(base_pin);
  tail.attach(tail_pin);
  HCSR04.begin(trigger_pin, echoPins, echoCount);
  JY901.StartIIC();

  xTaskCreatePinnedToCore( //dual core function, single wifi core can't handle all work
    task_0, /* Function to implement the task */
    "Task 0", /* Name of the task */
    10000,  /* Stack size in words */ //increased size otherwise stack overflow, default 1000
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &Task_0,  /* Task handle. */
    0);

  server.on("/data", HTTP_POST, [](AsyncWebServerRequest * request) {}, NULL, [](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total)
  {

    String val = String((char *)data_in, len);
    long input = atof(val.c_str());
    switch (input)
    {
      case 0: //update request
        break;

      case -1: //A extend
        digitalWrite(pwmA, HIGH);
        digitalWrite(in1A, HIGH);
        digitalWrite(in2A, LOW);
        left_LA_status = 1;
        break;

      case -2: //A stop
        digitalWrite(pwmA, LOW);
        digitalWrite(in1A, HIGH);
        digitalWrite(in2A, HIGH);
        left_LA_status = 0;
        break;

      case -3: //A withdraw
        digitalWrite(pwmA, HIGH);
        digitalWrite(in1A, LOW);
        digitalWrite(in2A, HIGH);
        left_LA_status = -1;
        break;

      case -4: //B extend
        digitalWrite(pwmB, HIGH);
        digitalWrite(in1B, HIGH);
        digitalWrite(in2B, LOW);
        right_LA_status = 1;
        break;

      case -5: //B stop
        digitalWrite(pwmB, LOW);
        digitalWrite(in1B, HIGH);
        digitalWrite(in2B, HIGH);
        right_LA_status = 0;
        break;

      case -6: //B withdraw
        digitalWrite(pwmB, HIGH);
        digitalWrite(in1B, LOW);
        digitalWrite(in2B, HIGH);
        right_LA_status = -1;
        break;

      default: //10101010 LRTB 0-89 0-89 0-178 +-0-89

        base_angle = input % 100; //mid:90 forward>90
        input -= base_angle;
        input = abs(input);
        input = input / 100;
        tail_angle = input % 100;
        input = (input - tail_angle) / 100;
        right_angle = input % 100;
        input = (input - right_angle) / 100;
        left_angle = input % 100;

        base_angle += 95;
        tail_angle = (tail_angle - 10) * 2;
        right_angle -= 10;
        right_angle += 40;
        left_angle -= 10; //left close:145 open:55
        left_angle = 145 - left_angle;
        left_claw.write(left_angle);
        right_claw.write(right_angle); //right close 40 open 130
        base.write(base_angle);
        tail.write(tail_angle);
        delay(100); //a delay to make sure sonar readings are the latest after servo move
        break;

    }
    double* distances = HCSR04.measureDistanceCm();
    left_distance = distances[0];
    right_distance = distances[1];
    String response = String(left_angle) + "," + String(right_angle) + "," + String(base_angle) + "," + String(tail_angle) + "," + String(left_distance) + "," + String(right_distance) + "," + String(IMU_angle) + "," + String(bump_triggered);
    request->send_P(200, "text/plain", response.c_str());
    if (bump_triggered == 1) //reset button indicator
    {
      bump_triggered = 0;
    }
    Serial.println(response);
    Serial.println("");
  });

  server.begin();
}

void task_0(void*) {
  while (1)
  {
    //button trigger is only used when linear actuator on that side is extending (reach) since buttons are constantly pressed when a gripper is holding the bar
    if (digitalRead(buttonL_pin) == HIGH & left_LA_status == 1) //left button triggered
    {
      digitalWrite(pwmA, LOW);
      left_LA_status = 0;
      left_claw.write(145);
      left_angle = 145;
      right_claw.write(40);
      right_angle = 40;
      bump_triggered = 1;
    }
    if (digitalRead(buttonR_pin) == HIGH & right_LA_status == 1) //right button triggered
    {
      digitalWrite(pwmB, LOW);
      right_LA_status = 0;
      left_claw.write(145);
      left_angle = 145;
      right_claw.write(40);
      right_angle = 40;
      bump_triggered = 1;
    }
    //IMU reading is not inside webserver because it's slow
    JY901.GetAngle();
    IMU_angle=(float)JY901.stcAngle.Angle[0] / 32768 * 180;
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

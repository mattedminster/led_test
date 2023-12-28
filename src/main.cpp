#include <Arduino.h>
#include <WiFi.h>
#include "esp_mac.h"
#include "driver/mcpwm.h"

#define LED_R 14
#define LED_G 27
#define LED_B 32
#define LED_W 33

#define CH_R 0
#define CH_G 1
#define CH_B 2
#define CH_W 3

volatile uint16_t p1, p2, p3, p4;
volatile uint16_t in0, in1, in2, in3;

int red, green, blue;

WiFiServer server(80);
String header;

static const char *TAG = "platform";

void IRAM_ATTR capturePWM(uint8_t pin, volatile uint16_t &p, volatile uint16_t &in) {
  unsigned long time = micros();

  if (digitalRead(pin) == HIGH) {
    p = time;
  } else if (p != 0) {  // Ensure a rising edge has been captured
    in = (time > p) ? (time - p) : (ULONG_MAX - p + time);
    p = 0;  // Reset to avoid false readings
  }
}

void IRAM_ATTR pwm5Capture() {
  capturePWM(4, p1, in0);
}

void IRAM_ATTR pwm6Capture() {
  capturePWM(22, p2, in1);
}

void IRAM_ATTR pwm7Capture() {
  capturePWM(23, p3, in2);
}

void IRAM_ATTR pwm8Capture() {
  capturePWM(34, p4, in3);
}

void breathe_once_ledc(uint8_t chan){
  for (uint16_t i = 0; i < 2048; i++) {
    ledcWrite(chan, i >= 1024 ? 2047 - i : i);
    delayMicroseconds(300);
  }
}

void led_writeall(int red, int green, int blue, int white){
  ledcWrite(CH_R, red);
  ledcWrite(CH_G, green);
  ledcWrite(CH_B, blue);
  ledcWrite(CH_W, white);
}

void breathe_rgb(){
  for (int x = 0; x < 3072; x++) {
    if (x <= 1023) {
      red = 1023 - x;
      green = x;
      blue = 0;
    } else if (x <= 2047) {
      red = 0;
      green = 1023 - (x - 1024);
      blue = x - 1024;
    } else {
      red = x - 2048;
      green = 0;
      blue = 2047 - (x - 2048);
    }
    led_writeall(red, green, blue, 0);
    delay(3);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  delay(1000);

  ledcAttachPin(LED_R, CH_R);
  ledcAttachPin(LED_G, CH_G);
  ledcAttachPin(LED_B, CH_B);
  ledcAttachPin(LED_W, CH_W);

  ledcSetup(CH_R, 15000, 10);
  ledcSetup(CH_G, 15000, 10);
  ledcSetup(CH_B, 15000, 10);
  ledcSetup(CH_W, 15000, 10);

  pinMode(4, INPUT); // red pwm input
  pinMode(22, INPUT); // green pwm input
  pinMode(23, INPUT); // blue pwm input
  pinMode(34, INPUT); // white pwm input

  attachInterrupt(digitalPinToInterrupt(4), pwm5Capture, CHANGE);
  attachInterrupt(digitalPinToInterrupt(22), pwm6Capture, CHANGE);
  attachInterrupt(digitalPinToInterrupt(23), pwm7Capture, CHANGE);
  attachInterrupt(digitalPinToInterrupt(34), pwm8Capture, CHANGE);

  Serial.println("3DR QuadZero PRO Light-show-drone");
  breathe_once_ledc(CH_R);
  breathe_once_ledc(CH_G);
  breathe_once_ledc(CH_B);
  breathe_once_ledc(CH_W);
  breathe_rgb();
  led_writeall(0, 0, 0, 0);
  ledcWrite(CH_W, 1023);
  delay(80);
  ledcWrite(CH_W, 0);
  delay(340);
  ledcWrite(CH_W, 1023);
  delay(100);
  ledcWrite(CH_W, 0);
  delay(400);
  ledcWrite(CH_W, 60);
  delay(3000); // Allow for autopilot boot
  ledcWrite(CH_W, 0);
  led_writeall(0, 0, 0, 0);
  Serial.println("Starting PWM passthrough mode.");
}

void loop() {
  noInterrupts();
  uint16_t localIn0 = in0;
  uint16_t localIn1 = in1;
  uint16_t localIn2 = in2;
  uint16_t localIn3 = in3;
  interrupts();

  // Serial.print("R: ");
  // Serial.print(localIn0);
  // Serial.print(" G: ");
  // Serial.print(localIn1);
  // Serial.print(" B: ");
  // Serial.print(localIn2);
  // Serial.print(" W: ");
  // Serial.println(localIn3);

  int valueR = map(in0, 1000, 2000, 0, 1023);
  valueR = constrain(valueR, 0, 1023);

  int valueG = map(in1, 1000, 2000, 0, 1023);
  valueG = constrain(valueG, 0, 1023);

  int valueB = map(in2, 1000, 2000, 0, 1023);
  valueB = constrain(valueB, 0, 1023);

  int valueW = map(in3, 1000, 2000, 0, 1023);
  valueW = constrain(valueW, 0, 1023);

  ledcWrite(CH_R, valueR);
  ledcWrite(CH_G, valueG);
  ledcWrite(CH_B, valueB);
  ledcWrite(CH_W, valueW);

}
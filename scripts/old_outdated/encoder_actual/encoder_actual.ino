#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#define EncoderPinL1 2 // interrupt
#define EncoderPinL2 4
#define EncoderPinR1 3 // interrupt
#define EncoderPinR2 5

//Initialize variable
long countL = 0; // Encoder count for left side
long countR = 0; // Encoder count for right side

void setup() {
  Serial.begin(9600);

  // Initialize Encoder Pins
  pinMode(EncoderPinL1, INPUT);
  pinMode(EncoderPinL2, INPUT);
  pinMode(EncoderPinR1, INPUT);
  pinMode(EncoderPinR2, INPUT);

  // Initialize Pin States
  digitalWrite(EncoderPinL1, LOW);
  digitalWrite(EncoderPinL2, LOW);
  digitalWrite(EncoderPinR1, LOW);
  digitalWrite(EncoderPinR2, LOW);

  // Initialize Interrupt 
  attachInterrupt(0, readEncoderL, CHANGE); // 0 = pin2, 1 = pin3
  attachInterrupt(1, readEncoderR, CHANGE);
}

void loop() {
  Serial.println(String(countL) + "," + String(countR));
  delay(10);
  
}

void readEncoderL() {
  // triggered from left encoder changing
  if(digitalRead(EncoderPinL1) == digitalRead(EncoderPinL2)) {
    countL--;
  } else {
    countL++;
  }
}

void readEncoderR() {
  // triggered from right encoder changing
  if(digitalRead(EncoderPinR1) == digitalRead(EncoderPinR2)) {
    countR--;
  } else {
    countR++;
  }
}


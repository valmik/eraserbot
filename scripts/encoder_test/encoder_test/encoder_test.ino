#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#define EncoderPinA 2 // interrupt
#define EncoderPinB 5

//Initialize variable
long counts = 0; //Encoder counts

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Initialize Encoder Pins
  pinMode(EncoderPinA, INPUT);
  pinMode(EncoderPinB, INPUT);

  // Initialize Pin States
  digitalWrite(EncoderPinA, LOW);
  digitalWrite(EncoderPinB, LOW);

  // Initialize Interrupt 
  attachInterrupt(0, readEncoder, CHANGE); // O = pin2, 1 = pin3
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(counts);
}

void readEncoder() // triggered by encoder change
{
  if(digitalRead(EncoderPinB) == digitalRead(EncoderPinA) )
  {
    counts = counts - 1;
  }
  else
  {
    counts = counts + 1;
  }
}

void 


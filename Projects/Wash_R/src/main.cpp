/*
  To upload through terminal you can use: curl -u admin:admin -F "image=@firmware.bin" esp8266-webupdate.local/firmware
*/
#include <Arduino.h>

#define R1_PIN D1
#define R2_PIN D2
#define R3_PIN D3
#define R4_PIN D4
#define R5_PIN D5
#define R6_PIN D6
#define R7_PIN D7
#define R8_PIN D8

void setup(void) {

  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting Sketch...");


  pinMode(R1_PIN,OUTPUT);
  pinMode(R2_PIN,OUTPUT);
  pinMode(R3_PIN,OUTPUT);
  pinMode(R4_PIN,OUTPUT);
  pinMode(R5_PIN,OUTPUT);
  pinMode(R6_PIN,OUTPUT);
  pinMode(R7_PIN,OUTPUT);
  pinMode(R8_PIN,OUTPUT);
  digitalWrite(R1_PIN,HIGH);
  digitalWrite(R2_PIN,HIGH);
  digitalWrite(R3_PIN,HIGH);
  digitalWrite(R4_PIN,HIGH);
  digitalWrite(R5_PIN,HIGH);
  digitalWrite(R6_PIN,HIGH);
  digitalWrite(R7_PIN,HIGH);
  digitalWrite(R8_PIN,HIGH);


}

void loop(void) {
  


  
  digitalWrite(R1_PIN,HIGH);
  digitalWrite(R2_PIN,HIGH);
  digitalWrite(R3_PIN,HIGH);
  digitalWrite(R4_PIN,HIGH);
  digitalWrite(R5_PIN,HIGH);
  digitalWrite(R6_PIN,HIGH);
  digitalWrite(R7_PIN,HIGH);
  digitalWrite(R8_PIN,HIGH);
  Serial.println("HIGH DONE");
  delay(5000);
  digitalWrite(R1_PIN,LOW);
  digitalWrite(R2_PIN,LOW);
  digitalWrite(R3_PIN,LOW);
  digitalWrite(R4_PIN,LOW);
  digitalWrite(R5_PIN,LOW);
  digitalWrite(R6_PIN,LOW);
  digitalWrite(R7_PIN,LOW);
  digitalWrite(R8_PIN,LOW);
  Serial.println("LOW DONE");
  delay(5000);
  

  

}
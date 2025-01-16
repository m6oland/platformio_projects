/*
 Blink led on PIN0
 by Mischianti Renzo <http://www.mischianti.org>
 https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/
*/

#include <Arduino.h>
#include <PCF8574.h>

// Set i2c address
PCF8574 pcf8574_1(0x20);
PCF8574 pcf8574_2(0x21);

int buttonStatus = 0;

void setup()
{
	Serial.begin(115200);
  pinMode(A0, INPUT);
  pinMode(D8,INPUT_PULLUP); //button input

	// Set pinMode to OUTPUT
	for (int i = 0; i<=7; i++){
    pcf8574_1.pinMode(i, OUTPUT);
    pcf8574_2.pinMode(i, OUTPUT);
  }
  
	//pcf8574.pinMode(P1, INPUT);
	pcf8574_1.begin();
  pcf8574_2.begin();

	// Write all Pins HIGH (off)
	for (int i = 0; i<=7; i++){
    pcf8574_1.digitalWrite(i, HIGH);
    pcf8574_2.digitalWrite(i, HIGH);
  }

}

void loop()
{
	
    if (digitalRead(D8)== HIGH){
      if (buttonStatus == 0 ) {
        buttonStatus = 1;
      }
      else if (buttonStatus == 1  ) {
        buttonStatus = 0;
      }      
    }


  
  if (buttonStatus == 1) {
    pcf8574_1.digitalWrite(0, LOW);
    delay(1000);
    pcf8574_1.digitalWrite(7, LOW);
    delay(1000);
    pcf8574_2.digitalWrite(0, LOW);
    delay(1000);
    pcf8574_2.digitalWrite(7, LOW);  
  }
  else if (buttonStatus == 0){
  	delay(1000);
    pcf8574_1.digitalWrite(0, HIGH);
    delay(1000);
    pcf8574_1.digitalWrite(7, HIGH);
    delay(1000);
    pcf8574_2.digitalWrite(0, HIGH);
    delay(1000);
    pcf8574_2.digitalWrite(7, HIGH);
    delay(1000);  
  }

  Serial.println(digitalRead(D8));
}
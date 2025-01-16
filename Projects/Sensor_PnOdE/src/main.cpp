#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/**************************** PIN DEFINITIONS *******************************************/
#define SCL_PIN 16 //GPIO5 D1 Pin for I2C Clock BME
#define SDA_PIN 13 //GPIO4 D2 Pin for I2C Data BME
#define MOT_PIN D5 //GPIO12 D6 Pin for Motion Sensor 5Volt
#define LIT_PIN A0 // A0 Pin for Light Sensor 3 Volt
#define DOWN_PIN 0 //Expander Pin PO for door down position


Adafruit_BME280 bme;


void setup() {
  // put your setup code here, to run once:
  Wire.begin(SDA_PIN,SCL_PIN);
 
  Serial.begin(115200);
  Serial.println("\nROSEnsor PnOdE");

  bme.begin(0x76); 
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println(bme.readTemperature());
}


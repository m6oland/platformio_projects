 

#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library
#include <PubSubClient.h>         //MQTT
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <pcf8574_esp.h>

 
////**********START CUSTOM PARAMS******************//
////Define parameters for the http firmware update // which we aren't using 
const char* host = "Garage1ESP";
const char* ssid = "BoLand";
const char* password = "6oland22";

Adafruit_BME280 bme; // I2C
bool status;
 
/**************************** PIN DEFINITIONS *******************************************/
#define RELAY_PIN D7 //GPIO 0 D3 Pin for Relay Activation
#define SCL_PIN 5 //GPIO5 D1 Pin for I2C Clock BME
#define SDA_PIN 4 //GPIO4 D2 Pin for I2C Data BME
#define MOT_PIN D5 //GPIO12 D6 Pin for Motion Sensor 5Volt
#define LIT_PIN A0 // A0 Pin for Light Sensor 3 Volt
#define DOWN_PIN 0 //Expander Pin PO for door down position
#define UP_PIN 1 //Expander Pin P1 for door up position
#define ENTRY_PIN 2 //Expander Pin P2 for entry door sensor

/**************************** SENSOR DEFINITIONS *******************************************/
float ldrValue;
int LDR;
float calcLDR;
float diffLDR = 25;

float diffTEMP = 0.2;
float tempValue;

float diffHUM = 1;
float humValue;

float diffPRES = 50;
float presValue;

int pirValue;
int pirStatus;
String motionStatus;

int downState;
int downStatus;
int upState;
int upStatus;
char* openStatus = "moving";

int entryState;

char* entryStatus = "open";

char message_buff[100];

int calibrationTime = 0;

const int BUFFER_SIZE = 300;

#define MQTT_MAX_PACKET_SIZE 512

static bool eth_connected = false;
 
//Define your own MQTT 
#define door_topic "sensor/scarage/state1" //you can change this name, but make sure you "replace all"
#define entry_topic "sensor/scarage/entry" //you can change this name, but make sure you "replace all"
//#define temp_topic "sensor/garage/temperature" //Temp from BME sensor
//#define pres_topic "sensor/garage/pressure" //Pressure from BME sensor
//#define humid_topic "sensor/garage/humidity" //Humidity ftom BME sensor
//#define light_topic "sensor/garage/lumens" //Light from light sensor
//#define motion_topic "sensor/garage/motion" //Motion from motion sensor

#define mqtt_server "192.168.1.150"  //This is what you set up in HA. 
#define button_topic "sensor/scarage/action1" //you can change this name, but make sure you "replace all"
#define sensor_state_topic "sensor/scarage"
const char* mqtt_user = "mosquito"; //This is what you set up in HA. 
const char* mqtt_pass = "D1337o"; //This is what you set up in HA. 
 
//************END CUSTOM PARAMS********************//
//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;
 
WiFiClient espCscarage1;
 
//Initialize MQTT
PubSubClient client(espCscarage1);

/*  We need to set up the I2C-bus for the library to use */
TwoWire testWire;

//Set i2c address
PCF857x pcf8574(0x24, &testWire);

//Setup Variables
String switch1;
String strTopic;
String strPayload;
char* last_state = "";
char* last_entry = "";
 
 
void callback(char* topic, byte* payload, unsigned int length) {
  //if the 'garage/button' topic has a payload "OPEN", then 'click' the relay
  payload[length] = '\0';
  strTopic = String((char*)topic);
  if (strTopic == button_topic)
  {
    switch1 = String((char*)payload);
    Serial.println(switch1);
    if (switch1 == "OPEN")
    {
      //'click' the relay
      Serial.println("ON");
      digitalWrite(RELAY_PIN, HIGH);
      delay(600);
      digitalWrite(RELAY_PIN, LOW);
    }
  }
}



void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  //Reconnect to Wifi and to MQTT. If Wifi is already connected, then autoconnect doesn't do anything.
  Serial.print("Attempting MQTT connection...");
  if (client.connect(host, mqtt_user, mqtt_pass)) {
    Serial.println("connected");
    client.subscribe("sensor/scarage/#");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
  }
}


/*
 * Calculate Heat Index value AKA "Real Feel"
 * NOAA heat index calculations taken from
 * http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
 */
float calculateHeatIndex(float humidity, float temp) {
  float heatIndex= 0;
  if (temp >= 80) {
    heatIndex = -42.379 + 2.04901523*temp + 10.14333127*humidity;
    heatIndex = heatIndex - .22475541*temp*humidity - .00683783*temp*temp;
    heatIndex = heatIndex - .05481717*humidity*humidity + .00122874*temp*temp*humidity;
    heatIndex = heatIndex + .00085282*temp*humidity*humidity - .00000199*temp*temp*humidity*humidity;
  } else {
     heatIndex = 0.5 * (temp + 61.0 + ((temp - 68.0)*1.2) + (humidity * 0.094));
  }

  if (humidity < 13 && 80 <= temp <= 112) {
     float adjustment = ((13-humidity)/4) * sqrt((17-abs(temp-95.))/17);
     heatIndex = heatIndex - adjustment;
  }

  return heatIndex;
}

/********************************** START SEND STATE*****************************************/
void sendState() {
  StaticJsonDocument<BUFFER_SIZE> doc;

  //JsonObject root = doc.as<JsonObject>();

  doc["humidity"] = (String)humValue;
  doc["motion"] = (String)motionStatus;
  doc["ldr"] = (String)LDR;
  doc["temperature"] = (String)tempValue;
  doc["heatIndex"] = (String)calculateHeatIndex(humValue, tempValue);
  //root["garageDoor"] = (String)openStatus;
  doc["pressure"] = (String)presValue;
  
  
  //char buffer[measureJson(doc)+1];
  char buffer[BUFFER_SIZE];
  serializeJson(doc, buffer);
 

  Serial.println(buffer);
  client.publish(sensor_state_topic, buffer, true);

}

/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}



void setup() {
  //Set Relay(output) and Door(input) pins
  pinMode(RELAY_PIN, OUTPUT);
  //digitalWrite(RELAY_PIN, HIGH);
  pinMode(SCL_PIN, INPUT);
  pinMode(SDA_PIN, INPUT);
  pinMode(MOT_PIN, INPUT_PULLUP);
  pinMode(LIT_PIN, INPUT);
  
  //Specsheets say PCF8574 is officially rated only for 100KHz I2C-bus
  testWire.begin(SCL_PIN,SDA_PIN);
  testWire.setClock(100000L);
  pcf8574.begin();

  //pcf8574.pinMode(DOWN_PIN, INPUT);
  //pcf8574.pinMode(UP_PIN, INPUT);
  //pcf8574.pinMode(ENTRY_PIN, INPUT);
  
  //pcf8574.write(1,HIGH);
  

  status = bme.begin(0x76);
  pcf8574.begin();
  
  
  Serial.begin(115200);

  setup_wifi();


  client.setServer(mqtt_server, 1883); //1883 is the port number you have forwared for mqtt messages. You will need to change this if you've used a different port 
  client.setCallback(callback); //callback is the function that gets called for a topic sub  

}


void loop() {
  //If MQTT client can't connect to broker, then reconnect
  if (!client.connected()) {
    reconnect();
  }

  float newTempValue = (bme.readTemperature() * 9/5) + 32 ; //to use celsius remove the true text inside the parentheses  
  float newHumValue = bme.readHumidity();  //dht.readHumidity();
  float newPresValue = bme.readPressure();



  client.loop(); //the mqtt function that processes MQTT messages

     //PIR CODE
    pirValue = digitalRead(MOT_PIN); //read state of the motion sensor
    //Read expander pins
    //PCF8574::DigitalInput di = pcf8574.digitalReadAll();
    downState = pcf8574.read(DOWN_PIN);
    upState = pcf8574.read(UP_PIN);
    entryState = pcf8574.read(ENTRY_PIN);


   
    
    last_state = openStatus;
    if (downState == 0 && upState == 1 ) {
      openStatus = "open";
      downStatus = 0;
      upStatus = 1;
    }
    else if (downState == 1  && upState == 0) {
      openStatus = "closed";
      downStatus = 1;
      upStatus = 0;
    }
    else {
      openStatus = "moving";
      upStatus = 0;
      downStatus= 0;
    }

 // Serial.println((String)"downState:"+downState+" downStatus:"+downStatus+" upState:"+upState+" upStatus:"+upStatus+" openStatus:"+openStatus);  

    if (last_state != openStatus) {
       client.publish(door_topic, openStatus ,true);
       Serial.println(openStatus);
    }
    
    last_entry = entryStatus;  
    if (entryState == 1) {
      entryStatus = "closed";
    }
    else if (entryState == 0) {
      entryStatus= "open";
    }

    if (last_entry != entryStatus) {
      client.publish(entry_topic, entryStatus, true);
    }


//int entryState;
//char* last_entry = "";
//char* entryStatus = "";
      
    if (pirValue == LOW && pirStatus != 1) {
      motionStatus = "standby";
      sendState();
      pirStatus = 1;
    }

    else if (pirValue == HIGH && pirStatus != 2) {
      motionStatus = "motion detected";
      sendState();
      pirStatus = 2;
    } 

    if (checkBoundSensor(newTempValue, tempValue, diffTEMP)) {
      tempValue = newTempValue;
      sendState();
    }

    if (checkBoundSensor(newHumValue, humValue, diffHUM)) {
      humValue = newHumValue;
      sendState();
    }

    if (checkBoundSensor(newPresValue, presValue, diffPRES)) {
      presValue = newPresValue;
      sendState();
    }

    int newLDR = analogRead(LIT_PIN);

    if (checkBoundSensor(newLDR, LDR, diffLDR)) {
      LDR = newLDR;
      sendState();
    }

//Serial.println((String)"Temp: "+newTempValue+" Humid: "+newHumValue+" Press:"+newPresValue+" LDR: "+newLDR);  
//Serial.println((String)"Temp: "+tempValue+" Humid: "+humValue+" Press:"+presValue+" LDR: "+LDR); 
}





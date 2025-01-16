 

#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library
#include <PubSubClient.h>         //MQTT
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>


////**********START CUSTOM PARAMS******************//
////Define parameters for the http firmware update // which we aren't using 
const char* host = "Rose1ESP";
const char* ssid = "NoLANd";
const char* password = "8c3vbwJse8";

Adafruit_BME280 bme; // I2C
bool status;
 
/**************************** PIN DEFINITIONS *******************************************/
#define SCL_PIN D1 //GPIO5 D1 Pin for I2C Clock BME
#define SDA_PIN D2 //GPIO4 D2 Pin for I2C Data BME
#define MOT_PIN D5 //GPIO12 D6 Pin for Motion Sensor 5Volt
#define LIT_PIN A0 // A0 Pin for Light Sensor 3 Volt

/**************************** SENSOR DEFINITIONS *******************************************/
float ldrValue;
int LDR;
float calcLDR;
float diffLDR = 25;

float diffTEMP = 0.2;
float tempValue = 500;

float diffHUM = 1;
float humValue = 500;

float diffPRES = 50;
float presValue = 500;

int pirValue;
int pirStatus;
String motionStatus;

char message_buff[100];

int calibrationTime = 0;

const int BUFFER_SIZE = 300;

#define MQTT_MAX_PACKET_SIZE 512

//Define your own MQTT 
//#define temp_topic "sensor/garage/temperature" //Temp from BME sensor
//#define pres_topic "sensor/garage/pressure" //Pressure from BME sensor
//#define humid_topic "sensor/garage/humidity" //Humidity ftom BME sensor
//#define light_topic "sensor/garage/lumens" //Light from light sensor
//#define motion_topic "sensor/garage/motion" //Motion from motion sensor

#define mqtt_server "192.168.1.150"  //This is what you set up in HA. 
#define button_topic "sensor/rose1/action1" //you can change this name, but make sure you "replace all"
#define sensor_state_topic "sensor/rose1"
const char* mqtt_user = "mosquito"; //This is what you set up in HA. 
const char* mqtt_pass = "D1337o"; //This is what you set up in HA. 
 
//************END CUSTOM PARAMS********************//
//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;
 
WiFiClient Rose1ESP;
 
//Initialize MQTT
PubSubClient client(Rose1ESP);

//Setup Variables
String switch1;
String strTopic;
String strPayload;
char* last_state = "";
char* last_entry = "";
 
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

void callback(char* topic, byte* payload, unsigned int length) {
  //if the 'garage/button' topic has a payload "OPEN", then 'click' the relay
  payload[length] = '\0';
  strTopic = String((char*)topic);

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

  JsonObject root = doc.to<JsonObject>();

  root["humidity"] = (String)humValue;
  root["motion"] = (String)motionStatus;
  root["ldr"] = (String)LDR;
  root["temperature"] = (String)tempValue;
  root["heatIndex"] = (String)calculateHeatIndex(humValue, tempValue);
  //root["garageDoor"] = (String)openStatus;
  root["pressure"] = (String)presValue;
  
  
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
  pinMode(MOT_PIN, INPUT_PULLUP);
  pinMode(LIT_PIN, INPUT);
  
  //Specsheets say PCF8574 is officially rated only for 100KHz I2C-bus
  Wire.begin();

  //pcf8574.pinMode(DOWN_PIN, INPUT);
  //pcf8574.pinMode(UP_PIN, INPUT);
  //pcf8574.pinMode(ENTRY_PIN, INPUT);
  
  //pcf8574.write(1,HIGH);
  

  status = bme.begin(0x76);

  
  
  Serial.begin(115200);

  setup_wifi();

  Serial.println(status);


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
  
  //presValue = bme.readPressure();
  //tempValue = bme.readTemperature();
  //humValue = bme.readHumidity();
  //sendState();

  //Serial.println(bme.readTemperature());
  //Serial.println(bme.readHumidity());
  //Serial.println(bme.readPressure());

  client.loop(); //the mqtt function that processes MQTT messages

     //PIR CODE
    pirValue = digitalRead(MOT_PIN); //read state of the motion sensor
      
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

}





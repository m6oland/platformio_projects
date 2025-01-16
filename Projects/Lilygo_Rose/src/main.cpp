#include <Arduino.h>
#include <ETH.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>         //MQTT
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <NimBLEDevice.h>
#include <NimBLEBeacon.h>

#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00) >> 8) + (((x)&0xFF) << 8))

/**************************** PIN DEFINITIONS UEXT*******************************************/
#define LIT_PIN 36  //PIN 1   Light Sensor                   GPIO36                    BLUE
#define DOOR_PIN 39 //PIN 2   Reed Switch for Door Sensor    GPIO39                    GREEN
//////////////////////PIN 3   Input Only                     GPIO34                    YELLOW
//////////////////////PIN 4   Input Only Cap Touch ADC       GPIO35                    ORANGE
#define SCL_PIN 16  //PIN 5   Pin for I2C Clock              GPIO16                    RED
#define SDA_PIN 32  //PIN 6   Pin for I2C Data               GPIO32                    BROWN
//////////////////////PIN 7   Cap Touch ADC                  GPIO33                    BLACK
//////////////////////PIN 8   DNU BOOT                       GPIO12                    WHITE
#define TUCH_PIN 4  //PIN 9   Capacitive Touch Pin           GPIO04                    GREY
//////////////////////PIN 10  PWM Cap Touch                  GPIO15                    PURPLE
//////////////////////PIN 11  DNU LED                        GPIO02                    BLUE
#define MOT_PIN 14  //PIN 12  Motion Sensor                  GPIO14                    GREEN
//////////////////////PIN 13                                 3V3                       YELLOW
//////////////////////PIN 14                                 GROUND                    ORANGE
//////////////////////PIN 15                                 3V3                       RED
//////////////////////PIN 16                                 GROUND                    BROWN


static bool eth_connected = false;

int scanTime = 1; //In seconds
BLEScan* pBLEScan;
static BLEAddress *pServerAddress;

String knownAddresses[] = {"6e:46:6a:e1:d4:dc","61741734-ad34-4ccf-9122-05b9912524c1"};

//Define MQTT 
#define mqtt_server "192.168.1.150"  //This is what you set up in HA. 
#define sensor_state_topic "sensor/LilyRose00"
#define sensor_command_topic "sensor/LilyRose00/#"
const char* mqtt_user = "mosquito"; //This is what you set up in HA. 
const char* mqtt_pass = "D1337o"; //This is what you set up in HA. 
////Define parameters for the http firmware update // which we aren't using 
const char* host = "LilyRose00";

WiFiClient espClient;
PubSubClient client(espClient);
WebServer server(80);

Adafruit_BME280 bme;

float bleValue = -99;

float ldrValue;
int LDR;
float calcLDR;
float diffLDR = 25;

float diffTEMP = 0.2;
float tempValue;
float tempAdjust = 8.5;

float diffHUM = 1;
float humValue;

float diffPRES = 50;
float presValue;

int pirValue;
int pirStatus;
char motionStatus[16] = "";

int doorValue;
int doorStatus;
char openStatus[16] = "";

int tuchValue;
int tuchStatus;
char touchedStatus[16] = "";

const int BUFFER_SIZE = 300;

// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int LUX_readings[numReadings];      // the readings from the analog input
int LUX_readIndex = 0;              // the index of the current reading
int LUX_total = 0;                  // the running total
int LUX_average = 0;                // the average

void reconnect() {
  //Reconnect to Wifi and to MQTT. If Wifi is already connected, then autoconnect doesn't do anything.
  Serial.print("Attempting MQTT connection...");
  if (client.connect(host, mqtt_user, mqtt_pass)) {
    Serial.println("connected");
    client.subscribe(sensor_command_topic);
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  //Do something on command topic if
  /*payload[length] = '\0';
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
  */
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    /*** Only a reference to the advertised device is passed now
      void onResult(BLEAdvertisedDevice advertisedDevice) { **/
    void onResult(BLEAdvertisedDevice *advertisedDevice)
    {}
    
};

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

/*
 * Login page
 */
const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='dadmin' && form.pwd.value=='Updad8')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";


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
  doc["pressure"] = (String)presValue;
  doc["touch"] = (String)touchedStatus;
  doc["door"] = (String)openStatus;
  doc["ble"] = (String)bleValue;
  
  char buffer[BUFFER_SIZE];
  serializeJson(doc, buffer);
 
  //Serial.println(buffer);
  client.publish(sensor_state_topic, buffer, true);

}

/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}


void setup() {
  Wire.begin(SDA_PIN,SCL_PIN);
  pinMode(MOT_PIN,INPUT);
  pinMode(LIT_PIN,INPUT);
  pinMode(TUCH_PIN,INPUT);
  pinMode(DOOR_PIN,INPUT);

  bme.begin(0x76); 
  
  Serial.begin(115200);
  
  WiFi.onEvent(WiFiEvent);
  ETH.begin();

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99); // less or equal setInterval value

  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
          }
  });
  server.begin();

  client.setServer(mqtt_server, 1883); //1883 is the port number you have forwared for mqtt messages. You will need to change this if you've used a different port 
  client.setCallback(callback); //callback is the function that gets called for a topic sub  

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // initialize all the readings to 0:
  for (int LUX_thisReading = 0; LUX_thisReading < numReadings; LUX_thisReading++) {
    LUX_readings[LUX_thisReading] = 0;
  }


}
 
void loop() {
  server.handleClient();
  delay(1);
  
  //If MQTT client can't connect to broker, then reconnect
  if (!client.connected()) {
    reconnect();
  }

  client.loop(); //the mqtt function that processes MQTT messages


  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  //Serial.print("Devices found: ");
  //Serial.println(foundDevices.getCount());
  for (uint32_t i = 0; i < foundDevices.getCount(); i++)
  {
    NimBLEAdvertisedDevice device = foundDevices.getDevice(i);
    pServerAddress = new BLEAddress(device.getAddress());

      if (device.haveManufacturerData() == true)
          {
            std::string strManufacturerData = device.getManufacturerData();

            uint8_t cManufacturerData[100];
            strManufacturerData.copy((char *)cManufacturerData, strManufacturerData.length(), 0);

            if (strManufacturerData.length() == 25 && cManufacturerData[0] == 0x4C && cManufacturerData[1] == 0x00)
            {
              //Serial.println("Found an iBeacon!");
              BLEBeacon oBeacon = BLEBeacon();
              oBeacon.setData(strManufacturerData);
              if (strcmp(oBeacon.getProximityUUID().toString().c_str(), knownAddresses[1].c_str()) == 0)
              {
                //Serial.println(oBeacon.getProximityUUID().toString().c_str());
                Serial.println(device.getRSSI());
                bleValue = device.getRSSI();
                sendState();
                //Serial.println(pServerAddress->toString().c_str());
              }
              
            }
          }
  }

  //Serial.println("Scan done!");
  pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory
  
  
  float newTempValue = (bme.readTemperature() * 9/5) + 32 - tempAdjust; //to use celsius remove the true text inside the parentheses  
  float newHumValue = bme.readHumidity();  //dht.readHumidity();
  float newPresValue = bme.readPressure();
  pirValue = digitalRead(MOT_PIN); //read state of the motion sensor
  doorValue = digitalRead(DOOR_PIN);
  
  // subtract the last reading:
  total = total - readings[readIndex];
  LUX_total = LUX_total - LUX_readings[LUX_readIndex];
  // read from the sensor:
  readings[readIndex] = touchRead(TUCH_PIN);
  LUX_readings[LUX_readIndex] = analogRead(LIT_PIN);
  // add the reading to the total:
  total = total + readings[readIndex];
  LUX_total = LUX_total + LUX_readings[LUX_readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  LUX_readIndex = LUX_readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  if (LUX_readIndex >= numReadings) {
    // ...wrap around to the beginning:
    LUX_readIndex = 0;
  }
  // calculate the average:
  tuchValue = total / numReadings;
  ldrValue = LUX_total / numReadings;

  if (pirValue == LOW && pirStatus != 1) {
      strcpy(motionStatus,"standby");
      sendState();
      pirStatus = 1;
    }

    else if (pirValue == HIGH && pirStatus != 2) {
      strcpy(motionStatus,"motion detected");
      sendState();
      pirStatus = 2;
    } 

    if (doorValue == LOW && doorStatus != 1) {
      strcpy(openStatus,"closed");
      sendState();
      doorStatus = 1;
    }

    else if (doorValue == HIGH && doorStatus != 2) {
      strcpy(openStatus,"open");
      sendState();
      doorStatus = 2;
    } 

    if (tuchValue < 15 && tuchStatus != 1) {
      strcpy(touchedStatus,"touch detected");
      sendState();
      tuchStatus = 1;
    }

    else if (tuchValue >= 15 && tuchStatus != 2) {
      strcpy(touchedStatus,"standby");
      sendState();
      tuchStatus = 2;
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

    if (checkBoundSensor(ldrValue, LDR, diffLDR)) {
      LDR = ldrValue;
      sendState();
    }
  
}
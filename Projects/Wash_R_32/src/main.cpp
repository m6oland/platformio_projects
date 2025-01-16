#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_PHY_POWER 12

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
#include <pcf8574.h>
#include <pcf8574_esp.h>
#include <ACS712.h>
#include <Nextion.h>

/**************************** PIN DEFINITIONS UEXT*******************************************/
/////////////////////+3.3 V UEXT1 PIN 1                                             BLACK
/////////////////////GND UEXT1 PIN 2                                                WHITE
//#define AMP_PIN 39 //UEXT1 PIN 3 Capacitive Touch Pin HARD WIRE EXT_1 PIN 9 GPIO4   GREY  
#define INT_PIN 36 //UEXT1 PIN 4 ROTARY POT SWITCH                                  PURPLE
#define SCL_PIN 16 //UEXT1 PIN 5 Pin for I2C Clock HARD WIRE EXT_2 PIN 7 GPIO16     BLUE
#define SDA_PIN 13 //UEXT1 PIN 6 Pin for I2C Data HARD WIRE EXT_2 PIN 10 GPIO13     GREEN
#define DAT_PIN 15 //UEXT1 PIN 7 GPIO15 ROTARTY POT CLK                             YELHIGH
#define CLK_PIN 2  //UEXT1 PIN 8 GPIO2 ROTARY POT DATA                              ORANGE
//#define TAC_PIN 5 //UEXT1 PIN 9 GPIO14 IR SENSOR FOR TACHOMETER                    RED
#define AMP_PIN 39  //UEXT1 PIN 10 GPIO10 MICROSWITCH FOR LID SENSOR                 BROWN

//POT SWITCH
//TACHOMETER
//LID PIN
//

NexButton b1 = NexButton(0,2,"b1");

NexTouch *nex_listen_list[] = 
{
    &b1,
    NULL
};

// Set i2c address
PCF8574 RB1(0x20);
//PCF8574 RB2(0x24);

PCF857x RB2(0x24, &Wire);
volatile bool PCFInterruptFlag = false;
ACS712 sensor(ACS712_30A, AMP_PIN);

//Define MQTT 
#define mqtt_server "192.168.1.150"  //This is what you set up in HA. 
#define sensor_state_topic "sensor/washr"
#define sensor_command_topic "sensor/washr/#"
const char* mqtt_user = "mosquito"; //This is what you set up in HA. 
const char* mqtt_pass = "D1337o"; //This is what you set up in HA. 
////Define parameters for the http firmware update // which we aren't using 
const char* host = "washr";

static bool eth_connected = false;

const char* cycleName = "OFF";

WiFiClient espClient;
PubSubClient client(espClient);
WebServer server(80);

int cyc = -1;

unsigned long dialInterval = 120000;
unsigned long cycleStart = 0;
unsigned long cycleElapsed = 0;
int activeCycle = 0;
int startCycle = 0;
bool CyclePaused = false;

double Amps = 0;

//TACH Vars
float rev = 0;
int rpms=0;
unsigned long oldtimeRPM=0;
unsigned long timeRPM;
unsigned long lastRPMRead=0;
unsigned long RPMInterval = 2000;
int RPMRange = 50;
int currentRPMS=0;
int TACHStateMoving = 1;
int lastTACHStateStatus = 1;
unsigned long TACHmillisPrevious;
byte TACHdebounceInterval = 10;

//BUTTON Vars PB
int PBStateMoving = 1;
int lastPBStateStatus = 1;
unsigned long PBmillisPrevious;
byte PBdebounceInterval = 20;

//LID SWITCH Vars LID
int LIDStateMoving = 1;
int lastLIDStateStatus = 1;
unsigned long LIDmillisPrevious;
byte LIDdebounceInterval = 20;


//AMP Vars
float currentAMPS=0;
float AMPS=0;
float AMPRange = 0.5;

//POT Vars
static uint8_t prevNextCode = 0;
static uint16_t store=0;
static int8_t c,val;
int deg;
int potButton = 0;
int potStatus = 0;

bool turned = false;

int buttonStart = 0;
int buttonEnd = 0;
int lastButtonState = 0;
int currentButtonState = 0;
bool buttonOn = false;
bool lastButtonOn =false;
bool buttonReset = false;
bool buttonHomeDial = false;
bool buttonOnTriggered = false;
bool buttonResetTriggered = false;
bool buttonHomeDialTriggered = false;

char openStatus[16] = "";

const int BUFFER_SIZE = 300;

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
      digitalWrite(RELAY_PIN, LOW);
      delay(600);
      digitalWrite(RELAY_PIN, HIGH);
    }
  }
  */
}

void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname(host);
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

/********************************** START SEND STATE*****************************************/
void sendState() {
  StaticJsonDocument<BUFFER_SIZE> doc;

  //JsonObject root = doc.as<JsonObject>();

  doc["cycle"] = cycleName;
  /*doc["motion"] = (String)motionStatus;
  doc["ldr"] = (String)LDR;
  doc["temperature"] = (String)tempValue;
  doc["heatIndex"] = (String)calculateHeatIndex(humValue, tempValue);
  doc["pressure"] = (String)presValue;
  */
  doc["amps"] = (String)currentAMPS;
  doc["rpm"] = (String)currentRPMS;
  doc["door"] = (String)openStatus;
  
  char buffer[BUFFER_SIZE];
  serializeJson(doc, buffer);
 
  Serial.println(buffer);
  client.publish(sensor_state_topic, buffer, true);

}


void setSwitch(byte switchNumber, byte switchStatus) {
  RB1.write(switchNumber-1,switchStatus);
}

// A vald CW or  CCW move returns 1, invalid returns 0.
int8_t read_rotary() {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode <<= 2;
  if (digitalRead(DAT_PIN)) prevNextCode |= 0x02;
  if (digitalRead(CLK_PIN)) prevNextCode |= 0x01;
  prevNextCode &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevNextCode] ) {
      store <<= 4;
      store |= prevNextCode;
      //if (store==0xd42b) return 1;
      //if (store==0xe817) return -1;
      if ((store&0xff)==0x2b) return -1;
      if ((store&0xff)==0x17) return 1;
   }
   return 0;
}
//Interrupt Routine for pot spin
void isr() {
  if((val=read_rotary())) {
      if (c == 19 && val == 1) {
        c = 0;
      }
      else if (c == 0 && val == -1) {
        c = 19;
      }
      else {c +=val;}
      deg = c*18;
      turned = true;
  }
   
}

//Interrupt Routine for tach measurement
void rpm() {
  rev++;
}

void BUTTON_CHECK () {
  
  if(currentButtonState == LOW) {  
    //button is being held down

    int difference = millis() - buttonStart;

    if (difference >= 100 && difference < 3000 && buttonOnTriggered == false) { 
      //pressed for 1/10 second
      //buttonOn = !buttonOn;
      buttonOnTriggered = true;
      Serial.println("Trigger Toggle");
    }

    if (difference >= 3000 && difference < 5000 && buttonResetTriggered == false) { 
      //pressed for 3 seconds
      //buttonReset = true;
      buttonOnTriggered = false;
      buttonResetTriggered = true;
      Serial.println("Trigger Reset");
    }

    if (difference >= 5000 && buttonHomeDialTriggered == false) { 
      //pressed for 5 seconds
      //buttonHomeDial= true;
      buttonOnTriggered = false;
      buttonResetTriggered = false;
      buttonHomeDialTriggered = true;
      Serial.println("Trigger HOME");
    }
  buttonEnd = difference;
  }

}

void ISR_BUTTON () {
//currentButtonState = digitalRead(POT_PIN);
BUTTON_CHECK();
} 

void PCF2Interrupt() {
  PCFInterruptFlag = true;
}

void functionSelect (int cycleNumber) {
    
  if (cycleNumber == 20) {
    //FILL AGITATE LOW//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(2,LOW);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    delay(500);
    setSwitch(3,HIGH);   //COLD WATER RINSE LOW = ON HIGH = OFF
    setSwitch(6,HIGH);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,LOW);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    delay(500);
    setSwitch(5,HIGH);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    //Validate
    
    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 20;
    cycleName = "TEMP FILL AGITATE";
    CyclePaused = false;

  }
  
  else if (cycleNumber == 4) {
    //DRY AGITATE LOW//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(2,HIGH);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    setSwitch(3,HIGH);   //COLD WATER RINSE LOW = ON HIGH = OFF
    setSwitch(6,HIGH);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,LOW);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    delay(500);
    setSwitch(5,HIGH);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    
    //Validate
    
    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 4;
    cycleName = "DRY FILL AGITATE";
    CyclePaused = false;
  }

  else if (cycleNumber == 3) {
    //DRAIN - SPIN LOW//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(2,HIGH);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    setSwitch(3,HIGH);   //COLD WATER RINSE LOW = ON HIGH = OFF
    setSwitch(6,LOW);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,HIGH);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    setSwitch(5,LOW);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    delay(500);
    
    //Validate

    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 3;
    cycleName = "DRAIN SPIN";
    CyclePaused = false;
  }

  else if (cycleNumber == 12) {
    //COLD FILL AGITATE LOW//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(2,HIGH);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    setSwitch(3,LOW);   //COLD WATER RINSE LOW = ON HIGH = OFF
    delay(500);
    setSwitch(6,HIGH);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,LOW);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    delay(500);
    setSwitch(5,HIGH);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    
    //Validate

    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 12;
    cycleName = "COLD FILL AGITATE";
    CyclePaused = false;
  }

  else if (cycleNumber == 11) {
    //COLD SPRAY SPIN LOW//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(2,HIGH);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    setSwitch(3,LOW);   //COLD WATER RINSE LOW = ON HIGH = OFF
    delay(500);
    setSwitch(6,LOW);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,HIGH);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    setSwitch(5,LOW);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    delay(500);
    
    //Validate
    
    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 11;
    cycleName = "COLD SPRAY SPIN";
    CyclePaused = false;
  }

  else if (cycleNumber == 19) {
    //TEMP SPRAY SPIN LOW//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(7,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(2,LOW);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    delay(500);
    setSwitch(3,HIGH);   //COLD WATER RINSE LOW = ON HIGH = OFF
    setSwitch(6,LOW);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,HIGH);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    setSwitch(5,LOW);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    delay(500);
    
    //Validate
    
    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 19;
    cycleName = "TEMP SPRAY SPIN";
    CyclePaused = false;
  }
  
  else if (cycleNumber == 10) {
    //COLD FILLING AGITATE//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(2,HIGH);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    setSwitch(3,LOW);   //COLD WATER RINSE LOW = ON HIGH = OFF
    delay(500);
    setSwitch(6,HIGH);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,HIGH);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    setSwitch(5,LOW);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    delay(500);
    
    //Validate
    
    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 10;
    cycleName = "COLD FILLING AGITATE";
    CyclePaused = false;
  }

  else if (cycleNumber == 18) {
    //TEMP FILLING AGITATE//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(2,LOW);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    delay(500);
    setSwitch(3,HIGH);   //COLD WATER RINSE LOW = ON HIGH = OFF
    setSwitch(6,HIGH);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,HIGH);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    setSwitch(5,LOW);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    delay(500);
    
    //Validate
    
    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 18;
    cycleName = "TEMP FILLING AGITATE";
    CyclePaused = false;
  }

  else if (cycleNumber == 8) {
    //COLD FILL//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(2,HIGH);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    setSwitch(3,LOW);   //COLD WATER RINSE LOW = ON HIGH = OFF
    delay(500);
    setSwitch(6,HIGH);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,HIGH);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    setSwitch(5,HIGH);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    delay(500);
    
    //Validate
    
    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 8;
    cycleName = "COLD FILL";
    CyclePaused = false;
  }

  else if (cycleNumber == 16) {
    //TEMP FILL//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(2,LOW);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    delay(500);
    setSwitch(3,HIGH);   //COLD WATER RINSE LOW = ON HIGH = OFF
    setSwitch(6,HIGH);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,HIGH);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    setSwitch(5,HIGH);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    delay(500);
    
    //Validate
    
    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 16;
    cycleName = "TEMP FILL";
    CyclePaused = false;
  }

  else if (cycleNumber == 1) {
    //NEUTRAL SHIFT//////////////////////////////////////////////////////////////////////
    setSwitch(7,LOW);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(1,LOW);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    delay(500);
    setSwitch(2,HIGH);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    setSwitch(3,HIGH);   //COLD WATER RINSE LOW = ON HIGH = OFF
    delay(500);
    setSwitch(6,LOW);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(4,HIGH);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    delay(500);
    setSwitch(5,HIGH);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    
    //Validate

    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 1;
    cycleName = "DRAIN SPIN SHIFT";
    CyclePaused = false;
  }


  else if (cycleNumber == 0) {
    //ALL OFF//////////////////////////////////////////////////////////////////////
    setSwitch(4,HIGH);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    setSwitch(5,HIGH);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(6,HIGH);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(2,HIGH);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    setSwitch(3,HIGH);   //COLD WATER RINSE LOW = ON HIGH = OFF
    delay(500);
    setSwitch(1,HIGH);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    setSwitch(7,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    delay(500);
    //Validate
    
    /////////////////////////////////////////////////////////////////////////////////////
    activeCycle = 0;
    cycleName = "OFF";
    CyclePaused = false;
  }

  else if (cycleNumber == -1) {
    //ALL OFF//////////////////////////////////////////////////////////////////////
    setSwitch(4,HIGH);  //AGITATE DEPENDS ON LEVEL SWITCH FULL LOW = ON HIGH = OFF
    setSwitch(5,HIGH);   //MOTOR BYPASS LEVEL SENSOR LOW = ON HIGH = OFF
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(6,HIGH);   //MOTOR FUNCTION SELECT HIGH = AGITATE LOW = SPIN
    delay(500);         //WAIT FOR CONTACTS TO MOVE
    setSwitch(2,HIGH);  //WATER TEMP SELECT LOW = ON HIGH = OFF
    setSwitch(3,HIGH);   //COLD WATER RINSE LOW = ON HIGH = OFF
    delay(500);
    setSwitch(1,HIGH);  //MASTER TIMER SWITCH LOW = ON HIGH = OFF
    setSwitch(7,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    setSwitch(8,HIGH);   //MASTER TIMER SWITCH SAFETY HIGH = ENABLE LOW = DISABLE
    delay(500);
    //Validate
    
    /////////////////////////////////////////////////////////////////////////////////////
    Serial.println("Physical Pause");
    cycleName = "PAUSE";
    CyclePaused = true;
  }


}

void timerSelect () {
  if (activeCycle != 0) {
    if (buttonOn == true) {
      functionSelect(activeCycle); //Master Timer
    }
    else if (buttonOn == false) {
      if (CyclePaused == false) {
        functionSelect(-1);
      }
    }
  Serial.println(cycleName);
  sendState(); 
  }
}

int mapCycle(int degree) {
  if(degree >= 0 && degree < 18) {
    return 0; //Super Wash
  }
  else if(degree >= 18 && degree < 144) {
    return 1; //Normal
  }
  else if(degree >= 144 && degree < 270) {
    return 2; //Permanent Press
  }
  else if(degree >= 270 ) {
    return 3; //Gentle Wash
  }
  else {
    return -1;
  }
}



void setup()
{
  Wire.begin(SDA_PIN,SCL_PIN);
  sensor.setZeroPoint(sensor.calibrate());
  Serial.begin(115200);
  RB1.begin();
  RB2.begin();
  
	for (int i = 0; i < 8; i++) {
    RB1.write(i,HIGH);
    RB2.write(i,HIGH);
  }

  

  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DAT_PIN, INPUT_PULLUP);
  pinMode(INT_PIN, INPUT_PULLUP);
  RB2.resetInterruptPin();

  currentButtonState = HIGH;
  lastButtonState = currentButtonState;

  attachInterrupt(DAT_PIN,isr,CHANGE);
  attachInterrupt(CLK_PIN,isr,CHANGE);
  attachInterrupt(INT_PIN,PCF2Interrupt,FALLING);


   
  
  WiFi.onEvent(WiFiEvent);
  ETH.begin();

  //use mdns for host name resolution
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  //return index page which is stored in serverIndex
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  //handling uploading firmware file 
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
      //flashing firmware to ESP
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
  

 //initialize Lid Status
 lastLIDStateStatus = RB2.read(1);
Serial.print("degrees.val=");
Serial.print(0);
Serial.write(0xff);
Serial.write(0xff);
Serial.write(0xff);

Serial.print("status.txt=");
Serial.print("\"");
Serial.print("Home");
Serial.print("\"");
Serial.write(0xff);
Serial.write(0xff);
Serial.write(0xff);

}


void loop()
{

  
  server.handleClient();
  delay(1);
  
  //If MQTT client can't connect to broker, then reconnect
  if (!client.connected()) {
    reconnect();
  }

  client.loop(); //the mqtt function that processes MQTT messages
  
  BUTTON_CHECK();
  
  //PCF Interrupted
  if (PCFInterruptFlag==true) {
      //TACH BUTTON INTERRUPT  PCF PIN 3
      TACHStateMoving = RB2.read(2);
      if(TACHmillisPrevious==0) {
        if ((TACHStateMoving == HIGH) && (lastTACHStateStatus == LOW)) {
          //PIN IS RISING ENTERING REFLECTOR

        }
        else if ((TACHStateMoving == LOW) && (lastTACHStateStatus == HIGH)) {
          //PIN IS FALLING LEAVING REFLECTOR
          rpm();
        }
        TACHmillisPrevious = millis();
        lastTACHStateStatus = TACHStateMoving;

      }
      

      if (millis() - TACHdebounceInterval >= TACHmillisPrevious) {
        //debounce interval expired
        TACHmillisPrevious = 0;
      }

      //PUSH BUTTON INTERRUPT  PCF PIN 2
      PBStateMoving = RB2.read(0);
      if(PBmillisPrevious==0) {
        if ((PBStateMoving == HIGH) && (lastPBStateStatus == LOW)) {
          //PIN IS RISING BUTTON WAS RELEASED
          Serial.print("Button held for ");
          Serial.print(buttonEnd/1000);
          Serial.println(" seconds");
          if (buttonOnTriggered == true) {
            buttonOn = !buttonOn;
          }
          else if (buttonResetTriggered == true) {
            buttonReset = true;
          }
          else if (buttonHomeDialTriggered == true) {
            buttonHomeDial= true;
          }

        }
        else if ((PBStateMoving == LOW) && (lastPBStateStatus == HIGH)) {
          //PIN IS FALLING BUTTON WAS PRESSED
          buttonStart = millis();
          buttonOnTriggered = false;
          buttonResetTriggered = false;
          buttonHomeDialTriggered = false;
          buttonReset = false;
          buttonHomeDial = false;
          Serial.println("Trigger Start");
          
        }
        PBmillisPrevious = millis();
        lastPBStateStatus = PBStateMoving;
        currentButtonState = lastPBStateStatus;

      }
      
      if (millis() - PBdebounceInterval >= PBmillisPrevious) {
        //debounce interval expired
        PBmillisPrevious = 0;
      }

      //LID SENSOR INTERRUPT  PCF PIN 1
      LIDStateMoving = RB2.read(1);
      if(LIDmillisPrevious==0) {
        if ((LIDStateMoving == HIGH) && (lastLIDStateStatus == LOW)) {
          //PIN IS RISING LID WAS OPENED
          strcpy(openStatus,"open");
          sendState();

        }
        else if ((LIDStateMoving == LOW) && (lastLIDStateStatus == HIGH)) {
          //PIN IS FALLING LID WAS CLOSED
          strcpy(openStatus,"closed");
          sendState();
        }
        LIDmillisPrevious = millis();
        lastLIDStateStatus = LIDStateMoving;       

      }
      
      if (millis() - LIDdebounceInterval >= LIDmillisPrevious) {
        //debounce interval expired
        LIDmillisPrevious = 0;
      }


    }

  AMPS = sensor.getCurrentAC();
 
  if ((millis() - lastRPMRead) > RPMInterval) {
    timeRPM = millis() - oldtimeRPM;
    rpms = (rev/timeRPM)*60000;
    oldtimeRPM=millis();
    rev = 0;
    lastRPMRead = millis();
  }
  

  if ((rpms <= (currentRPMS - RPMRange)) | (rpms >= (currentRPMS + RPMRange))) {
    currentRPMS = rpms;
    sendState();
  }
  
  
  if ((AMPS <=(currentAMPS - AMPRange)) | (AMPS >= (currentAMPS + AMPRange))) {
    currentAMPS = AMPS;
    sendState();
  }

  if (buttonOn != lastButtonOn) {
    timerSelect();
    if (buttonOn == true and lastButtonOn == false) {
      cycleStart = millis();
    }

    lastButtonOn = buttonOn;
  }


  if (buttonOn == true) {
    cycleElapsed = millis() - cycleStart;

    if ( cycleElapsed < 720000) { //12 Minutes 720 Seconds 720000
      startCycle = 20; //FILL AGITATE LOW
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();
      }
    }
    else if (cycleElapsed < 723000) { //725000
      //Pause before next cycleElapsed
      if (CyclePaused == false) {
        functionSelect(-1);
        Serial.println(cycleName);
        sendState();
      }

    }    
    else if (cycleElapsed < 800000) { //960000
      startCycle = 3; //NEUTRAL ENGAGED DRAIN SPIN
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();  
      }
    }

    else if (cycleElapsed < 805000) { //725000
      //Pause to kick neutral cams
      startCycle = 1;  // DRAIN SPIN SHIFT
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();  
      }    
    }    
    else if (cycleElapsed < 960000) { //960000
      startCycle = 3; //DRAIN SPIN LOW
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();  
      }
    }

    else if (cycleElapsed < 963000) { //965000
      //Pause before next cycleElapsed
      if (CyclePaused == false) {
        functionSelect(-1);
        Serial.println(cycleName);
        sendState();
      } 
    }

    else if (cycleElapsed < 1440000) { //1440000
      startCycle = 20; //FILL AGITATE HI
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();
      }
    }

    else if (cycleElapsed < 1443000) { //1445000
      //Pause before next cycleElapsed
      if (CyclePaused == false) {
        functionSelect(-1);
        Serial.println(cycleName);
        sendState();
      }
    }

    else if (cycleElapsed < 1520000) { //960000
      startCycle = 3; //NEUTRAL ENGAGED DRAIN SPIN
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();  
      }
    }

    else if (cycleElapsed < 1525000) { //725000
    //Pause to kick neutral cams
      startCycle = 1;  // DRAIN SPIN SHIFT
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();  
      }  
    }

    else if (cycleElapsed < 1560000) { //1560000
      startCycle = 3; //DRAIN SPIN LOW
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState(); 
      }
    }

    else if (cycleElapsed < 1680000) { //1680000
      startCycle = 11;  //SPRAY SPIN LOW
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();
      }
    }

    else if (cycleElapsed < 2280000) { //2280000
      startCycle = 3; //DRAIN SPIN LOW
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();
      }
    }

    else {
      startCycle = 0; //OFF
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();
        buttonOn = false;
      }
    }

  }

  if (buttonReset) {
    if (buttonReset == true) {
      Serial.println("Cycle Reset");
      startCycle = 0; //OFF
      if (activeCycle != startCycle) {
        functionSelect(startCycle);
        Serial.println(cycleName);
        sendState();
        }
      buttonOn = false;
      buttonReset = false;
    }
  }


  if (buttonHomeDial ) {
    if (buttonHomeDial == true) {
      Serial.println("Dial Homed");
      c = 0;
      buttonHomeDial = false;
    }
  }

   if( turned ) {
      Serial.println(deg);
      if ( prevNextCode==0x0b) {
         //Moving counterClockwise
 
      }

      if ( prevNextCode==0x07) {
         //Moving clockwise
 
   }
   turned = false;
  }


}
  

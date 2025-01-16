#include <Arduino.h>
#include <Wire.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

/**************************** PIN DEFINITIONS ********************************************/
const int redPin = 12;
const int greenPin = 14;
const int bluePin = 13;
static const int servoPin = 4;
const int BUFFER_SIZE = 800;

Servo servo1;

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire,-1);

/******************************** GLOBALS for fade/flash *******************************/
byte red = 255;
byte green = 255;
byte blue = 255;
byte brightness = 255;

byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;

bool stateOn = true;

bool startFade = false;
unsigned long lastLoop = 0;
int transitionTime = 0;
bool inFade = false;
int loopCount = 0;
int stepR, stepG, stepB;
int redVal, grnVal, bluVal;

bool flash = false;
bool startFlash = false;
int flashLength = 0;
unsigned long flashStartTime = 0;
byte flashRed = red;
byte flashGreen = green;
byte flashBlue = blue;
byte flashBrightness = brightness;

int servoAngle = 90;
int servoCurrent = 90;

int screenTimeout = 15000;
int servoTimeout = 15000;

int servoStartTime = 0;
int screenStartTime = 0;

bool screenInfo = false;
bool servoActive = false;

const int stringSize = 256;

char displayHead[stringSize] = "Tracy-801";
char displayLine1[stringSize] = "";
char displayLine2[stringSize] = "";
char displayLine3[stringSize] = "";

char currentHead[stringSize] = "";
char currentLine1[stringSize] = "";
char currentLine2[stringSize] = "";
char currentLine3[stringSize] = "";

/************* Input Variables  **************************/
//const byte numChars = BUFFER_SIZE;
char receivedChars[BUFFER_SIZE];

boolean newData = false;
/************* MQTT TOPICS (change these topics as you wish)  **************************/
#define light_state_topic "bruh/sensornode0"
#define light_set_topic "bruh/sensornode0/set"

const char* on_cmd = "ON";
const char* off_cmd = "OFF";


/************* Accept Serial Commands with <> markers  **************************/

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static int ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
 // if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= BUFFER_SIZE) {
                    ndx = BUFFER_SIZE - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

/********************************** START PROCESS JSON*****************************************/
bool processJson(char* message) {
  StaticJsonDocument<BUFFER_SIZE> doc;
  
  DeserializationError error = deserializeJson(doc, message);
  Serial.println("JSON doc");

  if (error) {
    Serial.println("deserializeJson() failed");
    return false;
  }

  if (doc.containsKey("state")) {
    Serial.println("JSON State");
    if (strcmp(doc["state"], on_cmd) == 0) {
      stateOn = true;
    }
    else if (strcmp(doc["state"], off_cmd) == 0) {
      stateOn = false;
    }
  }

  if (doc.containsKey("servo")) {
    servoAngle = (int)doc["servo"]["angle"];
    Serial.println(servoAngle);
  }

  if(doc.containsKey("display")){
    Serial.println("In Display Loop");
    const char* hd = doc["display"]["header"]["text"];
    const char* l1 = doc["display"]["line_1"]["text"];
    const char* l2 = doc["display"]["line_2"]["text"];
    const char* l3 = doc["display"]["line_3"]["text"];

    if (hd) {
      if (strcmp(currentHead,doc["display"]["header"]["text"]) != 0) {
        strcpy(displayHead, doc["display"]["header"]["text"]);
      }
    }
    if (l1) {
      if (strcmp(currentLine1,doc["display"]["line_1"]["text"]) != 0) {
          strcpy(displayLine1, doc["display"]["line_1"]["text"]);
      }
    }
    if (l2) {
      if (strcmp(currentLine2,doc["display"]["line_2"]["text"]) != 0) {
        strcpy(displayLine2, doc["display"]["line_2"]["text"]);
      }
    }
    if (l3) {
      if (strcmp(currentLine3,doc["display"]["line_3"]["text"]) != 0) {
        strcpy(displayLine3, doc["display"]["line_3"]["text"]);
      }
    }

  }

  // If "flash" is included, treat RGB and brightness differently
  if (doc.containsKey("flash")) {
    Serial.println("JSON Flash");
    flashLength = (int)doc["flash"] * 1000;

    if (doc.containsKey("brightness")) {
      Serial.println("JSON Brightness");
      flashBrightness = doc["brightness"];
    }
    else {
      flashBrightness = brightness;
    }

    if (doc.containsKey("color")) {
      Serial.println("JSON Color");
      flashRed = doc["color"]["r"];
      flashGreen = doc["color"]["g"];
      flashBlue = doc["color"]["b"];
    }
    else {
      flashRed = red;
      flashGreen = green;
      flashBlue = blue;
    }

    flashRed = map(flashRed, 0, 255, 0, flashBrightness);
    flashGreen = map(flashGreen, 0, 255, 0, flashBrightness);
    flashBlue = map(flashBlue, 0, 255, 0, flashBrightness);

    flash = true;
    startFlash = true;
  }
  else { // Not flashing
    flash = false;

    if (doc.containsKey("color")) {
      Serial.println("JSON Color");
      red = doc["color"]["r"];
      green = doc["color"]["g"];
      blue = doc["color"]["b"];
    }

    if (doc.containsKey("brightness")) {
      Serial.println("JSON Brightness");
      brightness = doc["brightness"];
    }

    if (doc.containsKey("transition")) {
      Serial.println("JSON transition");
      transitionTime = doc["transition"];
    }
    else {
      transitionTime = 0;
    }
  }

  return true;
}

/************* Print Received Characters  **************************/
void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);

        Serial.println(receivedChars);

        if (!processJson(receivedChars)) {
          Serial.println("JSON FAILED");
          return;
        }

        if (stateOn) {
          // Update lights
          realRed = map(red, 0, 255, 0, brightness);
          realGreen = map(green, 0, 255, 0, brightness);
          realBlue = map(blue, 0, 255, 0, brightness);
        }
        else {
          realRed = 0;
          realGreen = 0;
          realBlue = 0;
        }

        startFade = true;
        inFade = false; // Kill the current fade
        newData = false;
    }
}

/********************************** START SET COLOR *****************************************/
void setColor(int inR, int inG, int inB) {
  ledcWrite(1,inR);
  ledcWrite(2,inG);
  ledcWrite(3,inB);
  Serial.println("Setting LEDs:");
  Serial.print("r: ");
  Serial.print(inR);
  Serial.print(", g: ");
  Serial.print(inG);
  Serial.print(", b: ");
  Serial.println(inB);
}

/**************************** START TRANSITION FADER *****************************************/
// From https://www.arduino.cc/en/Tutorial/ColorCrossfader
/* BELOW THIS LINE IS THE MATH -- YOU SHOULDN'T NEED TO CHANGE THIS FOR THE BASICS

  The program works like this:
  Imagine a crossfade that moves the red LED from 0-10,
    the green from 0-5, and the blue from 10 to 7, in
    ten steps.
    We'd want to count the 10 steps and increase or
    decrease color values in evenly stepped increments.
    Imagine a + indicates raising a value by 1, and a -
    equals lowering it. Our 10 step fade would look like:

    1 2 3 4 5 6 7 8 9 10
  R + + + + + + + + + +
  G   +   +   +   +   +
  B     -     -     -

  The red rises from 0 to 10 in ten steps, the green from
  0-5 in 5 steps, and the blue falls from 10 to 7 in three steps.

  In the real program, the color percentages are converted to
  0-255 values, and there are 1020 steps (255*4).

  To figure out how big a step there should be between one up- or
  down-tick of one of the LED values, we call calculateStep(),
  which calculates the absolute gap between the start and end values,
  and then divides that gap by 1020 to determine the size of the step
  between adjustments in the value.
*/
int calculateStep(int prevValue, int endValue) {
  int step = endValue - prevValue; // What's the overall gap?
  if (step) {                      // If its non-zero,
    step = 1020 / step;          //   divide by 1020
  }

  return step;
}

/* The next function is calculateVal. When the loop value, i,
   reaches the step size appropriate for one of the
   colors, it increases or decreases the value of that color by 1.
   (R, G, and B are each calculated separately.)
*/
int calculateVal(int step, int val, int i) {
  if ((step) && i % step == 0) { // If step is non-zero and its time to change a value,
    if (step > 0) {              //   increment the value if step is positive...
      val += 1;
    }
    else if (step < 0) {         //   ...or decrement it if step is negative
      val -= 1;
    }
  }

  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255) {
    val = 255;
  }
  else if (val < 0) {
    val = 0;
  }

  return val;
}


void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    
    servo1.attach(servoPin,4);
    servoActive = true;
    servo1.write(servoCurrent);
    servoStartTime = millis();

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

    // Clear the buffer.
    display.clearDisplay();

    ledcAttachPin(redPin, 1);
    ledcAttachPin(greenPin, 2);
    ledcAttachPin(bluePin, 3);

    // Initialize channels 
    // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
    // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
    ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
    ledcSetup(2, 12000, 8);
    ledcSetup(3, 12000, 8);

     for(uint8_t i=0; i < 4; i++) {
      // ledcWrite(channel, dutycycle)
      // For 8-bit resolution duty cycle is 0 - 255
      ledcWrite(i, 255);  // test high output of all leds in sequence
      delay(333);
      ledcWrite(i, 0);
     }
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.print("Tracy-801 is online");
  display.setTextSize(1);
  display.print("Haunting doll....");
  display.print("Channeling spook energy");
  display.print("Doing monster mash...");
  display.print("This is a bunch of text that I need to print to make sure that the first several lines of text are displaying on my screen, so here we go. wheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");

  display.setCursor(0,0);
  display.display(); // actually display all of the above
  screenStartTime = millis();

  //Dim Blue Eye
  realRed = map(0, 0, 255, 0, 5);
  realGreen = map(0, 0, 255, 0, 5);
  realBlue = map(100, 0, 255, 0, 5);
  setColor(realRed, realGreen, realBlue);
}

void loop() {
  // put your main code here, to run repeatedly:

   recvWithStartEndMarkers();
   showNewData();

  if (!inFade) {

  }

//Servo servoTimeout
if (millis() >= (servoStartTime + servoTimeout) && servoActive == true) {
  servoAngle = 90;
  servo1.write(servoAngle);
  servoCurrent = servoAngle;
  servoActive = false;
  //Dim Blue Eye
  realRed = map(0, 0, 255, 0, 5);
  realGreen = map(0, 0, 255, 0, 5);
  realBlue = map(100, 0, 255, 0, 5);
  setColor(realRed, realGreen, realBlue);
}

if (servoAngle != servoCurrent) {
  
  servo1.write(servoAngle);
  servoCurrent = servoAngle;
  servoStartTime = millis();
  servoActive = true;
}

if (strcmp(currentHead,displayHead) != 0 || strcmp(currentLine1,displayLine1) != 0 ||
strcmp(currentLine2,displayLine2) != 0 || strcmp(currentLine3,displayLine3) != 0) {
  display.clearDisplay();
  display.stopscroll();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.setTextWrap(false);
  display.println(displayHead);
  //display.print("\n");
  strcpy(currentHead,displayHead);
  display.setTextSize(1);
  display.println(displayLine1);
  //display.print("\n");
  strcpy(currentLine1,displayLine1);
  display.println(displayLine2);
  strcpy(currentLine2,displayLine2);
  display.setTextWrap(true);
  display.print(displayLine3);
  strcpy(currentLine3,displayLine3);
  display.setCursor(0,0);
  display.display(); // actually display all of the above
  screenStartTime = millis();
  screenInfo = true;
}

//check screen screenTimeout
if (millis() >= (screenStartTime + screenTimeout) && screenInfo == true) {
  Serial.println("Screen has timed out");
  Serial.println(currentHead);
  screenInfo = false;
  display.clearDisplay();
  display.startscrollleft(0x00, 0x07);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(currentHead);
  display.display(); // actually display all of the above
  display.setTextSize(1);
  strcpy(displayLine1,"");
  strcpy(currentLine1,displayLine1);
  strcpy(displayLine2,"");
  strcpy(currentLine2,displayLine2);
  strcpy(displayLine3,"");
  strcpy(currentLine3,displayLine3);
}

if (flash) {
    if (startFlash) {
      startFlash = false;
      flashStartTime = millis();
    }

    if ((millis() - flashStartTime) <= flashLength) {
      if ((millis() - flashStartTime) % 1000 <= 500) {
        setColor(flashRed, flashGreen, flashBlue);
      }
      else {
        setColor(0, 0, 0);
        // If you'd prefer the flashing to happen "on top of"
        // the current color, uncomment the next line.
        // setColor(realRed, realGreen, realBlue);
      }
    }
    else {
      flash = false;
      setColor(realRed, realGreen, realBlue);
    }
  }

  if (startFade) {
    // If we don't want to fade, skip it.
    if (transitionTime == 0) {
      setColor(realRed, realGreen, realBlue);

      redVal = realRed;
      grnVal = realGreen;
      bluVal = realBlue;

      startFade = false;
    }
    else {
      loopCount = 0;
      stepR = calculateStep(redVal, realRed);
      stepG = calculateStep(grnVal, realGreen);
      stepB = calculateStep(bluVal, realBlue);

      inFade = true;
    }
  }

  if (inFade) {
    startFade = false;
    unsigned long now = millis();
    if (now - lastLoop > transitionTime) {
      if (loopCount <= 1020) {
        lastLoop = now;

        redVal = calculateVal(stepR, redVal, loopCount);
        grnVal = calculateVal(stepG, grnVal, loopCount);
        bluVal = calculateVal(stepB, bluVal, loopCount);

        setColor(redVal, grnVal, bluVal); // Write current values to LED pins

        Serial.print("Loop count: ");
        Serial.println(loopCount);
        loopCount++;
      }
      else {
        inFade = false;
      }
    }
  }


}

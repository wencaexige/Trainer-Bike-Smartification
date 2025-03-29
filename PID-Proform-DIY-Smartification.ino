/*
   A BLE Proform Trainer conversion  for Zwift

   Program receives environment date from the Zwift game (via BLE) and adjusts the resistance of 
   the Proform inddor bike, a revamped version : the original mainboard was discarded.
   A H-Bridge is used to control the resistance DC motor (9V). 

   Reuses code from previous similar projects: 

   for the BLE code https://www.instructables.com/Connecting-Old-Proform-TDF-Bike-to-Zwift-Part-2/
   
   for general architecture of the code (voids, loops,...) https://github.com/kevinmott09/proformBLE

   for PID that makes the integrated DC motor into a servo https://dronebotworkshop.com/custom-servo-motor/
*/


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
//#include <ArduinoBLE.h>

#include <JC_Button.h>  // https://github.com/JChristensen/JC_Button
#include <PID_v1.h>

#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>


//pin setup
#define UP_BUTTON 6    //rear gears shift up button, rear cassette
#define DOWN_BUTTON 5  //rear gears shift down button
//#define LED 9     //led attached to this pin to indicate BLE connection
#define FEEDBACK_POT_PIN 1  //feedback from pot integrated in DC motor
#define PWM_HARDER 2        //pin to spin DC motor to harder resistance
#define PWM_EASIER 3        //pin to spin DC motor to easier resistance


//variable that setups display I2C address etc...
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_SDA_PIN 0     // pin connected to SDA pin of display
#define SCREEN_SCK_PIN 10    // pin connected to SDA pin of display


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//instantiate button instances for debounce library
Button UpBtn(UP_BUTTON);  //rearUpButton
Button DownBtn(DOWN_BUTTON);


//...............................................................................................................................

//variables for PID & DC motor shield
double Setpoint, Input, Output;
double Kp = 20, Ki = 5, Kd = 0.00;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//...............................................................................................................................






// Declare timestamp variables
unsigned long currentMillis;
long previousMillis = 0;
long interval = 20;

// Integer to represent input value from potentiometer
int input_val;

// Integer to represent PWM output value to motor
int pwm_val;

// Integer to represent current error value
int gap;

// Minimum Potentiometer Input Value (determine through experimentation)
int pot_min = 1600;

// Maximum Potentiometer Input Value (determine through experimentation)
int pot_max = 4000;

// Max speed of pwm to restrain, test feature, 255 too fast. Scale all variables of PID to it
int speed_max = 130;

//PID reachied the target resistance
boolean reached = false;

//set to true to print messages to the console via serial0 (usb)
boolean debugging = true;

boolean gearChanged = false;  //track gear changes


//............................BLE objects, characteristics and variables setup..............

BLEServer* pServer = NULL;
BLECharacteristic* pIndoorBike = NULL;
BLECharacteristic* pFeature = NULL;
BLECharacteristic* pControlPoint = NULL;
BLECharacteristic* pStatus = NULL;

BLEAdvertisementData advert;
BLEAdvertisementData scan_response;
BLEAdvertising* pAdvertising;

bool deviceConnected = false;
bool oldDeviceConnected = false;
int value = 0;  //This is the value sent as "nothing".  We need to send something for some of the charactistics or it won't work.

#define FTMSDEVICE_FTMS_UUID "00001826-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_INDOOR_BIKE_CHAR_UUID "00002AD2-0000-1000-8000-00805F9B34FB"
//#define FTMSDEVICE_RESISTANCE_RANGE_CHAR_UUID "00002AD6-0000-1000-8000-00805F9B34FB"
//#define FTMSDEVICE_POWER_RANGE_CHAR_UUID "00002AD8-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_FTMS_FEATURE_CHAR_UUID "00002ACC-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_FTMS_CONTROL_POINT_CHAR_UUID "00002AD9-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_FTMS_STATUS_CHAR_UUID "00002ADA-0000-1000-8000-00805F9B34FB"

//response/acknowledgement to send to the client after writing to control point
uint8_t replyDs[3] = { 0x80, 0x00, 0x01 };  // set up replyDS array with 3 byte values for control point resp to zwift


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

//..........................................................................


//variables for calculating resistance.  Zwift only uses grade but other programs (not tested) might(?) use others
float wind_speed = 0;     // meters per second, resolution 0.001
int_least16_t grade = 0;  // grade value recieved from zwift


float gradeFloat = 0;  //both are float, this is the one divided by 100
                       // I have two grade variables, one from each of the precedent
                       // zwift DIY projects I reused for mine.

float oldGrade = 0;
float crr = 0;  // Coefficient of rolling resistance, resolution 0.0001
float cw = 0;   // Wind resistance Kg/m, resolution 0.01;

//.......................................................................................

unsigned int currentResistance = 0;
unsigned int resistanceSetting = 0;
unsigned int oldResistance = 0;
unsigned int oldGear = 0;

int gear = 0;
int rearGear = 0;
int gearOffset = 0;


// Set the correct resistance level on the physical trainer
//15 degrees of incline in 0.5 degree steps.  30 levels.  Max grade in Zwift is 18%?  Most not above 14%.
//but, we could say that 15% is the max resistance setting.  Works out to about 8 gear steps per degree (128/15=8.5333)
//incline is given in 0.01degree increments, so the gear change should be rounded to nearest

void ShowResistance() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 15);
  display.setTextSize(2);
  display.print(gradeFloat, 0);
  display.println("%");
  display.setCursor(40, 30);
  display.setTextSize(4);
  display.print(rearGear);
  display.println("s");
  display.display();
}

void NextGear(byte shift) {  //get 1,2,4 or 8. shift rear down, shift r up, shift f down, shift f up
  int offset = 0;
  if (shift == 1) {
    offset = -4;
    rearGear--;
    if (rearGear < 0) {
      rearGear = 0;  //can't go lower than 1 or no power will be generated
      offset = 0;    //no shift happens
    }
  }
  if (shift == 2) {
    offset = 4;
    rearGear++;
    if (rearGear > 16) {
      rearGear = 16;  //thirteen speed freewheel, 0-12
      offset = 0;
    }
  }

  gearOffset = gearOffset + offset;
  gearChanged = true;
  SetResistance(resistanceSetting);  //now update the resistance setting
  Serial.println("new resistance set, gearchanged = true");
}

void SetResistance(int setting) {

  currentResistance = setting + gearOffset;
  if (currentResistance < 0) {
    currentResistance = 0;
  }
  if (currentResistance > 127) {
    currentResistance = 127;
  }
  Serial.print("Setting resistance to: ");
  Serial.println(currentResistance);
}

void ChangeResistance() {
  reached = false;
  while (reached == false) {

    // Reset value of currentMillis
    currentMillis = millis();

    // See if interval period has expired, if it has then reset value
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      // Get value from potentiometer
      //input_val = analogRead(FEEDBACK_POT_PIN);
      input_val = 0;
      for (int i = 0; i < 100; i++) {
        input_val = input_val + analogRead(FEEDBACK_POT_PIN);
      }
      input_val = input_val / 100;


      // Establish Setpoint value for PID
      Setpoint = map(currentResistance, 0, 127, -255, 255);

      // Establish Input value for PID
      Input = map(input_val, pot_min, pot_max, -255, 255);

      gap = Setpoint - Input;

      // Run PID process to get Output value
      myPID.Compute();

      Serial.print("Pot = ");
      Serial.print(input_val);
      Serial.print(" Setpoint = ");
      Serial.print(Setpoint);
      Serial.print(" Input = ");
      Serial.print(Input);
      Serial.print(" Output = ");
      Serial.print(Output);


      pwm_val = map(Output, -255, 255, -speed_max, speed_max);
      Serial.print(" PWM Out = ");
      Serial.print(pwm_val);


      if (gap > 3) {

        // Need to move motor forward
        // Write PWM to motor driver
        Serial.print(" harder");
        analogWrite(PWM_HARDER, abs(pwm_val));
        analogWrite(PWM_EASIER, 0);  //we need to set the other pin to 0 otherwise the motor driver enters brake mode

      } else if (gap < -3) {

        // Need to move motor in reverse
        // Invert Output value, as we cannot use a negative PWM value

        // Write PWM to motor driver
        Serial.print(" easier");
        analogWrite(PWM_EASIER, abs(pwm_val));
        analogWrite(PWM_HARDER, 0);  //we need to set the other pin to 0 otherwise the motor driver enters brake mode


      } else {
        //stop PWMs on approaching setpoint enough, otherwise squeaking motor noise continues
        analogWrite(PWM_EASIER, 255);
        analogWrite(PWM_HARDER, 255);
        Serial.println("target reached");
        reached = true;  //break the while loop for ChangeResistance function and continue
        Serial.println("Exiting ChangeResistance.......");
        gearChanged = false;
      }
      Serial.println();
    }
  }
}

void ReadButton() {

  UpBtn.read();
  DownBtn.read();

  if (UpBtn.wasReleased()) {
    NextGear(2);
    Serial.println("up button pressed");
  }
  if (DownBtn.wasReleased()) {
    NextGear(1);
    Serial.println("down button pressed");
  }
}

void BluetoothSetup() {

  //Setup BLE
  //Serial.println("Creating BLE server...");
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  // Serial.println("Define service...");
  BLEService* pService = pServer->createService(FTMSDEVICE_FTMS_UUID);

  // Create BLE Characteristics
  //Serial.println("Define characteristics");
  pIndoorBike = pService->createCharacteristic(FTMSDEVICE_INDOOR_BIKE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pIndoorBike->addDescriptor(new BLE2902());
  pControlPoint = pService->createCharacteristic(FTMSDEVICE_FTMS_CONTROL_POINT_CHAR_UUID, BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_WRITE);
  pControlPoint->addDescriptor(new BLE2902());
  pFeature = pService->createCharacteristic(FTMSDEVICE_FTMS_FEATURE_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
  pFeature->addDescriptor(new BLE2902());
  pStatus = pService->createCharacteristic(FTMSDEVICE_FTMS_STATUS_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pStatus->addDescriptor(new BLE2902());

  // Start the service
  //Serial.println("Staring BLE service...");
  pService->start();

  // Start advertising
  //Serial.println("Define the advertiser...");
  pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->setScanResponse(true);
  pAdvertising->addServiceUUID(FTMSDEVICE_FTMS_UUID);
  pAdvertising->setMinPreferred(0x06);  // set value to 0x00 to not advertise this parameter
  //Serial.println("Starting advertiser...");
  BLEDevice::startAdvertising();
  //Serial.println("Waiting a client connection to notify...");

  //Serial.println("Waiting for");

  // Serial.println("connection...");
}

void Calibrate() {

  //test run for min to max resistance and back
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println("Calibrating");
  Serial.println("Calibrating the PID value slowly");
  display.display();
  currentResistance = 127;
  ChangeResistance();
  delay(2000);
  currentResistance = 0;
  ChangeResistance();
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println("Done");
  display.display();
  ShowResistance();
  Serial.println("Done");
  delay(2000);
  ShowResistance();
}

void setup() {

  //pinModes setup
  //pinMode(LED, OUTPUT);
  pinMode(PWM_HARDER, OUTPUT);
  digitalWrite(PWM_HARDER, 0);
  pinMode(PWM_EASIER, OUTPUT);
  digitalWrite(PWM_EASIER, 0);
  input_val = analogRead(FEEDBACK_POT_PIN);

  UpBtn.begin();
  DownBtn.begin();

  delay(2000);


  //Setup the PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255,255);
  myPID.SetSampleTime(20);


  Serial.begin(9600);  //USB serial communication

  //Setup Display
  Wire.begin(0, 10);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(4);
  display.println("Initializing");
  display.display();
  delay(4000);

  Calibrate();

  BluetoothSetup();
}

void loop() {

  ReadButton();  //we want to read the buttons even if we are not connected to bluetooth app
  //Serial.println("scanning buttons");

  if (gearChanged) {
    Serial.println("Gear changed manually");
    ShowResistance();
    ChangeResistance();

  }  //check gear has changed before updating

  char windStr[6];   //6 spot array 1 byte each -wind speed
  char gradeStr[6];  //6 spot array 1 byte each - grade
  char rStr[4];      //4spot array 1 byte each  - rolling res
  char wStr[4];      //4 spot array 1 byte each - wind coef

  if (deviceConnected && !oldDeviceConnected) { //do stuff on connection
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Connected to central... ");
  }

  while (deviceConnected) {  //main loop for the BLE connection

    Serial.println("Connected to central, scan buttons first... ");
    ReadButton();  //read the buttons for gear shifts


    //indoor bike char values
    uint8_t bikeData[19] = {};  //set all the bytes to zero .  If we're only transmitting power, there's no point transmitting beyond byte 18
    bikeData[0] = 0x01;         // flags for Indoor bike-set bit 6 low of byte 0 to say power data not present
    //bit 1 set high for inst speed NOT present
    // bikeData[15] = 0x00;  //set bytes? 15 and 16 for the power (note that this is a signed int16 no power (0)
    // bikeData[16] = 0x00;
    uint8_t flags = bikeData[0];  //set to 0x01
    uint8_t dataToSend[1] = { flags };
    pIndoorBike->setValue(dataToSend, 1);
    pIndoorBike->notify();  //notify zwift

    //configure machine feature characteristic 8 fields- only set indoor bike simulation bit(field 5)
    uint8_t feature[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00 };  // 2^13 = bike simulation bit set byte 5.
    pFeature->setValue(feature, 8);                                           //send data to feature char- 8 locations from feature array
    pStatus->setValue(value);                                                 //send data to status char- value =0
    pStatus->notify();                                                        //notify zwift

    //Get the data written to the control point
    String rxValue = pControlPoint->getValue();
    if (rxValue.length() == 0) {
      Serial.println("No data received...");

    } else {

      Serial.print("Rx Value recieved : ");
      Serial.println(rxValue);

      switch (rxValue[0]) {
        case 0x00:  //Request control from zwift
          //reply with 0x80, 0x00, 0x01 to say OK
          replyDs[1] = rxValue[0];
          pControlPoint->setValue(replyDs, 3);
          pControlPoint->indicate();
          break;

        case 0x01:  //reset
          //reply with 0x80, 0x01, 0x01 to say OK
          replyDs[1] = rxValue[0];
          pControlPoint->setValue(replyDs, 3);
          pControlPoint->indicate();
          break;

        case 0x07:  //Start/resume
          //reply with 0x80, 0x07, 0x01 to say OK
          replyDs[1] = rxValue[0];
          pControlPoint->setValue(replyDs, 3);
          pControlPoint->indicate();
          break;
        case 0x11:  //receive simulation parameters
          /*
            In the case of Zwift:
            Wind is always constant, even with an aero boost (you just go faster)
            When trainer difficulty is set to maximum, the grade is as per the BLE spec (500 = 5%)
            Rolling resistance only ever seems to be 0 or 255.  255 occurs on mud/off road sections but not always (short sections only, like a patch of sand?)
            Wind coefficient doesn't change during a ride, even with an aero boost.  It may change depending on bike used in the game.
            Note: only using grade for TDF bike in 1% increments*/

          int16_t wind = rxValue[2] << 8;  //register changed from [1] in turbotrainer original
          wind |= rxValue[1];              //register changed from [0]in turbotrainer original

          uint8_t Rres = rxValue[5];  //register changed from [4]in turbotrainer original
          uint8_t Wres = rxValue[6];  //register changed from [5]in turbotrainer original

          grade = rxValue[4] << 8;
          grade |= rxValue[3];

          gradeFloat = grade / 100.0;


          if (debugging) {  // Remember, if debugging with Zwift, that these values are divided by 2 if in normal settings!
            Serial.print("Grade (100): ");
            Serial.println(gradeFloat);
          }

          break;
      }
    }

    delay(50);  //wait so that the BLE stack doesnt congest


    bool changeResist = false;
    resistanceSetting = round(gradeFloat*4);  //returns a difficulty setting
    if (oldResistance != resistanceSetting) {                                   //check it is changed before we do anything
      Serial.print("resistance setting changed to ... ");
      Serial.println(resistanceSetting);
      SetResistance(resistanceSetting);
      oldResistance = resistanceSetting;
      changeResist = true;
    }
    if (gearOffset != oldGear) {
      oldGear = gearOffset;
      changeResist = true;
    }


    if (changeResist) {
      ChangeResistance();
      ShowResistance();
    }
  }
  if (!deviceConnected && oldDeviceConnected) { //do stuff on disconnecting
    delay(300);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    //Serial.println("Nothing connected, start advertising");
    oldDeviceConnected = deviceConnected;
    Serial.println("Searching for central... ");
  }
  // connecting
}

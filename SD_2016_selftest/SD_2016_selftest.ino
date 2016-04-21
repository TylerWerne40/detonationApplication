#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define SHM_SWIP 42
#define SHM_testP 43
#define RxP 0
#define TxP 1
#define GPS_Rx 7
#define GPS_Tx 8
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

int incomingByte = 0; //for incoming data
int CC_test = 0;
int n = 1;
int SHM_SWI = HIGH;

int test(int);
SoftwareSerial XBee(RxP, TxP);
SoftwareSerial GPSS(8,7);
Adafruit_GPS GPS(&GPSS);
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup() {
  // put your setup code here, to run once:
  pinMode(SHM_SWIP, OUTPUT); //Continuity test switch
  pinMode(SHM_testP,INPUT);  //Result form continuity test
  Serial.begin(9600); // 9600 Baud Rate, uses pin 0/1 and USB
  XBee.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  XBee.print("+++"); //AT mode
  delay(1000);
  lcd.begin(16, 2);
  int time = millis();
  lcd.print("Hello, world!");
  time = millis() - time;
  Serial.print("Took "); Serial.print(time); Serial.println(" ms");
  lcd.setBacklight(WHITE);
}


void loop() {
  // put your main code here, to run repeatedly:
    if (XBee.available() > 0) {
      // read the incoming byte:
      incomingByte = XBee.read();
      CC_test = test(SHM_SWI);
      //Figure out what to do with incoming byte with Joe.
      //The following is to test the XBee comms
      Serial.print("I received: ");
      Serial.println(incomingByte, DEC);
      Serial.print("The CC test came back: ");
      if(CC_test = 1){
        Serial.println("Positive.");
      }
      else {
        Serial.println("Negative");
      }
    }
    else {
      Serial.println("No Transmission Found");
      CC_test = test(SHM_SWI);
      Serial.print("Our GPS coords are:");
      Serial.println(GPS.read(), DEC);
      Serial.print("The CC test came back: ");
      if(CC_test == HIGH){
        Serial.println("Positive.");
      }
      else {
        Serial.println("Negative");
      }
    }
    delay(1000);
    if(n) {
      lcd.setBacklight(BLUE);
      n = 0;
    }
    else {
      lcd.setBacklight(YELLOW);
      n = 1;
    }
}

int test(int test_cond) {
  digitalWrite(SHM_SWIP,test_cond);
  int test_ret = digitalRead(SHM_testP);
  return test_ret;
}


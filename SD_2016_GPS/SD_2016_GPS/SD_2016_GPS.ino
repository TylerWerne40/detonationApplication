#include <Adafruit_GPS.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <Wire.h>

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
SoftwareSerial mySerial(17, 16);
Adafruit_GPS GPS(&mySerial);
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup() {
  // put your setup code here, to run once:
  pinMode(SHM_SWIP, OUTPUT); //Continuity test switch
  pinMode(SHM_testP,INPUT);  //Result form continuity test
  Serial.begin(9600); // 115200 Baud Rate, uses pin 0/1 and USB
  GPS.begin(9600);
  Serial.println("Hi!");
  
}

char *buf; int gps_cnt, out;
void loop() {
  // put your main code here, to run repeatedly:'
  bool newData = false;
  delay(500);
  for (int k = 0;k<1000;k++){
      //while (GPS.available()) {//Continuously checks to see if data is coming in on Serial1
        char c = GPS.read();  //Read data from Serial1.
        Serial.print(c);
        if (GPS.encode(c)) {
          newData = true;
        }
        
      //}
   }
   if (newData) {
    float flat, flon;
    unsigned long age;
    GPS.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(GPS.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : GPS.satellites());
    Serial.print(" PREC=");
    Serial.print(GPS.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : GPS.hdop());
   }
  delay(1000);
}
int test(int test_cond) {
  digitalWrite(SHM_SWIP,test_cond);
  int test_ret = digitalRead(SHM_testP);
  return test_ret;
}


#include <Adafruit_RGBLCDShield.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);

#define TO_HEX(i) (i <= 9 ? '0' + i : 'A' - 10 + i)
#define ARM 41
#define EXPLODE 40
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

char ID [2]= {'1', '0'};
int CC_test = 0;
int n = 1;
int SHM_SWI = HIGH;
int idk;
int p = 0;
char monTest;
char msg [16];
char gpsPH;
float latitude;
char latitudeC;

int a = atoi("00");
int const Init = (int const) a;
int b = atoi("01");
int const Arm = (int const) b;
int c = atoi("02");
int const Disarm = (int const) c;
int d = atoi("03");
int const S_Chk = (int const) d;
int e = atoi("04");
int const Data_Only = (int const) e;
int f = atoi("05");
int const Explode_D = (int const) f;
int g = atoi("06");
int const Explode_Now = (int const) g;
int z = atoi("07");
int const Explode_Time = (int const) z;
boolean usingInterrupt = false;

void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
int test(int);
void EXECUTE (void);
SoftwareSerial XBee(RxP, TxP);
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();



class Comms {                       //used to communicate with XBEE
  public:
    void ID_Set(void);
    void Receive(void);
    void Auto_Update(void);
    void Send(void);
    void Creator(char [12]);
    int Command;
    char data [12];
    char ID_R [2];
  private:
    char CommandC [2];
    char Received [16];
    char sent [16];
    void set_vals(char [16]);
};

void Comms::set_vals(char input [16]) {   //parses message
  ID_R [0] = input [0];
  ID_R [1] = input [1];
  CommandC [0]= input [2];
  CommandC [1] = input [3];
  Command =  atoi(CommandC);
  for(int p = 0; p < 12; p++) {
    data[p] = input[p+4];
  }
}

void Comms::ID_Set(void) {  //sets ID
  ID[0] = ID_R[0];
  ID[1] = ID_R[1];
  char HP [12];
  snprintf( HP, sizeof HP, "ID received.");
  Creator(HP);
  Send();
}

void Comms::Receive (void) {
  //Received = msg;
  Serial.readBytes(msg, 16);
  set_vals(msg);
}

void Comms::Creator (char r [12]) {   // r:Data
  /*
  sent[0] = '0';
  sent[1] = '0';
  sent[2] = '0';
  sent[3] = '4';
  */
  snprintf(sent, sizeof sent, "000%d", 4);  //sends to user end and command is data only
  strcat(sent,(char *) r);                  //sends data
}

void Comms::Send(void) {
  Serial.print(sent);
  //Serial.write(sent);
}

Comms XBEE;

class GPSo {                  //Used to operate the GPS
  public:
    void waitT(int, int, int, int, int, int);
    void update(void);
    void begin(void);
  private:
};

void GPSo::begin(void) {
  pinMode(10, OUTPUT);

  // connect to the GPS at the desired rate
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_NOANTENNA);
  useInterrupt(true);
}

void GPSo::update(void) { //must have sei() somewhere before this
  if (GPS.newNMEAreceived()) {
    char *stringptr = GPS.lastNMEA();
    
    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    lcd.print(GPS.longitudeDegrees);
  } 
}

void GPSo::waitT(int year_R, int month_R, int day_R, int hour_R, int minute_R, int second_R) {
  sei();
  while((year_R <= GPS.year) && (month_R <= GPS.month) && (day_R <= GPS.day) && (hour_R <= GPS.hour) && (minute_R <= GPS.minute) && (second_R << GPS.seconds)) {
    update();
  }
  cli();
}

GPSo myGPS;               //used to operate the GPS

class unit_cntl {
  public:
  void EXECUTE(void);
  void StageS(void);
  void Auto_Update(void);
  int m;
  private: 
  int CCT;
  int num;
  int prev_C;
  char PH [12];
};

unit_cntl Control;          //used to control the unit

void unit_cntl::StageS() {  //runs the stage selector code
  XBEE.Receive();
  lcd.setBacklight(BLUE);
  if((msg [0]) == ID[0]) {  //first stage sets ID.
    if (m == 0) {
      XBEE.ID_Set();
      lcd.clear();
      m = 1;
    }
    else if ((msg [0] == ID [0]) && (msg [1] == ID [1])) { //all other stages exceutes a command
      lcd.clear();
      EXECUTE();
      m = 1;
    }
  }
  int i = 0;
  for(i = 0; i < 17; i++){
    msg[i] = '\0';
  }
}

void unit_cntl::Auto_Update(void) { //auto update code
  XBEE.Command = S_Chk;
  for (int i; i < 14; i++) {
    XBEE.data [i] = 0;
  }
  EXECUTE();
}

void unit_cntl::EXECUTE (void) { //Executes the command from the user end or from auto update
  if (XBEE.Command == Arm) {
      digitalWrite(ARM, HIGH);
      lcd.clear();
      lcd.setBacklight(YELLOW);
      lcd.print("ARMED");
      snprintf(PH, sizeof PH, "000000000000");
      XBEE.Creator(PH);
      XBEE.Send();
  }
  else if(XBEE.Command == Disarm) {
      digitalWrite(ARM, LOW);
      lcd.clear();
      lcd.setBacklight(WHITE);
      lcd.print("DISARMED");
  }

  else if(XBEE.Command == S_Chk) { //status check
      CC_test = test(SHM_SWI); //checks continuity
      if (CC_test == HIGH) {
        CCT = 1;
      }
      else {
        CCT = 0;
      }
      //send user end the continuity and the time down to milliseconds
      snprintf( PH, sizeof PH , "%d",CCT); 
      /*
      strcat( PH, (char*) GPS.hour);
      strcat( PH, (char*) GPS.minute);
      strcat( PH, (char*) GPS.seconds);
      strcat( PH, (char*) GPS.milliseconds);
      */
      for( int l = (1 /*+ sizeof(GPS.hour) + sizeof(GPS.minute) + sizeof(GPS.seconds) + sizeof(GPS.milliseconds)*/); l<12; l++) {
        PH [l] = '0';      
      }
      XBEE.Creator(PH);
      XBEE.Send();
      prev_C = XBEE.Command;
      num = 0;
      XBEE.Command = Data_Only;
      EXECUTE();
  }

  else if(XBEE.Command == Data_Only) {
      lcd.clear();
      lcd.setBacklight(GREEN);
      lcd.print((XBEE.data));
      if (prev_C == S_Chk) { // run if the previous command was S_Chk
        if (num == 0) { //does this first
          //send speed and altitude to user end
          snprintf( PH, sizeof PH , "%02.1f %04.0f",GPS.speed, GPS.altitude);
          for( int l = 8; l<12; l++) {
            PH [l] = 'A';      
          }
          num = 1;
          EXECUTE();
          XBEE.Creator(PH);
          XBEE.Send();
        }
        else if (num == 1) {
          // send latitude to user end
          snprintf( PH, sizeof PH , "%06f%", GPS.latitudeDegrees);
          strcat(PH, "N");
          for( int l = 7; l<12; l++) {
            PH [l] = 'A';      
          }
          XBEE.Creator(PH);
          XBEE.Send();
          num = 2;
          EXECUTE();
        }
        else if (num == 2) {
          //send longitude to user end
          snprintf( PH, sizeof PH , "%07f%", GPS.longitudeDegrees);
          for( int l = 8; l<12; l++) {
            PH [l] = 'A';      
          }
          XBEE.Creator(PH);
          XBEE.Send();
          num = 0;
        }
      }
  }

  else if(XBEE.Command == Explode_D) {
      lcd.setBacklight(RED);
      //Delay code
      int days = (int) XBEE.data [0];
      days = days*100;
      days += (int) XBEE.data [1];
      days += (int) XBEE.data [2];
      int hour = (int) XBEE.data [3];
      hour = hour*10;
      hour += (int) XBEE.data [4];
      int minutes = 10*((int) XBEE.data [5]);
      minutes += (int) XBEE.data [6];
      int seconds = 60*(minutes + 60*(hour + 24*(days)));
      seconds += 10*((int) XBEE.data [7]);
      seconds += (int) XBEE.data [8];
      int milli = 100*((int) XBEE.data [9]);
      milli += 10*((int) XBEE.data [10]);
      milli += (int) XBEE.data [11];
      delay((1000*seconds + milli));
      //end delay code
      //run detonation code
      digitalWrite(EXPLODE, HIGH);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.write((int) XBEE.data);
      XBEE.Creator("Allah Ackbar");
      XBEE.Send();
  }

  else if(XBEE.Command == Explode_Now) {
      //run detonation code
      lcd.setBacklight(RED);
      //snprintf(PH, sizeof PH, "AllahAckbar!");
      XBEE.Creator("Allah Ackbar");
      XBEE.Send();
      lcd.print("Exploded");
      digitalWrite(EXPLODE, HIGH);
  }
  else if(XBEE.Command == Explode_Time) {
      //Extract time desired from the message
      lcd.print("Exploding");
      int year_R = 2000;
      year_R += 10*((int) XBEE.data [0]);
      year_R += (int) XBEE.data [1];
      int month_R = 10*((int) XBEE.data [2]);
      month_R = (int) XBEE.data [3];
      int day_R = 10*((int) XBEE.data [4]);
      day_R += (int) XBEE.data [5];
      int hour_R = 10*((int) XBEE.data [6]);
      hour_R += (int) XBEE.data [7];
      int minutes_R = (int) XBEE.data [8];
      minutes_R += (int) XBEE.data [9];
      int second_R = 10*((int) XBEE.data [10]);
      second_R += (int) XBEE.data [11];
      //run GPS check time code
      myGPS.waitT(year_R, month_R, day_R, hour_R, minutes_R, second_R);
      //run detonation code
      lcd.setBacklight(RED);
      XBEE.Creator((char*) "Allah Ackbar");
      XBEE.Send();
      digitalWrite(EXPLODE, HIGH);
  }
/*
  else {
      //Serial.print("I received: ");
      //Serial.println(msg);
      lcd.setCursor(1,1);
      lcd.print("Default");
      snprintf(PH, sizeof PH, "bleh0000000");
      //strcat(PH,"1");
      lcd.setBacklight(RED);
      delay(400);
      lcd.setBacklight(YELLOW);
  }
  */
}
void setup() {
  myGPS.begin();    //begin GPS
  Control.m = 0;
  // make sure that the default chip select pin is set to
  pinMode(SHM_SWIP, OUTPUT);  //Continuity test switch
  pinMode(SHM_testP, INPUT);  //Result form continuity test
  pinMode(ARM, OUTPUT);       //Arm Pin
  pinMode(EXPLODE, OUTPUT);   //Explode Pin
  digitalWrite(ARM, LOW);
  digitalWrite(EXPLODE, LOW);
  Serial.begin(9600); // 9600 Baud Rate, uses pin 0/1 and USB
  XBee.begin(9600);
  monTest=0;
  XBee.print("+++"); //AT mode
  delay(1000);
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
  //Serial.println("Hi!");
  /*
    // Timer 4
  
    cli();                    // disable global interrupts
    TCCR4A = 0;               // set entire TCCR4A register to 0
    TCCR4B = 0;

    // enable Timer4 overflow interrupt:
    TIMSK4 = (1 << TOIE1);
    // Set CS bits for 1024 prescaler:
    TCCR4B |= (1 << CS10);
    TCCR4B |= (1 << CS12);
    // enable timer compare interrupt:
    TIMSK4 |= (1 << OCIE4A);
    sei();                    // enable global interrupts
    */
}

SIGNAL(TIMER0_COMPA_vect) {   //interrupt that runs the GPS
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
//  #ifdef UDR0
//      if (GPSECHO)
//        if (c) UDR0 = c;  
//      // writing direct to UDR0 is much much faster than Serial.print 
//      // but only one character can be written at a time. 
//  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  int i = 0;
  for(i = 0; i < 17; i++){
    msg[i] = '\0';
  }
  delay(400);
  if (Serial.available() > 0) {
    Control.StageS();
  }
  myGPS.update();  
}

int test(int test_cond) {   //runs the continuity checker
  digitalWrite(SHM_SWIP, test_cond);
  int test_ret = digitalRead(SHM_testP);
  return test_ret;
}


  ISR(TIMER4_COMPA_vect) {  //interrupt that runs the Auto Update
    //Do status check
    Control.Auto_Update();
  }


#include <Adafruit_RGBLCDShield.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

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

int CC_test = 0;
int n = 1;
int SHM_SWI = HIGH;
int idk;
char monTest;

char const Arm = 01;
char const Disarm = 02;
char const S_Chk = 03;
char const Data_Only = 04;
char const Explode_D = 05;
char const Explode_Now = 06;

int test(int);
SoftwareSerial XBee(RxP, TxP);
SoftwareSerial GPSS(8, 7);
Adafruit_GPS GPS(&GPSS);
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

class Comms {
  public:
    void Receive(void);
    void Auto_Update(void);
    void Send(void);
    void EXECUTE(void);
    void Creator(char, char);
    char Command;
  private:
    int CCT;
    char Received [16];
    char sent [16];
    char data [14];
    void set_vals(char [16]);
    char ID = 3;
    char ID_R;
    char PH;
    char gpsPH;
};

void Comms::set_vals(char input [16]) {
  input[0] = TO_HEX(((ID & 0x00F0) >> 4));
  char tmp[16];
  tmp[0] = input[0];
  tmp[1] = input[1];
  tmp[2] = '\0';
  ID_R = atoi(tmp);
  tmp[0] = input[2];
  tmp[1] = input[3];
  tmp[2] = '\0';
  Command = atoi(tmp);
  for (int i = 4; i < 16; i++) {
    data[i - 4] = input[i];
  }
}

void Comms::Auto_Update(void) {
  Command = 10;
  for (int i; i < 14; i++) {
    data [i] = 0;
  }
  ID_R = 0x01;
  EXECUTE();
}

void Comms::Receive (void) {
  for (int i; i < 16; i++) {
    Received [i] = Serial.read();
  }
  set_vals(Received);
}

void Comms::Creator (char q, char r) {   // p:ID, q:Command, r:Data
  sent[0] = TO_HEX(((ID & 0x00F0) >> 4));
  sent[1] = TO_HEX((ID & 0x000F));
  //Write the command.
  sent[2] = 0;
  sent[3] = Command + '0';
  if (data == NULL) {
    for (int i = 4; i < 16; i++) {
      sent[i] = 0;
    }
  }
  else {
    for (int i = 4; i < 16; i++) {
      sent[i] = data[i - 4];
    }
  }
}

void Comms::Send(void) {
  XBee.write(sent);
}

void Comms::EXECUTE (void) {
  switch (Command) {

    case Arm:
      digitalWrite(ARM, HIGH);
      lcd.setBacklight(YELLOW);
      break;

    case Disarm:
      digitalWrite(ARM, LOW);
      lcd.setBacklight(WHITE);
      break;

    case S_Chk:
      CC_test = test(SHM_SWI);
      Serial.print("Our GPS coords are:");
      Serial.println(GPS.read(), DEC);
      Serial.print("The CC test came back: ");
      if (CC_test == HIGH) {
        Serial.println("Positive.");
        CCT = 1;
      }
      else {
        Serial.println("Negative");
        CCT = 0;
      }
      PH |= CCT;
      Creator(Command, PH);
      Send();
      break;

    case Data_Only:
      break;

    case Explode_D:
      lcd.setBacklight(RED);
      //Make delay code
      digitalWrite(EXPLODE, HIGH);
      break;

    case Explode_Now:
      lcd.setBacklight(RED);
      digitalWrite(EXPLODE, HIGH);
      break;

    default:
      Serial.print("I received: ");
      Serial.println(XBee.read(), DEC);
      gpsPH = GPS.read();
      PH = (gpsPH << 1);
      lcd.setBacklight(RED);
      delay(100);
      lcd.setBacklight(YELLOW);
      delay(100);
      Creator(0x04, 0xF8);
      Send();
  }
}

Comms XBEE;

void setup() {
  pinMode(SHM_SWIP, OUTPUT);  //Continuity test switch
  pinMode(SHM_testP, INPUT);  //Result form continuity test
  pinMode(ARM, OUTPUT);       //Arm Pin
  pinMode(EXPLODE, OUTPUT);   //Explode Pin
  digitalWrite(ARM, LOW);
  digitalWrite(EXPLODE, LOW);
  Serial.begin(9600); // 9600 Baud Rate, uses pin 0/1 and USB
  XBee.begin(9600);
  monTest=0;
  /*
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
  */
  XBee.print("+++"); //AT mode
  delay(1000);
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
  Serial.println("Hi!");
  // Timer 4
  /*
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

char msg [16] = {0};
void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0) {
    // read the incoming byte:
    //Figure out what to do with incoming byte with Joe.
    //The following is to test the XBee comms

    //Serial.print("I received: ");
    //Serial.println(Serial.read(), DEC);
    // for (int i = 0;i < 16; i++) {
    //   msg [i]=Serial.read();
    //}
    msg[0] = Serial.read();
      lcd.print(msg[0]);
    if (msg[0] == '9') {
      //for (int i = 0; i < 16; i++) {
      //  Serial.write(msg [i]);
      //}
      Serial.write("0HeyMoises345678");
      if(monTest == 0){
        monTest=1;
        lcd.setBacklight(RED);
        digitalWrite(EXPLODE, HIGH);
      }
      else{
        monTest=0;
        lcd.setBacklight(WHITE);
        digitalWrite(EXPLODE,LOW);
      }
    }
    //XBEE.Receive();
    //XBEE.EXECUTE();
    /*
      XBEE.Receive();
      XBEE.EXECUTE();

      Serial.print("The CC test came back: ");
      if(CC_test = 1){
      Serial.println("Positive.");
      }

      else {
      Serial.println("Negative");
      }
    */
  }
  /*
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
  */
}

int test(int test_cond) {
  digitalWrite(SHM_SWIP, test_cond);
  int test_ret = digitalRead(SHM_testP);
  return test_ret;
}

/*
  ISR(TIMER4_COMPA_vect) {
    Serial.println("works!");
    //Do status check
    XBEE.Auto_Update();
  }
*/

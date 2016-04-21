#include <SoftwareSerial.h>

#define SHM_SWI 42
#define SHM_test 43
#define RxP 0
#define TxP 1

int incomingByte = 0; //for incoming data
int CC_test = 0;

int test(int);

void setup() {
  // put your setup code here, to run once:
  pinMode(SHM_SWI, OUTPUT); //Continuity test switch
  pinMode(SHM_test,INPUT);  //Result form continuity test
  Serial.begin(9600); // 9600 Baud Rate, use 1,2,3?
}


void loop() {
  // put your main code here, to run repeatedly:
      CC_test = test(SHM_SWI);
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

int test(int test_cond) {
  digitalWrite(SHM_SWI,test_cond);
  int test_ret = digitalRead(SHM_test);
  return test_ret;
}


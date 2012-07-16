#include <IRremote.h>

IRsend irsend;
unsigned int bytes[] = {100,100,100,100,100,100};

#define PIN 13
#define INTERUPT 0 //digital 2

int val = 0;
bool sendingIR = false;

void setup(){
  Serial.begin(9600);
  pinMode(PIN,OUTPUT);
  attachInterrupt(INTERUPT,interuptHandler,LOW);
}

void interuptHandler(){
  if(!sendingIR){
    val = 1;
    Serial.println("INTERUPT");
  }
}

void loop(){

  sendIR();
  delay(100);
  receiveIR();

  //delay(1000);
}


void sendIR(){
  sendingIR = true;
  for(int i = 0; i < 3; i++){
    irsend.sendRaw(bytes, 6,40);
    //irsend.sendSony(0xa90, 12);
    //delay(10);
  }
  sendingIR = false;
}

void receiveIR(){
  if (val != 0) {
    digitalWrite(PIN,HIGH);
    Serial.println(val);
  }else{
    digitalWrite(PIN,LOW);
  }
  val = 0;
}

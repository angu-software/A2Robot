
#define PIN 13
bool on=true;

void setup(){
  pinMode(PIN,OUTPUT);
}

void loop(){
  digitalWrite(13,on);
  //on = !on;
  delay(25);
}

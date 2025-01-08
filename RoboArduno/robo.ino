#define PIN8 8 
#define PIN9 9 
#define PIN10 10 
#define PIN11 11
void setup() {
  pinMode(13, OUTPUT);
  pinMode(PIN8, OUTPUT);
  pinMode(PIN9, OUTPUT);
  pinMode(PIN10, OUTPUT);
  pinMode(PIN11, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  digitalWrite(PIN8,HIGH);
  digitalWrite(PIN9,HIGH);
  digitalWrite(PIN10,HIGH);
  digitalWrite(PIN11,HIGH);
  delay(1000);
  digitalWrite(13, LOW);
   digitalWrite(PIN8,LOW);
  digitalWrite(PIN9,LOW);
  digitalWrite(PIN10,LOW);
  digitalWrite(PIN11,LOW);
  delay(1000);
}
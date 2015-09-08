int ledpin = 13;

void setup() {
  //Set's pinmode to LED pin
  Serial.begin(9600);
  
  pinMode(ledpin, OUTPUT);
    
}

void loop() {
     digitalWrite(ledpin, HIGH);
     delay(2000);
     digitalWrite(ledpin, LOW);
     delay(2000);
}

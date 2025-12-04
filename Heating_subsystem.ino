const int analogPin=A0;
const float Vcc=3.3;
const float Rref=10000.0;
unsigned long pre=0;
const long interval=1000;
const int pwmPin=8;
void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, OUTPUT);
  // put your setup code here, to run once:
}

void loop() {
  float duty=0.3;
  int pwmV=duty*255;
  analogWrite(pwmPin, pwmV);
  unsigned long now=millis();
  if(now-pre>=interval)
  {
    pre=now;
    float vadc=analogRead(analogPin);
    float Vout=(vadc/4095)*Vcc;
    float Rx=Rref*(Vout/(Vcc-Vout));
  
    Serial.print("Vout = ");
    Serial.print(Vout);
    Serial.print(" V, R= ");
    Serial.print(Rx);
    Serial.println("ohms");

  }
  // put your main code here, to run repeatedly:

}

const int dirPin1 = 6;
const int stepPin1 = 8;
const int stepsPerRevolution = 200;
const int dirPin2 = 7;
const int stepPin2 = 9;
//const int stepsPerRevolution = 200;
const int dirPin3 = 4;
const int stepPin3 = 46;

int rep=1;
int sideLength=3;
//const int stepsPerRevolution = 200;
/*
int dir=0;
int speed=4;
int speedFactor=100;
*/
void setup()
{
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
}

void loop()
{
  delay(5000);
  if (rep<=0) return;
  //Side1
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, LOW);
  digitalWrite(dirPin3, HIGH);
  for (int i=0; i<stepsPerRevolution*sideLength; i++){
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, HIGH);
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(2000);
  }
  
  //Side2
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, HIGH);
  digitalWrite(dirPin3, HIGH);
  for (int i=0; i<stepsPerRevolution*sideLength; i++){
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(2000);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(2000);
  }

  //Side3
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, HIGH);
  digitalWrite(dirPin3, LOW);
  for (int i=0; i<stepsPerRevolution*sideLength; i++){
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(2000);
  }
  rep--;
}

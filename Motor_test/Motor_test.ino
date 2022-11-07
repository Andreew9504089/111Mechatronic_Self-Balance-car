const int L298N_IN1 = 7;
const int L298N_IN2 = 3;
const int L298N_IN3 = 6;
const int L298N_IN4 = 5;
const int L298N_ENA = 11;
const int L298N_ENB = 10;


void setup() {
  pinMode (L298N_IN1, OUTPUT);
  pinMode (L298N_IN2, OUTPUT);
  pinMode (L298N_IN3, OUTPUT);
  pinMode (L298N_IN4, OUTPUT);
  pinMode (L298N_ENA, OUTPUT);
  pinMode (L298N_ENB, OUTPUT);

  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, LOW);
  analogWrite(L298N_ENA, LOW);
  analogWrite(L298N_ENB, LOW); 
}

void forward(int speed){
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, HIGH);
  digitalWrite(L298N_IN3, HIGH);
  digitalWrite(L298N_IN4, LOW);
  analogWrite(L298N_ENA, speed);
  analogWrite(L298N_ENB, speed); 
}

void backward(int speed){
  digitalWrite(L298N_IN1, HIGH);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, HIGH);
  analogWrite(L298N_ENA, speed);
  analogWrite(L298N_ENB, speed); 
}

void stall(){
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, LOW);
  analogWrite(L298N_ENA, LOW);
  analogWrite(L298N_ENB, LOW); 
}

void loop() {
  int speed = 125;
  forward(speed);
  delay(5000);
  backward(speed);
  delay(5000);
  stall();
  delay(3000);
}

//Conection

//Servo - direction

//Analog pot ->  A0
//l298N ENB  -> D7
//l298N IN3  -> D6
//l298N IN4  -> D3
#define SERVO_POT A0

// Motor DC
//l298N ENA  -> D12
//l298N IN1  -> D11
//l298N IN2  -> D8
#define ENA  12
#define IN1  11
#define IN2  8


void setup() {                
  pinMode(ENA, OUTPUT);  
 pinMode(IN1, OUTPUT); 
pinMode(IN2, OUTPUT);  
  Serial.begin(115200);  
}

void loop() {
  
  delay(500);
  Serial.println(analogRead(SERVO_POT));
  
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite(ENA,55);
  delay(2000);
  analogWrite(ENA,105);
  delay(2000);
  analogWrite(ENA,255);
  delay(2000);
  analogWrite(ENA,10);
  delay(2000);
  
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  analogWrite(ENA,50);
  delay(2000);
  analogWrite(ENA,200);
  delay(2000);
  
}

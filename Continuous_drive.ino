
// connect motor controller pins to Arduino digital pins
//left motor:
int enA = 9;
int in1 = 8;
int in2 = 7;
int left_speed = 120;


//right motor:
int enB = 6;
int in3 = 5;
int in4 = 4;
int right_speed = 140;


int myTimeout = 90;

int driveTime=350;
double rover=3; //(inches) width of rover (wheel to wheel)
double rotInches;

// Define functions
void InitMotors(void);
double MoveForward(void);
double rotate(void);

char move;
volatile long motCount1 = 0;
volatile long motCount2 = 0;

void setup() {
 
// Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial.setTimeout(myTimeout);

  InitMotors();
}
String cmdStr;
int cmd;
double msg_val;
void loop() {

if(Serial.available() > 0) { 
  //delay(20);
  
  cmdStr= Serial.readString();
  
 // delay(20);
 // Serial.println(cmdStr);
  
  
  // LED code for debugging
  if(cmdStr.charAt(0) == '1') {

  digitalWrite(13, HIGH);
  //Serial.println("on!");
}
else if(cmdStr.charAt(0) == '0') {
  digitalWrite(13, LOW);
  //Serial.println("off!");
}
// drive commands eg. '[d:1] or [r:1]'
else if(cmdStr.charAt(0) == '[') {

//Remove characters to get only the inch distance value
cmdStr.remove(-1);
move=cmdStr.charAt(1);
//Serial.println(move);

cmdStr.remove(0,3);
msg_val = cmdStr.toFloat();


if (move=='d'){
  
 // Serial.println("d");
  MoveForward(msg_val);
  
}
else if(move=='c'){
  cont();
  //Serial.println("Move fwd");
}
else if(move=='r'){ //maybe change dig pins to low
  //delay(200);
  rotate(msg_val);
 // Serial.println("r");
}
else if(move=='x'){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
delay(10);
//Serial.println(msg_val);

}
    
}
delay(1);
}
void InitMotors(void)
{
// set all the motor control pins to outputs
pinMode(enA, OUTPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);

pinMode(enB, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);

}
int cont(){
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);

digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);

analogWrite(enA, left_speed);
analogWrite(enB, right_speed);

}
int MoveForward(double movInches)
{
// turn on motor in forward direction
if(movInches>0){
//Serial.println("fwd");
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);

digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
}
else{
 // Serial.println("bwd");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
// set speed to 200 out of possible range 0~255
analogWrite(enA, left_speed);
analogWrite(enB, right_speed);
delay(driveTime*abs(movInches));
//Serial.println(abs(movInches));
//delayMicroseconds(17777*movInches);
analogWrite(enA, 0);
analogWrite(enB, 0);
}

double rotate(double deg){

rotInches=rover*(deg*3.1415/180);
//Serial.println(rotInches);

if(deg<0){
 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
else{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
// set speed to 200 out of possible range 0~255
analogWrite(enA, left_speed);
analogWrite(enB, right_speed);
delay(driveTime*0.45*abs(rotInches)); //0.45 for fast //0.6 for 100/125
//delayMicroseconds(17777*movInches);
analogWrite(enA, 0);
analogWrite(enB, 0);
}

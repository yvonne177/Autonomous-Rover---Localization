#include<SoftwareSerial.h>

//SoftwareSerial bluetooth(10,11); //rx of bluetooth 11, tx of bluetooth 10

SoftwareSerial driveUno(10, 11); //to read stuff from drive uno 10-1, 11-0
int myTimeout = 90;
const int trigPin = 3;
const int trigPin2 = 2;
const int echoPin1 = 4; // FRONT
const int echoPin2 = 5; // RIGHT 1
const int echoPin3 = 6; // RIGHT 2
const int echoPin4 = 7; // LEFT 1
const int echoPin5 = 8; //LEFT 2
const int echoPin6 = 9; //BACK
const int echoPin7 = 13; //RIGHT FRONT CORNER
const int echoPin8 = 12; //LEFT FRONT CORNER
//long pingTime;

String p1=",";
String msg;

//input in sofia's code
double distance1;
double distance2;
double distance3;
double distance4;
double distance5;
double distance6;
double distance7;
double distance8;

char dist[20]; //DELETE LATER

String val = " ";

void setup() 
{
  Serial.begin(9600);//bluetooth
  Serial.setTimeout(myTimeout);
 
  driveUno.begin(9600);

  pinMode(trigPin, OUTPUT); // sets the trigPin as an output
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin1, INPUT); // sets echoPin as an input
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(echoPin4, INPUT);
  pinMode(echoPin5, INPUT);
  pinMode(echoPin6, INPUT);
  pinMode(echoPin7, INPUT);
  pinMode(echoPin8, INPUT);


}
String answer;
void loop() 
{ 
  
   
   // Serial.println(val);
  if (Serial.available()>0) {
     val = Serial.readString();
    

   
    if(val.charAt(1)=='d'|| val.charAt(1)=='r'||val.charAt(1)=='x'||val.charAt(1)=='c'||val.charAt(0)=='1'||val.charAt(0)=='0'){
      
      driveUno.println(val);
     // delay(2000);6
      //Serial.write(driveUno.read());
    
     // driveUno.write(val.c_str());
      //Serial.println(driveUno.available());
      //answer=driveUno.readString();
      //Serial.println(answer);
      delay(10);
      
    }
    //*************************WE HAVE TO DO IT LIKE THIS OTHERWISE IT HATES ITSELF********************************************
    if (val.charAt(2)=='0'){ //FRONT 
      
  
      digitalWrite(trigPin, LOW); // resets trigPin LOW
      delay(5); // adjustable delay
      digitalWrite(trigPin, HIGH); // trigger for 10 us
      delay(5);
      digitalWrite(trigPin, LOW);
      //pingTime1 = pulseIn(echoPin1, HIGH);
      distance1 = (0.5/2.54)*0.034*pulseIn(echoPin1, HIGH); // half of speed of light, multipy by inch conversion - 1 inch = 2.54 cm
      delay(5);
      //itoa(distance1,dist,10); //ignore
      Serial.println(distance1);
    }
    else if (val.charAt(2)=='1'){ //RIGHT 1
      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance2 = (0.5/2.54)*0.034*pulseIn(echoPin2, HIGH); // half of speed of light
      delay(5);
      Serial.println(distance2);
    }
    else if (val.charAt(2)=='2'){ //RIGHT 2
      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance3 = (0.5/2.54)*0.034*pulseIn(echoPin3, HIGH); // half of speed of light
      delay(5);
      Serial.println(distance3);  
    }
    else if (val.charAt(2)=='3'){ //LEFT 1
      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance4 = (0.5/2.54)*0.034*pulseIn(echoPin4, HIGH); // half of speed of light
      delay(5);
      Serial.println(distance4);
    }
    else if (val.charAt(2)=='4'){ //LEFT 2
      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance5 = (0.5/2.54)*0.034*pulseIn(echoPin5, HIGH); // half of speed of light
      delay(5);
      Serial.println(distance5);
    }
    else if (val.charAt(2)=='5'){ //BACK
      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance6 = (0.5/2.54)*0.034*pulseIn(echoPin6, HIGH); // half of speed of light
      delay(5);
      Serial.println(distance6);
    }
    else if (val.charAt(2)=='6'){ //FRONT RIGHT CORNER
      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance7 = (0.5/2.54)*0.034*pulseIn(echoPin7, HIGH); // half of speed of light
      delay(5);
      Serial.println(distance7);
      
    }
    else if (val.charAt(2)=='7'){ //FRONT LEFT CORNER
      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance8 = (0.5/2.54)*0.034*pulseIn(echoPin8, HIGH); // half of speed of light
      delay(5);
      Serial.println(distance8);
    }
    else if (val.charAt(2)=='s'){
      digitalWrite(trigPin, LOW); // resets trigPin LOW
      delay(5); // adjustable delay
      digitalWrite(trigPin, HIGH); // trigger for 10 us
      delay(5);
      digitalWrite(trigPin, LOW);
      distance1 = (0.5/2.54)*0.034*pulseIn(echoPin1, HIGH); // half of speed of light, multipy by inch conversion - 1 inch = 2.54 cm
      delay(5);

      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance2 = (0.5/2.54)*0.034*pulseIn(echoPin2, HIGH); // half of speed of light
      delay(5);

      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance3 = (0.5/2.54)*0.034*pulseIn(echoPin3, HIGH); // half of speed of light
      delay(5);

      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance4 = (0.5/2.54)*0.034*pulseIn(echoPin4, HIGH); // half of speed of light
      delay(5);

      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance5 = (0.5/2.54)*0.034*pulseIn(echoPin5, HIGH); // half of speed of light
      delay(5);

      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance6 = (0.5/2.54)*0.034*pulseIn(echoPin6, HIGH); // half of speed of light
      delay(5);

      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance7 = (0.5/2.54)*0.034*pulseIn(echoPin7, HIGH); // half of speed of light
      delay(5);

      digitalWrite(trigPin, LOW);
      delay(5);
      digitalWrite(trigPin, HIGH);
      delay(5);
      digitalWrite(trigPin, LOW);
      distance8 = (0.5/2.54)*0.034*pulseIn(echoPin8, HIGH); // half of speed of light
      delay(5);

      Serial.println(distance1 + p1 + distance2 + p1 + distance3 + p1 + distance4 + p1 + distance5 + p1 + distance6 + p1 + distance7 + p1 + distance8); //displays all distance
    
    }
    

    delay(5);
    //Serial.println(input);
   // Serial.print(" ");
  //delay(100);
   // bluetooth.print(input);
//    bluetooth.print(" ");
   
    //delay(5);


  }


}


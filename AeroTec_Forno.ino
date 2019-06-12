
#define T_SETPOINT 50

//#define DEBUG

#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <stdlib.h>

// set the LCD address to 0x27 for a 16 chars 2 line display
// A FEW use address 0x3F
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address


// SAMPLING
#define SAMPLING_T 100 // in ms
unsigned long previousMillis;
#define N_SENSORS 3
#define N_AVG 5

// TEMPERATURE
float T[N_SENSORS][N_AVG];
int currentT = 0;
int TPin[N_SENSORS];
float TSetPoint;
#define TSP_PIN A3
float stdT;

// RELAY
#define RELAY_PIN 12

// STATE
int stateG;
int stateC;
#define OFF 0
#define ON 1
#define MALFUNCTIONING_ON 2
#define MALFUNCTIONING_OFF 3

//SWITCH
#define SWITCH_PIN 2
int stateS;
unsigned long OnSince;
#define SWITCH_TIME 2000

// TERESHOLDS
#define STD_THRESHOLD 8
#define T_THRESHOLD 0.2

#include <Servo.h>
Servo ESC;
#define PIN_MOTOR 9
#define THROTTLE 1175


void setup() {

  Serial.begin(9600);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  
  digitalWrite(RELAY_PIN, LOW);

  previousMillis = 0;
  currentT = 0;

  stateG = OFF;
  stateC = OFF;
    
  TPin[1] = A0;
  TPin[2] = A1;
  TPin[3] = A2;

  Serial.println("SETUP");
  //TSetPoint = getTSetPoint();
  TSetPoint = T_SETPOINT;
  Serial.println(TSetPoint);

  
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.backlight(); // finish with backlight on  

  ESC.attach(PIN_MOTOR);
  ESC.writeMicroseconds(1000);
}

void loop() {

  unsigned long currentMillis = millis();
  float Tc;
  
  if (currentMillis - previousMillis > SAMPLING_T){
    Tc = getTemperature();
    
    if(stateG == ON){
      control(Tc);
      //Serial.println(Tc);
    previousMillis = currentMillis;
    }else{
      digitalWrite(RELAY_PIN, LOW);
      stateC = OFF;
    }
  }
  
  /*if(stateG == OFF){
    TSetPoint = getTSetPoint();
  }*/

  if(stateG == ON){
    ESC.writeMicroseconds(THROTTLE);
  }else{
    ESC.writeMicroseconds(1000);
  }

  
  
  checkState();
  printScreen(Tc);
  
  

  delay(100);
}




// TEMPERATURE
float getTemperature() {

    T[0][currentT] = analogRead(A0);
    //T[0][currentT] = (T[0][currentT]*(5000)/(1023.0)-750)/10.0+25;
    T[0][currentT] = (T[0][currentT]*(5000)/(1023.0)-1000)/10.0+50;
    delay(10);

    T[1][currentT] = analogRead(A1);
    //T[1][currentT] = (T[1][currentT]*(5000)/(1023.0)-750)/10.0+25;
    T[1][currentT] = (T[1][currentT]*(5000)/(1023.0)-1000)/10.0+50;
    delay(10);
    
    T[2][currentT] = analogRead(A2);
    //T[2][currentT] = (T[2][currentT]*(5000)/(1023.0)-750)/10.0+25;
    T[2][currentT] = (T[2][currentT]*(5000)/(1023.0)-1000)/10.0+50;
    delay(10);
  

#ifdef DEBUG
  for (int i = 0; i < N_SENSORS; i++) {
    Serial.print(T[i][currentT]);
    Serial.print(' ');
  }
  Serial.println();
#endif

  currentT++;
  if (currentT >= N_AVG){
    currentT = 0;
  }

  float Tc = 0.0;

  for(int i = 0; i<N_SENSORS; i++){
    for(int j = 0; j<N_AVG; j++){
      Tc = Tc + T[i][j];     
    }
  }
  Tc = Tc/(N_AVG*N_SENSORS);
 
  
  stdT = 0;
   for(int i = 0; i<N_SENSORS; i++){
    for(int j = 0; j<N_AVG; j++){
      stdT += pow((T[i][j]-Tc),2);     
    }
  }
  
  stdT = stdT/(N_AVG*N_SENSORS);
  stdT = sqrt(stdT);

  if(stateG == ON && stdT > STD_THRESHOLD){
    stateG = MALFUNCTIONING_ON;
  }else if(stateG == OFF && stdT > STD_THRESHOLD){
    stateG = MALFUNCTIONING_OFF;
  }else if(stdT <= STD_THRESHOLD){
    if(stateG == MALFUNCTIONING_OFF){
      stateG = OFF;
    }else if(stateG == MALFUNCTIONING_ON){
      stateG = ON;
    }
  }
  
  //Serial.println(std);
  
  return Tc;
}

void control(float Tc){

  //Serial.println(stateG);
  
  if(stateG != ON)
    return;

  if(stateC == ON && Tc > TSetPoint + T_THRESHOLD){
    digitalWrite(RELAY_PIN, LOW);
    stateC = OFF;
  }else if(stateC == OFF && Tc < TSetPoint - T_THRESHOLD){
    digitalWrite(RELAY_PIN, HIGH);
    stateC = ON;
  }
  
}

float getTSetPoint(){
  int reading = analogRead(TSP_PIN);
  delay(5);
  return 0.1*round(10*map(reading, 0, 1024, 45, 90));
}

void checkState(){
  int reading = digitalRead(2);
  if(reading == LOW){
    if(stateS == ON && millis()-OnSince> SWITCH_TIME){
      if(stateG < MALFUNCTIONING_ON){
         stateG = -stateG + 1;
         stateS = OFF;
      }
    }else if(stateS == OFF){
      stateS = ON;
      OnSince = millis();
    }   
  }else{
    stateS = OFF;
  }
}

void printScreen(float Tc){
  lcd.clear();
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print("T:");   lcd.print(Tc);  lcd.print(" SD:"); lcd.print(stdT); 
  lcd.setCursor(0,1);
  lcd.print("TSP:"); lcd.print(TSetPoint);
  switch(stateG){
    case ON:{
     lcd.print("   ON");
     break; 
    }
    case OFF:{
     lcd.print("   OFF");
     break; 
    }
    case MALFUNCTIONING_ON:{
     lcd.print("ERROR ON");
     break; 
    }
    case MALFUNCTIONING_OFF:{
     lcd.print("ERROR OFF");
     break; 
    }
  }
}



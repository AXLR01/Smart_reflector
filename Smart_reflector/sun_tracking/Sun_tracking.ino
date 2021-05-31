#include <RCSwitch.h> //Library to communicate by radio frequency 433MHz
#include <EEPROM.h>

#define NORD  A5
#define EST   A2 
#define SUD   A3
#define OUEST A4 

const int MotorIN_North_East = 11; //Direction
const int MotorIN_South_West = 3;
const int MotorEN_North_South = 7; //Engine
const int MotorEN_West_East = 8;

unsigned long time_previousCommand, latence(200); //in milliseconds

RCSwitch myRadio = RCSwitch();
unsigned long data = 0;   //Store data information received

bool automatic_mode(false), alreadyStop(true), inCorrectionOE(false), inCorrectionNS(false);
int Speed(98), TrSN(55), TrOE(-5), val_min_soleil(0), average;
int ErrorSN(15), ErrorOE(25); //TrSN and TrOE need to be upgraded automatically

void setup(){
  DDRC=0xFF;
  //Set motors pins
  pinMode(MotorIN_North_East, OUTPUT);
  pinMode(MotorIN_South_West, OUTPUT);
  pinMode(MotorEN_North_South, OUTPUT);
  pinMode(MotorEN_West_East, OUTPUT);
  digitalWrite(MotorEN_North_South, LOW);
  digitalWrite(MotorEN_West_East, LOW);

  //Initiate PWM on Timer 2
  TCCR2A = 0b10100011;
  TCCR2B = 0b00000001;
  OCR2A = 0; //D11
  OCR2B = 0; //D3 
  time_previousCommand = millis() + latence;

  //Set luminosity sensor pins
  pinMode(NORD,INPUT);
  pinMode(SUD,INPUT);
  pinMode(OUEST,INPUT);
  pinMode(EST,INPUT);

  //Extract the last position set values
  TrSN = (int)EEPROM.read(0) + (int)(EEPROM.read(1) << 8);
  TrOE = (int)EEPROM.read(2) + (int)(EEPROM.read(3) << 8);
  Speed = 0;

  //Set the RF 433 MHz receiver pin
  myRadio.enableReceive(0); // Receiver on interrupt at digital pin #2

  //Set communication with the PC
  Serial.begin(9600);
  Serial.println("Start OK");
}

void loop(){
  if(myRadio.available()){
    data = myRadio.getReceivedValue() & 255; //Save the data received
    time_previousCommand = millis() + latence;

    //Cases of remote control data
    switch(data){
      case 192:
        Serial.println("nord");
        engine_control(true, true, 98); // 50
        automatic_mode = false;
        break;
      case 48:
        Serial.println("sud");
        engine_control(true, false, 98);
        automatic_mode = false;
        break;
      case 240:
        Serial.println("ouest");
        engine_control(false, true, 98); // 45
        automatic_mode = false;
        break;
      case 3:
        Serial.println("Button 8");
        break;
      case 12:
        Serial.println("est");
        engine_control(false, false, 98); // 45
        automatic_mode = false;
        break;
      case 204:
        Serial.println("Button 5");
        automatic_mode = true;
        break;
      case 60:
        Serial.println("Button 6");
        automatic_mode = false;
        break;
      case 252:
        Serial.println("Button 7");
        TrSN = analogRead(SUD) - analogRead(NORD);
        TrOE = analogRead(OUEST) - analogRead(EST);
        average = (analogRead(NORD) + analogRead(OUEST))*0.5;
        ErrorOE = average*0.08; //Sensibility factor OUEST EST
        ErrorSN = average*0.04; //Sensibility factor NORD SUD
        val_min_soleil = average*0.5;

        //Enregistrement dans l'EEPROM
        EEPROM.write(0, lowByte(TrSN));
        EEPROM.write(1, highByte(TrSN));
        EEPROM.write(2, lowByte(TrOE));
        EEPROM.write(3, highByte(TrOE));
        break;
        
      default:
        Serial.print(data);
        Serial.println("->  Unknown");
        break;
    }
    myRadio.resetAvailable();
    
  }else{
    average = (analogRead(NORD) + analogRead(OUEST))*0.5;
    
    if(automatic_mode && average > val_min_soleil){
      time_previousCommand = millis() + latence;
      asservissement();
     
    }
    if(millis() > time_previousCommand && !alreadyStop){
      alreadyStop = true;
      Serial.println("Engine stopped");      
      digitalWrite(MotorEN_North_South, LOW);
      digitalWrite(MotorEN_West_East, LOW);
      OCR2A=0;
      OCR2B=0;      
    }
  }
}

void engine_control(bool vertical, bool forward, int vitesse){
  //Max vitesse = 100
  //Min vitesse = 1
  if(vertical){ //North / South
    if(forward)
        send_simple_PWM(MotorEN_North_South, MotorIN_North_East, vitesse); //North
    else
        send_simple_PWM(MotorEN_North_South, MotorIN_South_West, vitesse); //South
  }else{  //Left / Right
    if(forward)
        send_simple_PWM(MotorEN_West_East, MotorIN_South_West, vitesse); //West
    else
        send_simple_PWM(MotorEN_West_East, MotorIN_North_East, vitesse); //East
  }
}

void send_simple_PWM(const int enable_pin, const int engine_pin, int duty_cycle){
  alreadyStop = false;
  
  if(enable_pin == MotorEN_North_South){
    digitalWrite(MotorEN_West_East, LOW);
    digitalWrite(MotorEN_North_South, HIGH);    
  }else if(enable_pin == MotorEN_West_East){
    digitalWrite(MotorEN_North_South, LOW);
    digitalWrite(MotorEN_West_East, HIGH);
  }

  if(engine_pin == MotorIN_North_East){
    OCR2A=0;
    OCR2B=(duty_cycle*255)/100;
  }else if(engine_pin == MotorIN_South_West){
    OCR2B=0;
    OCR2A=(duty_cycle*255)/100;
  }
}


void asservissement(){
  int Nord = analogRead(NORD);
  int Sud = analogRead(SUD);
  int Est = analogRead(EST);
  int Ouest = analogRead(OUEST);
  bool flaggySN(false), flaggyOE(false), verti, forw;
  //flaggy are to know if the system has not to move: true = no move necessary ; false = repositionning necessary
  //verti is to select the engine: true = NORD SUD engine ; false = OUEST EST engine
  //forw is to select the direction of the engine: true = forward ; false = backward
  int TrOE_max, TrOE_min, TrSN_max, TrSN_min;
  //These values are to set the error intervals for OUEST-EST and SUD-NORD
  int differenceOE(Ouest-Est), differenceSN(Sud-Nord);
  //Save the voltage digit difference between two opposite panels in order to save calculation process

  //We set a smaller error interval when it corrects position than when it is waiting
  if(inCorrectionOE){ //For OUEST EST
    TrOE_max = TrOE + ErrorOE/2;
    TrOE_min = TrOE - ErrorOE/2;
  }else {
    TrOE_max = TrOE + ErrorOE;
    TrOE_min = TrOE - ErrorOE;
  }
  if(inCorrectionNS){ //For NORD SUD
    TrSN_max = TrSN + ErrorSN/2;
    TrSN_min = TrSN - ErrorSN/2;
  }else {
    TrSN_max = TrSN + ErrorSN;
    TrSN_min = TrSN - ErrorSN;
  }

  //Watch if the system is in the error invervals
  if(differenceSN<TrSN_max && differenceSN>TrSN_min){ //If position North South OK
    flaggySN = true;
    inCorrectionNS = false;
    digitalWrite(MotorEN_North_South,LOW);
  }
  if(differenceOE<TrOE_max && differenceOE>TrOE_min){ //If position West East OK
    flaggyOE = true;
    inCorrectionOE = false;
    digitalWrite(MotorEN_West_East,LOW);
  }
  
  //And if position NS or OE is not OK, the system corrects his position and activate engines
  if(!flaggySN||!flaggyOE){
    
    //OUEST EST control and regulation
    if(!flaggyOE){
      inCorrectionOE = true;

      //Light is more on OUEST on the panel
      if(differenceOE<TrOE_min){
        forw = true;
        if(differenceOE<TrOE_min-ErrorOE){
          if(Speed<70){
            Speed++;
          }
        }else if(differenceOE<TrOE_min && Speed >35){
          Speed--;
        }
      }

      //Light is more on EST on the panel
      if(differenceOE>TrOE_max){
        forw = false;
        if(differenceOE>TrOE_max+ErrorOE){
          if(Speed<70){
            Speed++;
          }
        } else if(differenceOE>TrOE_max && Speed>35){
          Speed--;
        }
      }
      //Send instructions to engines
      verti = false;
      engine_control(verti,forw,Speed);
      return;
    }
   
    //NORD SUD control and regulation
    if(!flaggySN){
      inCorrectionNS = true;
      
      //Set minimum speed to 30
      if(Speed<70){
        Speed = 70;
      }
      
      //Light is more on SUD on the panel
      if(differenceSN<TrSN_min){
        forw = false;
        if(differenceSN<TrSN_min-ErrorSN){
          if(Speed<100){
            Speed+=10;
          }
        }else if(differenceSN<TrSN_min && Speed >70){ 
          Speed--;
        }
      }
      //Light is more on NORD on the panel
      if(differenceSN>TrSN_max){
        forw = true;
        if(differenceSN>TrSN_max+ErrorSN){
          if(Speed<100){
            Speed+=10;
          }
        } else if(differenceSN>TrSN_max && Speed>70){
            Speed--;
        }
      }
      verti = true;
      //Send instruction to engines
      engine_control(verti,forw,Speed);
      return;
    }
  }
}

#include <RCSwitch.h> //Library to communicate by radio frequency 433MHz

#define RapportCyclique  98

// capteur de fin de course gauche et droite
#define END_GAUCHE 12  
#define END_DROITE 13 
#define END_HAUT 10   
#define END_BAS 9   

// constante (moteurs)
// pins ou sont connectés le driver moteur
const int MotorIN_North_East = 11; //Direction
const int MotorIN_South_West = 3;
const int MotorEN_North_South = 7; //Engine
const int MotorEN_West_East = 8;

// varriable de fin de course
int fin_gauche=0;
int fin_droite=0;
int fin_haut=0;
int fin_bas=0;

RCSwitch myRadio = RCSwitch();
unsigned long data = 0;   //Store data information received

bool automatic_mode(false);

void setup() {
  DDRC=0xFF;
  //Set motors pins
  //Configuration des pattes moteurs en sortie
  //*******************OUTPUT= sortie et INPUT= Entrée******************
  pinMode(MotorIN_North_East, OUTPUT);
  pinMode(MotorIN_South_West, OUTPUT);
  pinMode(MotorEN_North_South, OUTPUT);
  pinMode(MotorEN_West_East, OUTPUT);
  //************************* Low =0  et HIGH=5v *********************
  digitalWrite(MotorEN_North_South, LOW);
  digitalWrite(MotorEN_West_East, LOW);

    //Initiate PWM on Timer 2
  TCCR2A = 0b10100011;
  TCCR2B = 0b00000001;
  OCR2A = 0; //D11
  OCR2B = 0; //D3 

    //Set the RF 433 MHz receiver pin
  myRadio.enableReceive(0); // Receiver on interrupt at digital pin #2

}

void loop() {
  // put your main code here, to run repeatedly:
   if(myRadio.available()){
    data = myRadio.getReceivedValue() & 255; //Save the data received

    //Cases of remote control data
    switch(data){
      case 192: // bouton 1 - monter
        Serial.println("nord");
          engine_control(true, true, RapportCyclique);
          automatic_mode = false;
          break;
      case 48:
        Serial.println("sud"); // bouton 2 - descendre
          engine_control(true, false, RapportCyclique); // 91 valeur de la vitesse en pourcentage
          automatic_mode = false;
          break;
      case 240:  // bouton 4 - droite
        Serial.println("ouest");
           engine_control(false, true, RapportCyclique);
           automatic_mode = false;
           break;
      case 3:
        Serial.println("Button 8");
        break;
      case 12:  // bouton 3 - gauche
        Serial.println("est");
           engine_control(false, false, RapportCyclique);
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
   //     Calibration();
        break;
      default:
        Serial.print(data);
        Serial.println("->  Unknown");
        break;
    }
    myRadio.resetAvailable();
    
  }else{
    if(automatic_mode==true){ 
       balayageZonz ();
  }
  }

}

// fonction qui fait déplacer le moteur fin de course inclu
void engine_control(bool vertical, bool forward, int vitesse){
  //Max vitesse = 100
  //Min vitesse = 1
  if(vertical){ //North / South
    if(forward){
        fin_haut=digitalRead(END_HAUT);  // se déplacer vers le Nord
        if (fin_haut==HIGH){             //si le fin de course n'est pas atteint faire le mouvement
        send_simple_PWM(MotorEN_North_South, MotorIN_North_East, vitesse); //North
        }else{send_simple_PWM(MotorEN_North_South, MotorIN_North_East, 0);} // sinon arreter le mouvement
    }else{
        fin_bas=digitalRead(END_BAS);  // se deplacer vers le sud
        if (fin_bas==HIGH){
        send_simple_PWM(MotorEN_North_South, MotorIN_South_West, vitesse); //South
        } else {send_simple_PWM(MotorEN_North_South, MotorIN_South_West, 0);}
    }
  }else{  //Left / Right
    if(forward){
        fin_gauche=digitalRead(END_GAUCHE); // se deplacer vers l'ouest
        if (fin_gauche==HIGH){
        send_simple_PWM(MotorEN_West_East, MotorIN_South_West, vitesse); //West
        }else {send_simple_PWM(MotorEN_West_East, MotorIN_South_West, 0);}
    }else{
        fin_droite=digitalRead(END_DROITE); // se deplacer vers l'est
        if (fin_droite==HIGH){
        send_simple_PWM(MotorEN_West_East, MotorIN_North_East, vitesse); //East
        }else{send_simple_PWM(MotorEN_West_East, MotorIN_North_East, 0);}
    }
  }
}

// fonction qui controle la direction du moteur (Oest Est Sud ou Nord)
void send_simple_PWM(const int enable_pin, const int engine_pin, int duty_cycle){
//  alreadyStop = false;
  
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
//fonction pour se deplacer vers le nord
void goNord(){
  engine_control(true, true, RapportCyclique);
  }
  
//...sud
void goSud (){
  engine_control(true, false, RapportCyclique);
  }

//***ouest
void goOuest() {
  engine_control(false, true, RapportCyclique);
  }  

//***est
void goEst(){
  engine_control(false, false, RapportCyclique);
  }
  
// fonction pour effectuer le balayage de la zonne
void balayageZonz (){
  do {
    goSud();
    }while (fin_bas==HIGH); // ok

   do {
    goOuest();
    }while (fin_gauche==HIGH);// ok

      do {
    goEst();
    }while (fin_droite==HIGH); // ok

      do {
    goNord();
    }while (fin_haut==HIGH); // ok
}

   

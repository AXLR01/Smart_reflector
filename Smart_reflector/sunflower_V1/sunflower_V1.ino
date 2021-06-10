#include <RCSwitch.h> //Library to communicate by radio frequency 433MHz
#include <EEPROM.h>

#define NORD  A5
#define EST   A2
#define SUD   A3
#define OUEST A4
#define soleil 19
#define END_GAUCHE 12
#define END_DROITE 13
#define END_HAUT 10
#define END_BAS 9
#define TIMENS 5000
#define TIMEOE 5000
#define TIMEEND 90000

int latenceNS = 1500;
int latenceOE = 8000;
int largeur_de_scrutation = 0;
int val_min_lum = 410;

const int MotorIN_North_East = 11; //Direction
const int MotorIN_South_West = 3;
const int MotorEN_North_South = 7; //Engine
const int MotorEN_West_East = 8;

unsigned long time_previousCommand, latence(200); //in milliseconds

RCSwitch myRadio = RCSwitch();
unsigned long data = 0;   //Store data information received

bool automatic_mode(false), first_depart(false), calibrationOK(false), alreadyStop(true), depart_sens(true), calibration_sens(false), inCorrectionOE(false), inCorrectionNS(false);
int Speed(98), TrSN(55), TrOE(-5), val_min_soleil(0), average;
int ErrorSN(15), ErrorOE(25); //TrSN and TrOE need to be upgraded automatically

void setup() {
  DDRC = 0xFF;
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
  pinMode(NORD, INPUT);
  pinMode(SUD, INPUT);
  pinMode(OUEST, INPUT);
  pinMode(EST, INPUT);
  pinMode(soleil, INPUT);

  pinMode(END_GAUCHE, INPUT);
  pinMode(END_DROITE, INPUT);
  pinMode(END_HAUT, INPUT);
  pinMode(END_BAS, INPUT);

  //Extract the last position set values
  TrSN = (int)EEPROM.read(0) + (int)(EEPROM.read(1) << 8);
  TrOE = (int)EEPROM.read(2) + (int)(EEPROM.read(3) << 8);
  Speed = 0;

  //Set the RF 433 MHz receiver pin
  myRadio.enableReceive(0); // Receiver on interrupt at digital pin #2

}

void loop() {
  
  if (myRadio.available())
  {
    data = myRadio.getReceivedValue() & 255; //Save the data received
    time_previousCommand = millis() + latence;

    //Cases of remote control data
    switch (data)
    {
      case 192:
        //Serial.println("nord");
        /*while ((digitalRead(END_HAUT) == HIGH) && (data == 192)) // appui une fois et ca monte tant que pas FC ou autre appui
          {
          if (data == 192)
          {
            engine_control(true, true, 98);
            automatic_mode = false;
          }
          data = myRadio.getReceivedValue() & 255; //Save the data received
          }*/

        if (digitalRead(END_HAUT) == HIGH) // check si en buté
        {
          while ( ( data == 192) && (digitalRead(END_HAUT) == HIGH) ) // peut monter tant que appui sur bon bouton et pas en buté
          {
            
            engine_control(true, true, 98);
            automatic_mode = false;
            data = 0;
          }
        }
        break;

      case 48:
        //Serial.println("sud");
        engine_control(true, false, 98);
        automatic_mode = false;
        break;

      case 240:
        //Serial.println("ouest");

        engine_control(false, false, 98);
        automatic_mode = false;

        break;

      case 12:
        //Serial.println("est");

        engine_control(false, true, 98);
        automatic_mode = false;

        break;

      case 204:
        //Serial.println("Button 5");
        automatic_mode = false;
        break;

      case 60:
        //Serial.println("Button 6");
        automatic_mode = false;
        break;

      case 252:
        //Serial.println("Button 7");
        Calibration();
        asservissement();
        automatic_mode = true;
        break;

      case 3:
        //Serial.println("Button 8");
        automatic_mode = true;
        fastCalibration();
        slowCalibration();
        Calibration();
        asservissement();
        break;

      default:
        Serial.print(data);
        Serial.println("->  Unknown");
        break;
    }
    myRadio.resetAvailable();
  }
  else
  { average = (analogRead(NORD) + analogRead(OUEST)) * 0.5;

    if (automatic_mode && average > val_min_soleil)
    {
      time_previousCommand = millis() + latence;
      asservissement();
    }
    if (millis() > time_previousCommand && !alreadyStop)
    {
      alreadyStop = true;
      digitalWrite(MotorEN_North_South, LOW);
      digitalWrite(MotorEN_West_East, LOW);
      OCR2A = 0;
      OCR2B = 0;
    }
  }
}

void engine_control(bool vertical, bool forward, int vitesse)
{
  //Max vitesse = 100
  //Min vitesse = 1
  if (vertical) //North / South
  {
    if (forward)
      send_simple_PWM(MotorEN_North_South, MotorIN_North_East, vitesse); //North
    else
      send_simple_PWM(MotorEN_North_South, MotorIN_South_West, vitesse); //South
  }
  else //Left / Right
  {
    if (forward)
      send_simple_PWM(MotorEN_West_East, MotorIN_South_West, vitesse); //West
    else
      send_simple_PWM(MotorEN_West_East, MotorIN_North_East, vitesse); //East
  }
}

void send_simple_PWM(const int enable_pin, const int engine_pin, int duty_cycle)
{
  alreadyStop = false;
  if (enable_pin == MotorEN_North_South)
  {
    digitalWrite(MotorEN_West_East, LOW);
    digitalWrite(MotorEN_North_South, HIGH);
  }
  else if (enable_pin == MotorEN_West_East)
  {
    digitalWrite(MotorEN_North_South, LOW);
    digitalWrite(MotorEN_West_East, HIGH);
  }

  if (engine_pin == MotorIN_North_East)
  {
    OCR2A = 0;
    OCR2B = (duty_cycle * 255) / 100;
  }
  else if (engine_pin == MotorIN_South_West)
  {
    OCR2B = 0;
    OCR2A = (duty_cycle * 255) / 100;
  }
}

//stop_engine
void stop_engine()
{
  digitalWrite(MotorEN_North_South, LOW);
  digitalWrite(MotorEN_West_East, LOW);
  OCR2A = 0;
  OCR2B = 0;
}

// Asservissement avec les modifications du calcul de dalai de retour
void asservissement()
{
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
  int differenceOE(Ouest - Est), differenceSN(Sud - Nord);
  //Save the voltage digit difference between two opposite panels in order to save calculation process

  //We set a smaller error interval when it corrects position than when it is waiting
  if (inCorrectionOE) { //For OUEST EST
    TrOE_max = TrOE + ErrorOE / 2;
    TrOE_min = TrOE - ErrorOE / 2;
  } else {
    TrOE_max = TrOE + ErrorOE;
    TrOE_min = TrOE - ErrorOE;
  }
  if (inCorrectionNS) { //For NORD SUD
    TrSN_max = TrSN + ErrorSN / 2;
    TrSN_min = TrSN - ErrorSN / 2;
  } else {
    TrSN_max = TrSN + ErrorSN;
    TrSN_min = TrSN - ErrorSN;
  }

  //Watch if the system is in the error invervals
  if (differenceSN < TrSN_max && differenceSN > TrSN_min) { //If position North South OK
    flaggySN = true;
    inCorrectionNS = false;
    digitalWrite(MotorEN_North_South, LOW);
  }
  if (differenceOE < TrOE_max && differenceOE > TrOE_min) { //If position West East OK
    flaggyOE = true;
    inCorrectionOE = false;
    digitalWrite(MotorEN_West_East, LOW);
  }

  //And if position NS or OE is not OK, the system corrects his position and activate engines
  if (!flaggySN || !flaggyOE) {

    //OUEST EST control and regulation
    if (!flaggyOE) {
      inCorrectionOE = true;

      //Light is more on OUEST on the panel
      if (differenceOE < TrOE_min) {
        forw = true;
        if (differenceOE < TrOE_min - ErrorOE) {
          if (Speed < 70) {
            Speed++;
          }
        } else if (differenceOE < TrOE_min && Speed > 35) {
          Speed--;
        }
      }

      //Light is more on EST on the panel
      if (differenceOE > TrOE_max) {
        forw = false;
        if (differenceOE > TrOE_max + ErrorOE) {
          if (Speed < 70) {
            Speed++;
          }
        } else if (differenceOE > TrOE_max && Speed > 35) {
          Speed--;
        }
      }
      //Send instructions to engines
      verti = false;
      engine_control(verti, forw, Speed);
      return;
    }

    //NORD SUD control and regulation
    if (!flaggySN) {
      inCorrectionNS = true;

      //Set minimum speed to 30
      if (Speed < 70) {
        Speed = 70;
      }

      //Light is more on SUD on the panel
      if (differenceSN < TrSN_min) {
        forw = false;
        if (differenceSN < TrSN_min - ErrorSN) {
          if (Speed < 100) {
            Speed += 10;
          }
        } else if (differenceSN < TrSN_min && Speed > 70) {
          Speed--;
        }
      }
      //Light is more on NORD on the panel
      if (differenceSN > TrSN_max) {
        forw = true;
        if (differenceSN > TrSN_max + ErrorSN) {
          if (Speed < 100) {
            Speed += 10;
          }
        } else if (differenceSN > TrSN_max && Speed > 70) {
          Speed--;
        }
      }
      verti = true;
      //Send instruction to engines
      engine_control(verti, forw, Speed);
      return;
    }
  }
}

// regarde si les cellules sont illuminées
bool isSensorFound()
{
  if ((analogRead(NORD) > 100) && (analogRead(EST) > 100))
  {
    return true;
  }
  else {
    return false;
  }
}

// fait la spiral egyptien pour chercher grossièrement le soleil
// démarre d'une position random pour go haut droite
void fastCalibration()
{
  if (isSensorFound() == true)
  {
    return;
  }
  else
  {
    while (((digitalRead(END_HAUT) == HIGH) && (digitalRead(END_DROITE) == HIGH))) // tant que FC haut et droite sont pas atteint et que le capteur n'est pas trouvé
    {
      engine_control(false, false, 98); // go droite
      while (digitalRead(END_DROITE) == HIGH)
      {
        if (isSensorFound() == true)
        {
          return;
        }
      } // tant que FC droite ouvert
      stop_engine();

      if (digitalRead(END_HAUT) == HIGH) // si je suis pas en buté haute
      {
        engine_control(true, true, 98);
        delay(5000);
      } stop_engine();

      if (digitalRead(END_HAUT) == HIGH) // si je suis pas en buté haute
      {
        engine_control(false, true, 98); // go gauche
        while (digitalRead(END_GAUCHE) == HIGH)
        {
          if (isSensorFound() == true)
          {
            return;
          }
        } // tant que FC gauche ouvert
        stop_engine();
      }

      if (digitalRead(END_HAUT) == HIGH) // si je suis pas en buté haute
      {
        engine_control(true, true, 98);
        delay(5000);
      } stop_engine();
    }
  }
  stop_engine();
}

// On relève 3 fois la valeur de chaque cellules avec 0,5 sec de delais entre chauque relevé puis on réajuste la position
void slowCalibration()
{
  int valOuest, valEst, valNord, valSud, cmpt(0); // variable lecture cellule et une variable itération de boucle

  for (cmpt = 0; cmpt <= 3; cmpt++)
  {
    valOuest += analogRead(OUEST);
    valEst += analogRead(EST);
    valNord += analogRead(NORD);
    valSud += analogRead(SUD);
    delay(500);
  }

  if (((valNord - valSud) != 0) && ((valEst - valOuest) != 0))
  {
    if ((valEst - valOuest) > 0)
    {
      while (valEst < valOuest) // si O<E go ouest, on lit l'état des cellules à chaque tour
      {
        if (digitalRead(END_DROITE) == HIGH)
        {
          engine_control(false, true, 98);
          valOuest = analogRead(OUEST);
          valEst = analogRead(EST);
        }
      }
    }

    if (valOuest - valEst > 0)
    {
      while (valOuest > valEst) // si E<O go est, on lit l'état des cellules à chaque tour
      {
        if (digitalRead(END_GAUCHE) == HIGH)
        {
          engine_control(false, false, 98);
          valOuest = analogRead(OUEST);
          valEst = analogRead(EST);
        }
      }
    }
    if (valNord - valSud > 0)
    {
      while (valNord > valSud) // si N>S go sud
      {
        if (digitalRead(END_BAS) == HIGH)
        {
          engine_control(true, false, 98);
          valSud = analogRead(SUD);
          valNord = analogRead(NORD);
        }
      }
    }

    if (valSud - valNord)
    {
      while (valNord < valSud) // si N<S go nord
      {
        if (digitalRead(END_HAUT) == HIGH)
        {
          engine_control(true, true, 98);
          valSud = analogRead(SUD);
          valNord = analogRead(NORD);
        }
      }
    }
  }
  else
  {
    return;
  }
}

//CALIBRATION
void Calibration() // save les valeurs des capteurs afin d'avoir un indicateur
{
  TrSN = analogRead(SUD) - analogRead(NORD);
  TrOE = analogRead(OUEST) - analogRead(EST);
  average = (analogRead(NORD) + analogRead(OUEST)) * 0.5;
  ErrorOE = average * 0.08; //Sensibility factor OUEST EST
  ErrorSN = average * 0.04; //Sensibility factor NORD SUD
  val_min_soleil = average * 0.5;

  //Enregistrement dans l'EEPROM
  EEPROM.write(0, lowByte(TrSN));
  EEPROM.write(1, highByte(TrSN));
  EEPROM.write(2, lowByte(TrOE));
  EEPROM.write(3, highByte(TrOE));
}

bool dayOrNight()
{
  if (analogRead(soleil) > 100)
  {
    return true;
  }
  if (analogRead(soleil) < 41)
  {
    return false;
  }
}

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

  // fin de course possible qu'ils soient inversé
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
        engine_control(true, true, 98);
        automatic_mode = false;
        break;
      case 48:
        //Serial.println("sud");
        engine_control(true, false, 98);
        automatic_mode = false;
        break;
      case 240:
        //Serial.println("ouest");
        engine_control(false, true, 98);
        automatic_mode = false;
        break;
      case 12:
        //Serial.println("est");
        engine_control(false, false, 98);
        automatic_mode = false;
        break;
      case 204:
        //Serial.println("Button 5");
        automatic_mode = true;
        break;
      case 60:
        //Serial.println("Button 6");
        automatic_mode = false;
        break;
      case 252:
        //Serial.println("Button 7");
        Calibration();
        break;
      case 3:
        //Serial.println("Button 8");
        fastCalibration();
        break;

      default:
        Serial.print(data);
        Serial.println("->  Unknown");
        break;
    }
    myRadio.resetAvailable();
  }
  else
  {
    average = (analogRead(NORD) + analogRead(OUEST)) * 0.5;
    if (automatic_mode == true)
    {
      time_previousCommand = millis() + latence;
      if (dayOrNight() == false)
      {
        if (calibrationOK == true)
        {
          normal_utilisation();
        }
        else
        {
          process_calibration();
        }
      }
      else
      {
        calibrationOK == false;
        first_depart = false;
      }
    }
    if (millis() > time_previousCommand && !alreadyStop)
    {
      alreadyStop = true;
      Serial.println("Engine stopped");
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
  Speed = 98;

  //flaggy are to know if the system has not to move: true = no move necessary ; false = repositionning necessary
  //verti is to select the engine: true = NORD SUD engine ; false = OUEST EST engine
  //forw is to select the direction of the engine: true = forward ; false = backward
  bool flaggySN(false), flaggyOE(false), verti, forw;

  //These values are to set the error intervals for OUEST-EST and SUD-NORD
  int TrOE_max, TrOE_min, TrSN_max, TrSN_min;
  int differenceOE(Ouest - Est), differenceSN(Sud - Nord);

  //We set a smaller error interval when it corrects position than when it is waiting
  if (inCorrectionOE) //For OUEST EST
  {
    TrOE_max = TrOE + ErrorOE / 2;
    TrOE_min = TrOE - ErrorOE / 2;
  }
  else
  {
    TrOE_max = TrOE + ErrorOE;
    TrOE_min = TrOE - ErrorOE;
  }
  if (inCorrectionNS) //For NORD SUD
  {
    TrSN_max = TrSN + ErrorSN / 2;
    TrSN_min = TrSN - ErrorSN / 2;
  }
  else
  {
    TrSN_max = TrSN + ErrorSN;
    TrSN_min = TrSN - ErrorSN;
  }

  //Watch if the system is in the error intervals
  if (differenceSN < TrSN_max && differenceSN > TrSN_min) //If position North South OK
  {
    flaggySN = true;
    inCorrectionNS = false;
    digitalWrite(MotorEN_North_South, LOW);
  }
  if (differenceOE < TrOE_max && differenceOE > TrOE_min) //If position West East OK
  {
    flaggyOE = true;
    inCorrectionOE = false;
    digitalWrite(MotorEN_West_East, LOW);
  }

  //And if position NS or OE is not OK, the system corrects his position and activate engines
  if (!flaggySN || !flaggyOE)
  {

    //OUEST EST control and regulation
    if (!flaggyOE)
    {
      inCorrectionOE = true;

      //Whether light is more on OUEST or on EST on the panel
      forw = (differenceOE < TrOE_min) ? false : true;

      //Check if it's the first move
      if (first_depart == false)
      {
        depart_sens = forw; // the direction is stored to be used for the next day scrutation
        first_depart = true;
      }

      //Send instructions to engines
      verti = false;

      engine_control(verti, forw, Speed);

      //verifierSens(forw);
      return;
    }

    //NORD SUD control and regulation
    if (!flaggySN)
    {
      inCorrectionNS = true;

      //Whether light is more on SUD or NORTH on the panel
      forw = (differenceSN < TrSN_min) ? true : false;

      //Send instruction to engines
      verti = true;

      engine_control(verti, forw, Speed);
      return;
    }
  }
}

// regarde si les cellules sont illuminées
// 410 trop élevé ne parais pas mais 100 pas mal
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
      delay(1000);
    } stop_engine();
  }

  stop_engine();
}

//Find the calibration point for the horizontal axis
void findCalibrationPointHz() // fait 3 releves consecutif de valeur O et E, fait la diff pour savoir s'il doit gauche/droite/pas bouger
{
  int valOuest, valEst;
  int diff = 0;
  int point = 0;
  int i;
  do
  {
    engine_control(false, calibration_sens, 98);
    for (i = 0; i <= 3; i++)
    {
      valOuest += analogRead(OUEST);
      valEst += analogRead(EST);
      delay(500);
    }
    diff = valOuest - valEst;
    if (diff < 0)
    {
      if (calibration_sens == true)
      {
        point++;
      }
      calibration_sens = false;
    }
    else if (diff > 0)
    {
      if (calibration_sens == false)
      {
        point++;
      }
      //      calibration_sens = true;
    }
    else
    {
      break;
    }
    valOuest = 0;
    valEst = 0;

  } while (point < 3);
  stop_engine();
  //automatic_mode == false;
}

//DIFF NORD/SUD
void findCalibrationPointVt() // fait 3 releves consecutif de valeur H et B, fait la diff pour savoir s'il doit haut/bas/pas bouger
{
  int valNord, valSud;
  int diff = 0;
  int i = 0;
  int point = 0;
  bool sens = false;
  do
  {
    engine_control(true, sens, 100);
    for (i = 0; i <= 3; i++)
    {
      valNord += analogRead(NORD);
      valSud += analogRead(SUD);
      delay(500);
    }
    diff = valNord - valSud;
    if (diff < 0)
    {
      if (sens == true)
      {
        point++;
      }
      sens = false;
    }
    else if (diff > 0)
    {
      if (sens == false)
      {
        point++;
      }
      sens = true;
    }
    else //si on a zéro on sort
    {
      break;
    }
    valNord = 0;
    valSud = 0;
  } while (point < 3); // -erreur <diff< erreur
  stop_engine();
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
  int current_val_min_lum = analogRead(soleil) * 0.5;
  val_min_lum = (val_min_lum < current_val_min_lum) ? val_min_lum : (current_val_min_lum < 41) ? val_min_lum : current_val_min_lum;

  //store luminosity sensor value
  //store
  //Enregistrement dans l'EEPROM
  EEPROM.write(0, lowByte(TrSN));
  EEPROM.write(1, highByte(TrSN));
  EEPROM.write(2, lowByte(TrOE));
  EEPROM.write(3, highByte(TrOE));
}

bool dayOrNight()
{
  if (analogRead(soleil) > 410)
  {
    return true;
  }
  if (analogRead(soleil) < 41)
  {
    return false;
  }
}


void normal_utilisation()
{
  if (isSensorFound() == true)
  {
    asservissement();
  }
  else
  {
    if (dayOrNight())
    {
      delay(900000);

    }
  }
}
void process_calibration()
{
  if (isSensorFound() == true)
  {
    fastCalibration();
    findCalibrationPointHz();
    findCalibrationPointVt();
    Calibration();
    calibrationOK = true;
  }
  else
  {
    if (dayOrNight())
    {

    }
  }
}

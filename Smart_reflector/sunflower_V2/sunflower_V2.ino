#include <RCSwitch.h> //Library to communicate by radio frequency 433MHz
#include <EEPROM.h>

// Pins cellules photovoltaiques
#define NORD  A5
#define EST   A2
#define SUD   A3
#define OUEST A4
#define soleil 19

// Pins fins de courses
#define END_GAUCHE 12
#define END_DROITE 13
#define END_HAUT 10
#define END_BAS 9

// Pins moteurs
const int MotorIN_North_East = 11; //Direction
const int MotorIN_South_West = 3;
const int MotorEN_North_South = 7; //Engine
const int MotorEN_West_East = 8;

unsigned long time_previousCommand, latence(200); //in milliseconds

// Passage de la fontcion RCswitch en variable et déclaration d'une variable data
RCSwitch myRadio = RCSwitch();
unsigned long data = 0;   //Store data information received

// Varaibles pour le fonctionnement du systeme
bool automatic_mode(false), alreadyStop(true); // variable passage en mode auto et check etat du systeme
bool inCorrectionOE(false), inCorrectionNS(false); // ?

int Speed(98); // definition de la vitesse moteur
int TrSN(55); // = S-N
int TrOE(-5); // = O-E
int val_min_soleil(0); // definition d'un seuil de tolerance
int average; // moyenne entre la cellule nord et sud

//TrSN and TrOE need to be upgraded automatically
// correspond a une tolerance
int ErrorSN(15); // = average * un coef
int ErrorOE(25); // = average * un coef

void setup() {
  DDRC = 0xFF;

  // Configuration pins moteurs
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

  // Configuration pins cellules photovoltaiques
  pinMode(NORD, INPUT);
  pinMode(SUD, INPUT);
  pinMode(OUEST, INPUT);
  pinMode(EST, INPUT);
  pinMode(soleil, INPUT);

  // Configuration pins fins de courses
  pinMode(END_GAUCHE, INPUT);
  pinMode(END_DROITE, INPUT);
  pinMode(END_HAUT, INPUT);
  pinMode(END_BAS, INPUT);

  // Lecture de l'EEPROM
  TrSN = (int)EEPROM.read(0) + (int)(EEPROM.read(1) << 8);
  TrOE = (int)EEPROM.read(2) + (int)(EEPROM.read(3) << 8);
  Speed = 0;

  // Configuration de la radio frequence 433 MHz
  myRadio.enableReceive(0); // Receiver on interrupt at digital pin #2
}

void loop() {

  if (myRadio.available()) // Si la telecommande envoie une donnee
  {
    data = myRadio.getReceivedValue() & 255; // stockage de la donne et conversion
    time_previousCommand = millis() + latence;


    switch (data) // Selon le button qu'on a presse
    {
      case 192: // bouton 1 on monte la surface
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
          engine_control(true, true, 98);
          automatic_mode = false;
        }
        break;

      case 48: // bouton 2 on descend la surface
        if (digitalRead(END_BAS) == HIGH) // check si en buté
        {
          engine_control(true, false, 98);
          automatic_mode = false;
        }
        break;

      case 240: // bouton 3 on tourne vers la gauche
        if (digitalRead(END_GAUCHE) == HIGH) // check si en buté
        {
          engine_control(false, false, 98);
          automatic_mode = false;
        }
        break;

      case 12: // bouton 4 on tourne vers la droite
        if (digitalRead(END_DROITE) == HIGH) // check si en buté
        {
          engine_control(false, true, 98);
          automatic_mode = false;
        }
        break;

      case 204: // bouton 5
        fastCalibration();
        automatic_mode = false;
        break;

      case 60: // bouton 6
        automatic_mode = false;
        break;

      case 252: // bouton 7 releve de l etat des cellules et suivi soleil
        Calibration();
        asservissement();
        automatic_mode = true;
        break;

      case 3: // bouton 8 recherche du soleil, centrage du capteur, releve des cellules, suivi soleil
        automatic_mode = true;
        fastCalibration();
        slowCalibration();
        Calibration();
        asservissement();
        break;

      default: // si aucun case au dessus ne se realise
        Serial.print(data);
        Serial.println("->  Unknown");
        break;
    }
    myRadio.resetAvailable(); // on se remet a ecouter la frequence radio
  }

  else // Si aucun info de la part de la telecomande
  {
    average = (analogRead(NORD) + analogRead(OUEST)) * 0.5; // moyenne nord/ouest

    if (automatic_mode && average > val_min_soleil) // Si le mode auto est active,  et que la moyenne lumineuse est superieur a seuil
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

void engine_control(bool vertical, bool forward, int vitesse) // selection et control du sens de rotation des moteurs
{
  //Max vitesse = 98
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

void send_simple_PWM(const int enable_pin, const int engine_pin, int duty_cycle) // gestion de la pwm pour les moteurs
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

void stop_engine() // fonction pour arreter les moteurs
{
  digitalWrite(MotorEN_North_South, LOW);
  digitalWrite(MotorEN_West_East, LOW);
  OCR2A = 0;
  OCR2B = 0;
}

void asservissement() // realise le suivi du soleil
{
  // lecture des cellules
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
  if (inCorrectionOE)
  { //For OUEST EST
    TrOE_max = TrOE + ErrorOE / 2;
    TrOE_min = TrOE - ErrorOE / 2;
  }
  else
  {
    TrOE_max = TrOE + ErrorOE;
    TrOE_min = TrOE - ErrorOE;
  }
  if (inCorrectionNS)
  { //For NORD SUD
    TrSN_max = TrSN + ErrorSN / 2;
    TrSN_min = TrSN - ErrorSN / 2;
  }
  else
  {
    TrSN_max = TrSN + ErrorSN;
    TrSN_min = TrSN - ErrorSN;
  }

  //Watch if the system is in the error invervals
  if (differenceSN < TrSN_max && differenceSN > TrSN_min)  //If position North South OK
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

      //Light is more on OUEST on the panel
      if (differenceOE < TrOE_min && digitalRead(END_GAUCHE) == HIGH) // check fin de course gauche
      {
        forw = true;
        if (differenceOE < TrOE_min - ErrorOE)
        {
          if (Speed < 70)
          {
            Speed++;
          }
        }
        else if (differenceOE < TrOE_min && Speed > 35)
        {
          Speed--;
        }
      }

      //Light is more on EST on the panel
      if (differenceOE > TrOE_max && digitalRead(END_DROITE) == HIGH) // check fin de course droite
      {
        forw = false;
        if (differenceOE > TrOE_max + ErrorOE)
        {
          if (Speed < 70)
          {
            Speed++;
          }
        }
        else if (differenceOE > TrOE_max && Speed > 35)
        {
          Speed--;
        }
      }
      //Send instructions to engines
      verti = false;
      engine_control(verti, forw, Speed);
      return;
    }

    //NORD SUD control and regulation
    if (!flaggySN)
    {
      inCorrectionNS = true;

      //Set minimum speed to 30
      if (Speed < 70)
      {
        Speed = 70;
      }

      //Light is more on SUD on the panel
      if (differenceSN < TrSN_min && digitalRead(END_BAS) == HIGH) // check fin de course bas ici
      {
        forw = false;
        if (differenceSN < TrSN_min - ErrorSN)
        {
          if (Speed < 98)
          {
            Speed += 10;
          }
        } else if (differenceSN < TrSN_min && Speed > 70)
        {
          Speed--;
        }
      }
      //Light is more on NORD on the panel
      if (differenceSN > TrSN_max && digitalRead(END_HAUT) == HIGH) // check fin de course haut ici
      {
        forw = true;
        if (differenceSN > TrSN_max + ErrorSN)
        {
          if (Speed < 98)
          {
            Speed += 10;
          }
        }
        else if (differenceSN > TrSN_max && Speed > 70)
        {
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

bool isSensorFound() // lecture des cellules nord et est pour savoir si elles sont illuminees
{
  if ((analogRead(NORD) > 100) && (analogRead(EST) > 100)) // si nord et est son > a 100
  {
    return true;
  }
  else {
    return false;
  }
}

void fastCalibration() // cherche ou se trouve le soleil
{
  if (isSensorFound() == true) // si on est deja bien place on sort de la fonction
  {
    return;
  }
  else
  {
    while (((digitalRead(END_HAUT) == HIGH) && (digitalRead(END_DROITE) == HIGH))) // tant que FC haut et droite ne sont pas atteint et que le capteur n est pas trouve
    {
      engine_control(false, false, 98); // go droite
      while (digitalRead(END_DROITE) == HIGH) // tant qu'on est pas en butee
      {
        if (isSensorFound() == true) // si le capteur est trouve on sort
        {
          return;
        }
      }
      stop_engine();


      unsigned long debut = millis();
      unsigned long atm = millis();

      if (digitalRead(END_HAUT) == HIGH) // si je suis pas en butee haute
      {
        debut = millis();
        atm = millis();
        while ( (atm - debut < 5000) && (digitalRead(END_HAUT) == HIGH) )
        {
          engine_control(true, true, 98);
          atm = millis();
        }
      }
      stop_engine();
      if (digitalRead(END_HAUT) == HIGH) // si je suis pas en butee haute
      {
        engine_control(false, true, 98); // go gauche
        while (digitalRead(END_GAUCHE) == HIGH) // tant que je suis pas en butee gauche
        {
          if (isSensorFound() == true) // si le capteur est trouve on sort
          {
            return;
          }
        } // tant que FC gauche ouvert
        stop_engine();
      }
      if (digitalRead(END_HAUT) == HIGH) // si je suis pas en butee haute
      {
        debut = millis();
        atm = millis();
        while ( (atm - debut < 5000) && (digitalRead(END_HAUT) == HIGH) )
        {
          engine_control(true, true, 98);
          atm = millis();
        }
      }
      stop_engine();
    }
  }
  stop_engine();
}

void slowCalibration() // centre la lumière refletee par rapport au cellules photovoltaiques
{
  int valOuest, valEst, valNord, valSud, cmpt(0); // variable lecture des cellules et une variable itération de boucle

  for (cmpt = 0; cmpt <= 3; cmpt++) // on echantillone, 3 releves espaces de 0,5 secs
  {
    valOuest += analogRead(OUEST);
    valEst += analogRead(EST);
    valNord += analogRead(NORD);
    valSud += analogRead(SUD);
    delay(500);
  }

  if (((valNord - valSud) != 0) && ((valEst - valOuest) != 0)) // Si on n est pas centre
  {
    if ((valEst - valOuest) > 0) // si E>O
    {
      while (valEst < valOuest) // tant que E>O
      {
        if (digitalRead(END_DROITE) == HIGH) // si on n est pas en butee droite
        {
          engine_control(false, true, 98); // go est
          valOuest = analogRead(OUEST); // lecture ouest
          valEst = analogRead(EST); // lecture est
        }
      }
    }

    if (valOuest - valEst > 0) // si O>E
    {
      while (valOuest > valEst) // tant que O>E
      {
        if (digitalRead(END_GAUCHE) == HIGH) // si on n est pas en butee gauche
        {
          engine_control(false, false, 98); // go ouest
          valOuest = analogRead(OUEST); // lecture ouest
          valEst = analogRead(EST); // lecture est
        }
      }
    }

    if (valNord - valSud > 0) // si N>S
    {
      while (valNord > valSud) // tant que N>S
      {
        if (digitalRead(END_BAS) == HIGH) // si on n est pas en bute
        {
          engine_control(true, false, 98); // go sud
          valSud = analogRead(SUD); // lecture sud
          valNord = analogRead(NORD); // lecture nord
        }
      }
    }

    if (valSud - valNord > 0) // si S>N
    {
      while (valNord < valSud) // tant que S>N
      {
        if (digitalRead(END_HAUT) == HIGH) // si on n est pas en butee
        {
          engine_control(true, true, 98); // go haut
          valSud = analogRead(SUD); // lecture sud
          valNord = analogRead(NORD); // lecture nord
        }
      }
    }
  }
  else // si on est deja centre
  {
    return;
  }
}

//CALIBRATION
void Calibration() // save les valeurs des capteurs afin d'avoir une reference
{
  TrSN = analogRead(SUD) - analogRead(NORD); // fait la difference sud/nord, peut etre negatif
  TrOE = analogRead(OUEST) - analogRead(EST); // fait la difference ouest/est, peut etre negatif
  average = (analogRead(NORD) + analogRead(OUEST)) * 0.5; // fait la moyenne nord/ouest
  ErrorOE = average * 0.02; //Sensibility factor OUEST EST
  ErrorSN = average * 0.02; //Sensibility factor NORD SUD
  val_min_soleil = average * 0.5;

  //Enregistrement dans l'EEPROM
  EEPROM.write(0, lowByte(TrSN));
  EEPROM.write(1, highByte(TrSN));
  EEPROM.write(2, lowByte(TrOE));
  EEPROM.write(3, highByte(TrOE));
}

#include <RCSwitch.h> //Library to communicate by radio frequency 433MHz
#include <EEPROM.h>

// si la luminosité baisse sortir des fonction et attendre que la
// prevoir la telecommande en sureme user

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
//Direction
const int MotorIN_North_East = 11;
const int MotorIN_South_West = 3;
//Engine
const int MotorEN_North_South = 7;
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

  DDRC = 0xFF; //  set port C as an input port

  //Initiate PWM on Timer 2
  TCCR2A = 0b10100011;
  TCCR2B = 0b00000001;
  OCR2A = 0; //D11
  OCR2B = 0; //D3
  time_previousCommand = millis() + latence;

  // Configuration pins moteurs
  pinMode(MotorIN_North_East, OUTPUT);
  pinMode(MotorIN_South_West, OUTPUT);
  pinMode(MotorEN_North_South, OUTPUT);
  pinMode(MotorEN_West_East, OUTPUT);
  digitalWrite(MotorEN_North_South, LOW);
  digitalWrite(MotorEN_West_East, LOW);

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

  // Configuration de la radio frequence 433 MHz
  myRadio.enableReceive(0); // Receiver on interrupt at digital pin #2
}

void loop() {

  // variable locale pour gestion du temps
  unsigned long debut = millis(); // heur d'activation
  unsigned long atm = millis(); // heur actuelle (at this moment)

  if (myRadio.available()) // Si la telecommande envoie une donnee
  {
    data = myRadio.getReceivedValue() & 255; // stockage de la donne et conversion
    time_previousCommand = millis() + latence;

    switch (data) // Selon le button qu'on a presse
    {
      case 192: // bouton 1 on monte la surface
        if (digitalRead(END_HAUT) == HIGH) // check si on est en butee
        {
          debut = millis(); // heur de demarrage
          atm = millis(); // heur actuelle

          // tant qu on est pas en butee et que le temps n est pas ecoule
          while (( atm - debut < 100 ) && (digitalRead(END_HAUT) == HIGH))
          {
            engine_control(true, true, 98);
            atm = millis(); // on releve le temps
          }
          automatic_mode = false;
          break;
        }
        else // si deja en butee
        {
          stop_engine(); // arret des moteurs
          break;
        }

      case 48: // bouton 2 on descend la surface

        if (digitalRead(END_BAS) == HIGH) // check si en butee
        {
          debut = millis(); // heur de demarrage
          atm = millis(); // heur actuelle
          while (( atm - debut < 100 ) && (digitalRead(END_BAS) == HIGH)) // tant qu on est pas en butee et que le temps n est pas ecoule
          {
            engine_control(true, false, 98);
            atm = millis(); // on releve le temps
          }
          automatic_mode = false;
          break;
        }
        else // si deja en butee
        {
          stop_engine(); // arret des moteurs
          break;
        }

      case 240: // bouton 3 on tourne vers la gauche

        if (digitalRead(END_DROITE) == HIGH) // check si en butee
        {
          debut = millis(); // heur de demarrage
          atm = millis(); // heur actuelle
          while (( atm - debut < 100 ) && (digitalRead(END_DROITE) == HIGH)) // tant qu on est pas en butee et que le temps n est pas ecoule
          {
            engine_control(false, false, 98);
            atm = millis(); // on releve le temps
          }
          automatic_mode = false;
          break;
        }
        else // si deja en butee
        {
          stop_engine(); // arret des moteurs
          break;
        }

      case 12: // bouton 4 on tourne vers la droite
        if (digitalRead(END_GAUCHE) == HIGH) // check si en buté
        {
          debut = millis(); // heur de demarrage
          atm = millis(); // heur actuelle
          while (( atm - debut < 100 ) && (digitalRead(END_GAUCHE) == HIGH)) // tant qu on est pas en butee et que le temps n est pas ecoule
          {
            engine_control(false, true, 98);
            atm = millis(); // on releve le temps
          }
          automatic_mode = false;
          break;
        }
        else // si deja en butee
        {
          stop_engine(); // arret des moteurs
          break;
        }

      case 204: // bouton 5
        automatic_mode = false;
        fastCalibration(); // recherche du soleil
        slowCalibration(); // centrage des cellules
        stop_engine(); // arret des moteurs
        break;

      case 60: // bouton 6
        automatic_mode = false;
        stop_engine(); // arret des moteurs
        break;

      case 252: // bouton 7 releve de l etat des cellules et suivi soleil

        Calibration(); // releve des valeures des cellules et sauvegarde
        asservissement(); // tracking du soleil
        automatic_mode = true;
        break;


      case 3: // bouton 8 recherche du soleil, centrage du capteur, releve des cellules, suivi soleil

        fastCalibration(); // recherche du soleil
        slowCalibration(); // centrage des cellules
        Calibration(); // releve des valeures des cellules et sauvegarde
        asservissement(); // tracking du soleil
        automatic_mode = true;
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

    // Si le mode auto est active, et que la moyenne lumineuse est superieur au seuil
    if (automatic_mode && average > val_min_soleil)
    {
      time_previousCommand = millis() + latence;
      asservissement(); // tracking du soleil
    }
    // si ca fait un moment qu il n y a pas eu de correction on coupe les moteurs
    if (millis() > time_previousCommand && !alreadyStop)
    {
      alreadyStop = true;
      stop_engine(); // arret des moteurs
    }
    else // defaut
    {
      stop_engine(); // arret des moteurs
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

  //flaggy are to know if the system has not to move: true = no move necessary ; false = repositionning necessary
  //verti is to select the engine: true = NORD SUD engine ; false = OUEST EST engine
  //forw is to select the direction of the engine: true = forward ; false = backward
  bool flaggySN(false), flaggyOE(false), verti, forw;

  //These values are to set the error intervals for OUEST-EST and SUD-NORD
  int TrOE_max, TrOE_min, TrSN_max, TrSN_min;

  //Save the voltage digit difference between two opposite panels in order to save calculation process
  int differenceOE(Ouest - Est), differenceSN(Sud - Nord);

  if (checkRemote() == true )
  {
    return;
  }
  else
  {
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
        if (differenceOE < TrOE_min )
        {
          if (digitalRead(END_GAUCHE) == LOW) // test fin de course gauche
          {
            stop_engine();
            return;
          }
          else
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
        }

        //Light is more on EST on the panel
        if (differenceOE > TrOE_max ) // check fin de course droite
        {
          if (digitalRead(END_DROITE) == LOW) // test fin de course droit
          {
            stop_engine();
            return;
          }
          else
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
        if (differenceSN < TrSN_min)
        {
          if (digitalRead(END_BAS) == LOW)
          {
            stop_engine();
            return;
          }
          else
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
        }

        //Light is more on NORD on the panel
        if (differenceSN > TrSN_max) // check fin de course haut ici
        {
          if (digitalRead(END_HAUT) == LOW)
          {
            stop_engine();
            return;
          }
          else
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
        }
        verti = true;
        //Send instruction to engines
        engine_control(verti, forw, Speed);
        return;
      }
    }
  }

}

bool isSensorFound() // lecture des cellules nord et est pour savoir si elles sont illuminees
{
  if ((analogRead(NORD) > 100) && (analogRead(EST) > 100)) // si nord et est son > a 100 correspond a 0,48V // On test 180 qui doit correspondre à une tension de 0,87V
  {
    return true;
  }
  else {
    return false;
  }
}

bool checkRemote()
{
  data = 0;
  data = myRadio.getReceivedValue() & 255; // stockage de la donne et conversion
  if ( (data == 60) || (data == 192) || (data == 48) || (data == 240) || (data == 12) || (data == 252)  || (data == 3) ) // || (data == 204) ajouter quand bouton 5 pas utilisé pour test
  {
    myRadio.resetAvailable();
    return true;
  }
  else
  {
    myRadio.resetAvailable();
    return false;
  }
}

void fastCalibration() // cherche ou se trouve le soleil
{
  // variable locale pour gestion du temps
  unsigned long debut = millis(); // heur d'activation
  unsigned long atm = millis(); // heur actuelle (at this moment)

  if (isSensorFound() == true) // si on est deja bien place on sort de la fonction
  {
    return;
  }
  else
  {
    while (((digitalRead(END_HAUT) == HIGH) && (digitalRead(END_DROITE) == HIGH))) // tant que les FC haut et droite ne sont pas atteint et que le capteur n est pas trouve
    {
      engine_control(false, false, 98); // go droite ( quand face le reflecteur )
      while (digitalRead(END_DROITE) == HIGH) // tant qu on est pas en butee
      {

        if (isSensorFound() == true) // si le capteur est trouve on sort
        {
          return;
        }
        else if (checkRemote() == true )
        {
          return;
        }
      }
      stop_engine(); // arret des moteurs

      if (digitalRead(END_HAUT) == HIGH) // si je suis pas en butee haute
      {
        debut = millis(); // heur de demarrage
        atm = millis(); // heur actuelle
        while ( (atm - debut < 5000) && (digitalRead(END_HAUT) == HIGH) ) // tant qu on est pas en butee haute et qu on a pas depasse 5 sec
        {
          if (checkRemote() == true )
          {
            return;
          }
          else
          {
            engine_control(true, true, 98);
            atm = millis(); // on releve le temps
          }
        }
      }
      stop_engine(); // arret des moteurs

      if (digitalRead(END_HAUT) == HIGH) // si je suis pas en butee haute
      {
        engine_control(false, true, 98); // go gauche ( quand face au reflecteur )
        while (digitalRead(END_GAUCHE) == HIGH) // tant que je suis pas en butee gauche
        {
          if (isSensorFound() == true) // si le capteur est trouve on sort
          {
            return;
          }
          else if (checkRemote() == true )
          {
            return;
          }
        }
        stop_engine(); // arret des moteurs
      }

      if (digitalRead(END_HAUT) == HIGH) // si je suis pas en butee haute
      {
        debut = millis(); // heur de demarrage
        atm = millis(); // heur actuelle
        while ( (atm - debut < 5000) && (digitalRead(END_HAUT) == HIGH) )
        {
          if (checkRemote() == true )
          {
            return;
          }
          else
          {
            engine_control(true, true, 98);
            atm = millis(); // on releve le temps
          }
        }
      }
      stop_engine(); // arret des moteurs
    }
  }
  reverseFastCalibration(); // si capteur pas trouvé fait le chemin en sens inverse
}

void reverseFastCalibration() // cherche ou se trouve le soleil dans le sens opposé a fast calibration
{
  // variable locale pour gestion du temps
  unsigned long debut = millis(); // heur d'activation
  unsigned long atm = millis(); // heur actuelle (at this moment)

  if (isSensorFound() == true) // si on est deja bien place on sort de la fonction
  {
    return;
  }
  else
  {
    while (((digitalRead(END_BAS) == HIGH) && (digitalRead(END_GAUCHE) == HIGH))) // tant que les FC bas et gauche ne sont pas atteint et que le capteur n est pas trouve
    {
      engine_control(false, true, 98); // go gauche
      while (digitalRead(END_GAUCHE) == HIGH) // tant qu'on est pas en butee gauche
      {
        if (isSensorFound() == true) // si le capteur est trouve on sort
        {
          return;
        }
        else if (checkRemote() == true )
        {
          return;
        }
      }
      stop_engine(); // arret des moteurs

      if (digitalRead(END_BAS) == HIGH) // si je suis pas en butee basse
      {
        debut = millis(); // heur de demarrage
        atm = millis(); // heur actuelle
        while ( (atm - debut < 5000) && (digitalRead(END_BAS) == HIGH) ) // tant qu on est pas en butee basse et qu on a pas depasse 5 sec
        {
          if (checkRemote() == true )
          {
            return;
          }
          else
          {
            engine_control(true, false, 98);
            atm = millis(); // on releve le temps
          }
        }
      }
      stop_engine();
      if (digitalRead(END_BAS) == HIGH) // si je suis pas en butee haute
      {
        engine_control(false, false, 98); // go gauche
        while (digitalRead(END_DROITE) == HIGH) // tant que je suis pas en butee gauche
        {
          if (isSensorFound() == true) // si le capteur est trouve on sort
          {
            return;
          }
          else if (checkRemote() == true )
          {
            return;
          }
        }
        stop_engine();
      }
      if (digitalRead(END_BAS) == HIGH) // si je suis pas en butee basse
      {
        debut = millis(); // heur de demarrage
        atm = millis(); // heur actuelle
        while ( (atm - debut < 5000) && (digitalRead(END_BAS) == HIGH) ) // tant qu on est pas en butee basse et qu on a pas depasse 5 sec
        {
          if (checkRemote() == true )
          {
            return;
          }
          else
          {
            engine_control(true, false, 98);
            atm = millis(); // on releve le temps
          }
        }
      }
      stop_engine();
    }
  }
  fastCalibration(); // si capteur pas n est trouve, fait le chemin en sens inverse
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
  }

  if (((valNord - valSud) != 0) && ((valEst - valOuest) != 0)) // Si on n est pas centre
  {
    if ( ( (valEst - valOuest) > 0) && (digitalRead(END_GAUCHE) == HIGH) ) // si E>O et pas en butee haute
    {
      while ( (valEst > valOuest) && (digitalRead(END_GAUCHE) == HIGH) ) // tant que E>O et check FC gauche
      {
        if ( checkRemote() == true)
        {
          return;
        }
        else
        {
          engine_control(false, true, 98);
          valOuest = analogRead(OUEST); // lecture ouest
          valEst = analogRead(EST); // lecture est
        }
      }
    }

    if ( (valOuest - valEst > 0) && (digitalRead(END_DROITE) == HIGH) ) // si O>E et pas deja en butee
    {
      while ((valOuest > valEst) && (digitalRead(END_DROITE) == HIGH) ) // tant que O>E check FC droite
      {
        if ( checkRemote() == true)
        {
          return;
        }
        else
        {
          engine_control(false, false, 98);
          valOuest = analogRead(OUEST); // lecture ouest
          valEst = analogRead(EST); // lecture est
        }
      }
    }

    if ((valNord - valSud > 0) && (digitalRead(END_BAS) == HIGH)) // si N>S et pas deja en butee
    {
      while ((valNord > valSud) && (digitalRead(END_BAS) == HIGH)) // tant que N>S check FC bas
      {
        if ( checkRemote() == true)
        {
          return;
        }
        else
        {
          engine_control(true, false, 98); // go sud
          valSud = analogRead(SUD); // lecture sud
          valNord = analogRead(NORD); // lecture nord
        }
      }
    }

    if ((valSud - valNord > 0) && (digitalRead(END_HAUT) == HIGH) ) // si S>N et pas deja en butee
    {
      while ((valNord > valSud) && (digitalRead(END_HAUT) == HIGH) )// tant que S>N check FC haut
      {
        if (digitalRead(END_HAUT) == HIGH) // si on n est pas en butee
        {
          if ( checkRemote() == true)
          {
            return;
          }
          else
          {
            engine_control(true, true, 98); // go haut
            valSud = analogRead(SUD); // lecture sud
            valNord = analogRead(NORD); // lecture nord
          }
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

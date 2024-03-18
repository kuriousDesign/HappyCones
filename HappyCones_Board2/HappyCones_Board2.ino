////////////////////////////////////////////////////////////////////////////
//BOARD 2 (SLAVE)//
//This board is a slave controller, and it controls the folllowing items:
///////////////////////////////////////////////////////////////////////////

//STATION 2 PROGRAM
//ST2 Flipper Stepper
//ST2 Stopper Stepper

//Library Declarations
#include <AccelStepper.h>
#include <Executor.h>
#include <State.h>
#include <SerialBroadcast.h>
#include <Timer.h>


//I-O Mapping: Pin Definitions
const int PIN_RX_BRD1 = 0;
const int PIN_TX_BRD1 = 1;

const int PIN_ST2_STOPPER_EN = 2;
const int PIN_ST2_STOPPER_DIR = 3;
const int PIN_ST2_STOPPER_PULS = 4;

const int PIN_ST2_FLIPPER_EN = 5;
const int PIN_ST2_FLIPPER_DIR = 6;
const int PIN_ST2_FLIPPER_PULS = 7;

const int PIN_BROADCAST_RX = A1;
const int PIN_BROADCAST_TX = A0;


//Manual Mode Variables
String inString = "";
bool manualREQ;


//BRD2 Definitions
Executor BRD2("BRD2"),FLIP("FLIP");
State BRD1("BRD1"),PHASE("PHAS"),MODE("MODE"),CON("CON"),ST2("ST2");
State states1[]={BRD1,PHASE,MODE,CON,ST2};
int NUM_OF_STATES_BRD1=5;
SerialBroadcast serialBroadcast1(&Serial,38400,false);
bool connectionEstablished;
Timer timerBRD2;

//PHASE Variables
int phase = 0;
const int PHASE_STOP = 0;
const int PHASE_IDLE = 1;
const int PHASE_MODES = 2;
const int PHASE_ABORT = 3;

//MODE Variables
int mode = 0;
const int MODE_INACTIVE = 0;
const int MODE_RESET = 1;
const int MODE_AUTO = 2;
const int MODE_MANUAL = 3;

//Stepper Declarations and Parameters
const int stepsPerRev = 400; //microstep setting on stepper drivers
AccelStepper stopperStepper(AccelStepper::DRIVER, PIN_ST2_STOPPER_PULS, PIN_ST2_STOPPER_DIR);//Kart Stop Stepper
AccelStepper flipperStepper(AccelStepper::DRIVER, PIN_ST2_FLIPPER_PULS, PIN_ST2_FLIPPER_DIR);
bool steppers_areIdle;

//ST2 Declarations
Timer timerFLIP;
int jiggle_COUNT;
int distance;


///POSITION REGISTERS///
int FS_Clear_POS = -70;
int FS_Enter_POS = -33;
int FS_Flip_POS = -120;

int SS_offset=23;

int SS_Receive_POS = 24+ SS_offset;
int SS_Tip_POS = 69+ SS_offset;
int SS_SetDown_POS=36+ SS_offset;
int SS_Release_POS = 5 + SS_offset;

bool flipCart_REQ = 0; //commands from outside
bool flipCart_DONE=0;



//Diagnostics
int timeStamp=0,thisScanTime=0, maxScanTime=0;
 



void setup() {
  //BRD2
  Serial.begin(115200);
  //Serial.flush();
  connectionEstablished=0;

  //ST2
  stopperStepper.setMaxSpeed(100);
  stopperStepper.setAcceleration(100);
  flipperStepper.setMaxSpeed(100);
  flipperStepper.setAcceleration(100);
  bool steppers_areIdle = 0;
  pinMode(PIN_ST2_STOPPER_EN, OUTPUT);
  pinMode(PIN_ST2_FLIPPER_EN, OUTPUT);
  digitalWrite(PIN_ST2_FLIPPER_EN, LOW);
  digitalWrite(PIN_ST2_STOPPER_EN, LOW);
  
}//END setup()








void loop() {


  ////////////////////------------------------------------------
  //SYSTEM PROGRAM   ///////////////////////////////////////////
  ////////////////////------------------------------------------
  // Board 2 Executor Routine (BRD2)
  // Serial Broadcast Manager


  
  ////BRD2 EXECUTOR ROUTINE   /////////////////
  ////////////////////----------------

while(!connectionEstablished){
  serialBroadcast1.checkAndSend(BRD2);
  serialBroadcast1.listenAndUpdate(states1,NUM_OF_STATES_BRD1);
  BRD1=states1[0];
  //delay(5);

  if (BRD2.nextStepNumber != BRD2.isStepNumber) {
    Serial.print("BRD2 next Step Number is ");
    Serial.println(BRD2.nextStepNumber);
  }

  BRD2.call(PHASE_MODES, MODE_RESET);//necessary to call each loop
  switch(BRD2.isStepNumber){
    case 0:
      //Serial.println("BRD1 Step Number is: " + (String)BRD1.isStepNumber);
      if(BRD1.isStepNumber==5){
        BRD2.nextStepNumber=10;
      }
    break;

    case 10://Idle and Connection Established
    if(!BRD2.isFirstScanOfStep){
      connectionEstablished=1;
    }
    break;
  }//switch

  timeStamp=millis();
}//while


  ////SERIAL BROADCAST MANAGER ROUTINE   /////////////////
  ////////////////////----------------

  //receive section
  serialBroadcast1.listenAndUpdate(states1,NUM_OF_STATES_BRD1);
  BRD1=states1[0];
  PHASE=states1[1];
  MODE=states1[2];
  CON=states1[3];
  ST2=states1[4];

  //send section
  //serialBroadcast1.checkAndSend(BRD2);
  serialBroadcast1.checkAndSend(FLIP);


 //update phase and mode variables
  phase=PHASE.isStepNumber;
  mode=MODE.isStepNumber;
  //Serial.println("MODE is "+(String)mode);











  ////////////////////------------------------------------------
  //FLIP PROGRAM (FLIP)  //////////////////////////////////////////
  ////////////////////------------------------------------------
  //MONITOR
  //EXECUTOR
  //STEPPER MANAGER

  //PARENT: STGR Station Grinder
  //CHILDREN: N/A


  ////FLIP MONITOR ROUTINE   /////////////////
  ////////////////////----------------
  if (FLIP.nextStepNumber != FLIP.isStepNumber) {
    Serial.print("FLIP next Step Number is ");
    Serial.println(FLIP.nextStepNumber);
  }

    if ( flipperStepper.distanceToGo() == 0 && stopperStepper.distanceToGo() == 0) {
    steppers_areIdle = 1;
  }
  else {
    steppers_areIdle = 0;
  }

  if(ST2.isStepNumber==11){
    flipCart_REQ=1;
  }

////FLIP EXECUTOR ROUTINE   /////////////////
////////////////////----------------


  
  //Serial.println("mode="+(String)mode);
  FLIP.call(phase, mode);//necessary to call each loop
  stopperStepper.run();
  flipperStepper.run();



  ////FLIP EXECUTOR ROUTINE   /////////////////
  ////////////////////----------------

  switch (FLIP.isStepNumber) {

    ///Inactive State///
    case 0: 
      digitalWrite(PIN_ST2_FLIPPER_EN, LOW);
      digitalWrite(PIN_ST2_STOPPER_EN, LOW);
      //digitalWrite(EN_PIN_IDX, LOW);
  
      if (mode == MODE_RESET) {
        FLIP.nextStepNumber = 5;
      }
      break;

    ///Resetting: manually home the steppers///  
    case 5: 
      digitalWrite(PIN_ST2_FLIPPER_EN, LOW);
      digitalWrite(PIN_ST2_STOPPER_EN, LOW);
      
      //Serial.println("FLIP: Resetting");
      //delay(6000);
      //flipperStepper.move(10);
      //stopperStepper.move(-10);
 
      FLIP.nextStepNumber = 6;
      break;

    ///Resetting Cont'd: waiting until stepper moves are finished
    case 6: 
      if (steppers_areIdle) {
        stopperStepper.setCurrentPosition(0);
        flipperStepper.setCurrentPosition(0);
  
        FLIP.nextStepNumber = 10;
       
      }
      break;


    case 10: //Idle. Waiting for Commands
      FLIP.isIdle = 1;
      digitalWrite(PIN_ST2_FLIPPER_EN, LOW);
      digitalWrite(PIN_ST2_STOPPER_EN, LOW);
     
    

      if (flipCart_REQ == 1) {
        FLIP.nextStepNumber = 11;
        jiggle_COUNT=0;
        FLIP.isIdle = 0;
      }


      if (mode == MODE_MANUAL) {
        FLIP.nextStepNumber = 110;
        FLIP.isIdle = 0;
      }
      break;



    case 11: //stopper gets into receive pos and flipper goes up to clear pos
      digitalWrite(PIN_ST2_FLIPPER_EN, HIGH);
      digitalWrite(PIN_ST2_STOPPER_EN, HIGH);


      flipperStepper.moveTo(FS_Clear_POS);
      stopperStepper.moveTo(SS_Receive_POS);
   
      FLIP.nextStepNumber = 12;
      

      break;

      case 12: //
        if (steppers_areIdle) {
          
          if (jiggle_COUNT<3) {
          FLIP.nextStepNumber = 13;
          }
          else
          FLIP.nextStepNumber = 23;
        }
      break;

      case 13: //jiggle stopper forward
      stopperStepper.move(3);
      jiggle_COUNT=jiggle_COUNT+1;
      FLIP.nextStepNumber=14;
      break;

      case 14: //
        if (steppers_areIdle) {
          jiggle_COUNT++;
          FLIP.nextStepNumber = 11;
        }
    
      break;



    case 23: //flipCart Task: flipper lowers for cart entering
      flipperStepper.moveTo(FS_Enter_POS);
      
      FLIP.nextStepNumber = 24;

      break;

    case 24: //
      if (steppers_areIdle) {
        FLIP.nextStepNumber = 26;
      }
      break;

    case 26: //flipCart Task: stopper to tip position
      stopperStepper.moveTo(SS_Tip_POS);
      FLIP.nextStepNumber = 27;

      break;

    case 27: //
      if (steppers_areIdle) {
        FLIP.nextStepNumber = 28;
      }
      break;


   case 28: //flipCart Task: flipper flips cart
      flipperStepper.moveTo(FS_Flip_POS);
      jiggle_COUNT=0;
      FLIP.nextStepNumber = 29;
      //Serial.println("Flipping cart");

      break;

    case 29: //kill flipper if more flips needed, otherwise jump
      if (steppers_areIdle) {
         if (jiggle_COUNT<9){
         digitalWrite(PIN_ST2_FLIPPER_EN, LOW);
         delay(80);
         digitalWrite(PIN_ST2_FLIPPER_EN, HIGH);
         delay(20);
         FLIP.nextStepNumber = 30;
         }
         else
         FLIP.nextStepNumber=35;
      }
  
      break;

       case 30: //jiggle backwards
      flipperStepper.move(38);
      FLIP.nextStepNumber = 29;
      //Serial.println("jiggling cart");
      jiggle_COUNT++;

      break;

      
   case 35: //Flipper returns cart cart
      flipperStepper.move(65);
      stopperStepper.moveTo(SS_SetDown_POS);
      FLIP.nextStepNumber = 36;
      break;

    case 36: //
      if (steppers_areIdle) {
         digitalWrite(PIN_ST2_FLIPPER_EN, LOW);
         delay(100);
         flipperStepper.setCurrentPosition(FS_Enter_POS);
         digitalWrite(PIN_ST2_FLIPPER_EN, HIGH);
         delay(100);
        FLIP.nextStepNumber = 37;
         //flipCart_REQ = 0;
      }
      break;

   case 37: //flipCart Task: 
      flipperStepper.moveTo(FS_Clear_POS);
      stopperStepper.moveTo(SS_Release_POS);
      FLIP.nextStepNumber = 38;

      break;

    case 38: //
      if (steppers_areIdle) {
        delay(800);
        FLIP.nextStepNumber = 40;
         //flipCart_REQ = 0;
      }
      break;

case 40: //flipCart Task: 
      flipperStepper.moveTo(5);
      stopperStepper.moveTo(0);
      FLIP.nextStepNumber = 41;

      break;

    case 41: //
      if (steppers_areIdle) {
       digitalWrite(PIN_ST2_FLIPPER_EN, LOW);
         delay(300);
        FLIP.nextStepNumber = 100;
      }
      break;

    case 100: //
      flipCart_DONE=1;
      flipCart_REQ=0;
    if (!flipCart_REQ) {
        FLIP.nextStepNumber = 5;
      }
      break;


    case 110: //Manual Mode
      digitalWrite(PIN_ST2_FLIPPER_EN, HIGH);
      digitalWrite(PIN_ST2_STOPPER_EN, HIGH);

      while (Serial.available() > 0) {
        manualREQ = 1;
        int inChar = Serial.read();
        if (isDigit(inChar)) {
          inString += (char)inChar;
        }
        // if you get a newline, print the string, then the string's value:
        if (inChar == '\n') {
          //Serial.print("Value:");
          //Serial.println(inString.toInt());
          //Serial.print("String: ");
          //Serial.println(inString);
          distance = inString.toInt();
          // clear the string for new input:
          inString = "";
        }
      }

      if (manualREQ) {
        flipperStepper.moveTo(distance);
        manualREQ = 0;
        distance = 0;
      }


      break;

    default:
      //Serial.print("Unknown Step Number:   ");
      break;
  }

  if (FLIP.nextStepNumber != FLIP.isStepNumber) {
    //Serial.print("Next Step Number is ");
    //Serial.println(FLIP.nextStepNumber);
  }

  //Scan Check
  thisScanTime=millis()-timeStamp;
  if(thisScanTime>maxScanTime){
    maxScanTime=thisScanTime;
    Serial.println("New Max Scan: " + String(maxScanTime) + "ms");
  }
  timeStamp=millis();


}//END loop

////HELPER METHODS///////



//////////////////////////////////////////////////////////////

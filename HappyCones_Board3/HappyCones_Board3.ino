//BOARD 3 (SLAVE)//
//This board is a slave controller, and it controls the folllowing items:

//Cone Disco (CONE)


#include <AccelStepper.h>
#include <Executor.h>
#include <State.h>
#include <SerialBroadcast.h>
#include <Timer.h>

Timer timerINDEX;

String inString = "";
bool manualREQ;

//Pin Definitions
const int rx_SPARE = 0;
const int tx_SPARE = 1;
const int PIN_CONE_STOPPER_EN = 8;//HIGH to retract stopper
const int PIN_CONE_PINCHER_EN = 6;//HIGH to release pincher
const int PIN_CONE_TRAP_EN = 7;
//const int PIN_ = 5;
//const int PIN_ = 6;
//const int PIN_ = 7;
const int PIN_CONE_MOTOR_EN = 30;
const int PIN_CONE_MOTOR_IN1 = 28;
const int PIN_CONE_MOTOR_IN2 = 26;
const int PIN_BROADCAST_RX = A1;
const int PIN_BROADCAST_TX = A0;
//const int PIN_ = A2;
//const int PIN_ = A3;

//Serial Definitions
SerialBroadcast serialBroadcast(PIN_BROADCAST_TX, PIN_BROADCAST_RX);
bool connectionEstablished;

//Stepper Declarations and Parameters
const int stepsPerRev = 400; //microstep setting on stepper drivers

bool steppers_areIdle;
int jiggle_COUNT;
int distance;

///POSITION REGISTERS///

bool CON_indexing_REQ;

int phase;
int mode;
 
Executor BRD3("BRD2"),CONE("CONE");//Change "BRD2" back to "BRD3" after testing
State PHASE("PHAS"),MODE("MODE"),BRD1("BRD1"),CON("CON");

State states[]={PHASE,MODE,BRD1,CON};
int NUM_OF_STATES=4;

//PHASE Constants
const int PHASE_STOP = 0;
const int PHASE_IDLE = 1;
const int PHASE_MODES = 2;
const int PHASE_ABORT = 3;

//MODE Constants
const int MODE_INACTIVE = 0;
const int MODE_RESET = 1;
const int MODE_AUTO = 2;
const int MODE_MANUAL = 3;

//INITIALIZATION
void setup() {
  Serial.begin(9600);
  Serial.flush();
  bool steppers_areIdle = 0;
  pinMode(PIN_CONE_PINCHER_EN, OUTPUT);
  pinMode(PIN_CONE_STOPPER_EN, OUTPUT);
  pinMode(PIN_CONE_MOTOR_EN, OUTPUT);
  pinMode(PIN_CONE_MOTOR_IN1, OUTPUT);
  pinMode(PIN_CONE_MOTOR_IN2, OUTPUT);
  pinMode(PIN_CONE_STOPPER_EN, OUTPUT);
  digitalWrite(PIN_CONE_STOPPER_EN, HIGH);
  digitalWrite(PIN_CONE_TRAP_EN, HIGH);
  digitalWrite(PIN_CONE_PINCHER_EN,HIGH);

  connectionEstablished=0;

  //delete me after testing
  //CON_indexing_REQ = 1;
  
}//END SETUP

void loop() {

  
  ////BRD3 EXECUTOR ROUTINE   /////////////////
  ////////////////////----------------

while(!connectionEstablished){
  serialBroadcast.checkAndSend(BRD3);
  serialBroadcast.listenAndUpdate(states,NUM_OF_STATES);
  BRD1=states[2];
  delay(5);

  if (BRD3.nextStepNumber != BRD3.isStepNumber) {
    Serial.print("BRD3 next Step Number is ");
    Serial.println(BRD3.nextStepNumber);
  }

  BRD3.call(PHASE_MODES, MODE_RESET);//necessary to call each loop
  switch(BRD3.isStepNumber){
    case 0:
      //Serial.println("BRD1 Step Number is: " + (String)BRD1.isStepNumber);
      //Serial.println("hello");
      if(BRD1.isStepNumber==5){
        BRD3.nextStepNumber=10;
      }
    break;

    case 10://Idle and Connection Established
    if(!BRD3.isFirstScanOfStep){
      connectionEstablished=1;
    }
    break;
  }//switch
}//while


  ////SERIAL BROADCAST MANAGER /////////////////
  ////////////////////----------------

  //receive section
  serialBroadcast.listenAndUpdate(states,NUM_OF_STATES);
  PHASE=states[0];
  MODE=states[1];
  BRD1=states[2];
  CON=states[3];

  //send section
  //serialBroadcast.checkAndSend(BRD3);
  serialBroadcast.checkAndSend(CONE);

 //update phase and mode variables
  phase=PHASE.isStepNumber;
  mode=MODE.isStepNumber;
  //Serial.println("MODE is "+(String)mode);

 
  ////////////////////------------------------------------------
  //CONE PROGRAM (CONE)  //////////////////////////////////////////
  ////////////////////------------------------------------------
  //MONITOR
  //EXECUTOR
  //STEPPER MANAGER

  //PARENT: Conductor (CON)
  //CHILDREN: N/A


  ////CONE MONITOR ROUTINE   /////////////////
  ////////////////////----------------
  if (CONE.nextStepNumber != CONE.isStepNumber) {
    Serial.print("CONE next Step Number is ");
    Serial.println(CONE.nextStepNumber);
  }

  if(CON.isStepNumber==20){
    CON_indexing_REQ=1;
  }
  else CON_indexing_REQ=0;

   
////CONE EXECUTOR ROUTINE   /////////////////
////////////////////----------------
  
  CONE.call(phase, mode);//necessary to call each loop
  CONE.isIdle = 0;

  switch (CONE.isStepNumber) {
    
    ///Inactive State///
    case 0: 
      digitalWrite(PIN_CONE_MOTOR_EN, LOW);
      digitalWrite(PIN_CONE_MOTOR_IN1, LOW);
      digitalWrite(PIN_CONE_MOTOR_IN2, LOW);
      digitalWrite(PIN_CONE_STOPPER_EN, HIGH);
      digitalWrite(PIN_CONE_TRAP_EN, HIGH);
      digitalWrite(PIN_CONE_PINCHER_EN,HIGH);
  
      if (mode == MODE_RESET) {
        CONE.nextStepNumber = 5;
      }
      
    break;

    ///Resetting State///  
    case 5: 
      digitalWrite(PIN_CONE_MOTOR_EN, LOW);
      digitalWrite(PIN_CONE_MOTOR_IN1, LOW);
      digitalWrite(PIN_CONE_MOTOR_IN2, LOW);
      digitalWrite(PIN_CONE_STOPPER_EN, HIGH);
      digitalWrite(PIN_CONE_TRAP_EN, HIGH);
      digitalWrite(PIN_CONE_PINCHER_EN,HIGH);
      CONE.nextStepNumber = 10;
      
    break;

    ///Idle: Waiting for Commands
    case 10: 
      CONE.isIdle = 1;
      digitalWrite(PIN_CONE_MOTOR_EN, LOW);
      digitalWrite(PIN_CONE_MOTOR_IN1, LOW);
      digitalWrite(PIN_CONE_MOTOR_IN2, LOW);
      digitalWrite(PIN_CONE_STOPPER_EN, HIGH);
      digitalWrite(PIN_CONE_TRAP_EN, HIGH);
      digitalWrite(PIN_CONE_PINCHER_EN,HIGH);

      if (CON_indexing_REQ == 1) {
        CONE.nextStepNumber = 11;
        Serial.print("hello jake");
      }

      if (mode == MODE_MANUAL) {
        CONE.nextStepNumber = 110;
      }
    break;

    ///Indexing: retracting stopper
    case 11: 
      digitalWrite(PIN_CONE_STOPPER_EN, LOW);
      delay(50);
      CONE.nextStepNumber = 12;
    break;

    ///Indexing: starting motor & opening pincher
    case 12:
      digitalWrite(PIN_CONE_MOTOR_EN,HIGH);
      digitalWrite(PIN_CONE_MOTOR_IN1,HIGH);
      digitalWrite(PIN_CONE_PINCHER_EN,LOW);
      CONE.nextStepNumber=13;
    break;

    ///Indexing: waiting for move to finish
    case 13:
      delay(630);
      digitalWrite(PIN_CONE_STOPPER_EN, HIGH);
      CONE.nextStepNumber=15;
    break;

    ///Indexing: Done
    case 15:
      digitalWrite(PIN_CONE_MOTOR_EN,LOW);
      digitalWrite(PIN_CONE_MOTOR_IN1,LOW);
      digitalWrite(PIN_CONE_PINCHER_EN,HIGH);
      delay(500);
      CON_indexing_REQ=0;//delete me
      if (!CON_indexing_REQ) {
        CONE.nextStepNumber=20;
      }
    break;

    //Singulating: opening pincher momentarily  
    case 20: 
      digitalWrite(PIN_CONE_PINCHER_EN,LOW);
      delay(1300);
      digitalWrite(PIN_CONE_PINCHER_EN,HIGH);
      delay(100);
      CONE.nextStepNumber = 21;
    break;

    //Singulating: opening trap door momentarily  
    case 21: 
      digitalWrite(PIN_CONE_TRAP_EN,LOW);
      delay(700);
      digitalWrite(PIN_CONE_TRAP_EN,HIGH);
      delay(100);
      CONE.nextStepNumber = 10;
    break;

    case 110: //Manual Mode
  
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
     break;

     default:
      Serial.print("Unknown Step Number:   ");
     break;
  }

}//END LOOP

////HELPER METHODS///////
//END HELPER METHODS

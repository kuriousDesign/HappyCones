/////////////////////////////////////////////////////////////////////////////////
//////BOARD 1 (MASTER)//
//////This board is the master controller, and it controls the folllowing items:
////////////////////////////////////////////////////////////////////////////////


//SYSTEM PROGRAM//
//Phase Manager
//Mode Manager
//Broadcast Manager

//CONDUCTOR PROGRAM//
//Conductor Executor (CON)

//INDEXER PROGRAM//
//Indexer Executor  (IDX)
//IDX Stepper Motor

//Cone Disco (CONE)

//STATION 1 PROGRAM//
//Station 1 Executor (ST1), Unload Joint/Load Cone

//STATION 2 PROGRAM
//Station 2 Executor (ST2), HempKart Dump

//STATION 4 PROGRAM
//Station 4 Executor (ST4), Chopstick Tamp (1st Stage)

//STATION 5 PROGRAM
//Station 5 Executor (ST5), GolfTee Tamp (2nd Stage)

//STATION GRIND PROGRAM
//Station Grind Executor (STGR), Grinder/Scale Kart
//Grinder Executor (GRND)

//Library Declarations
#include <AccelStepper.h>
#include <Executor.h>
#include <Scale.h>
#include <SerialBroadcast.h>
#include <State.h>
#include <Timer.h>


//I-O Mapping: Pin Definitions
const int PIN_ST4_STIR = 53; //not actually wired

const int PIN_RX_BRD2 = 0;
const int PIN_TX_BRD2 = 1;

const int PIN_IDX_STEPPER_IN1 = 2;
const int PIN_IDX_STEPPER_IN2 = 3;
const int PIN_IDX_STEPPER_IN3 = 4;
const int PIN_IDX_STEPPER_IN4 = 5;

const int PIN_CONE_PINCHER_EN = 6;//HIGH to release pincher
const int PIN_CONE_TRAP_EN = 52;
const int PIN_CONE_STOPPER_EN = 8;//HIGH to retract stopper

const int PIN_VALVE_7 = 7; //Lift Cylinder UP
const int PIN_VALVE_6 = 8; //Spare
const int PIN_VALVE_5 = 9; //ST1 Pinch Cylinder OPEN
const int PIN_VALVE_4 = 10;
const int PIN_VALVE_ST4 = 11;//ST4 Tamp Cyl DOWN
const int PIN_VALVE_ST5 = 12;// ST5 Tamp Cyl DOWN
const int PIN_VALVE_1= 13;

const int PIN_SCALE_TX = 14;
const int PIN_SCALE_RX = 15;

const int PIN_INDEXONLY_PB = 23;//
const int PIN_CYCLEONCE_PB = 25;//

const int PIN_CONE_MOTOR_IN2 = 26;
const int PIN_CONE_MOTOR_IN1 = 28;
const int PIN_CONE_MOTOR_EN = 30;

const int PIN_GRND_MOTOR_EN = 32;
const int PIN_GRND_MOTOR_IN1 = 34;
const int PIN_GRND_MOTOR_IN2 = 36;

const int PIN_BROADCAST_RX = A1;
const int PIN_BROADCAST_TX = A0;


//Manual Mode Variables
bool manualREQ;
String inString = "";


//BRD1 Variable Declarations
Executor PHASE("PHAS"),MODE("MODE"),BRD1("BRD1"),CON("CON"),IDX("IDX"),CONE("CONE"),
         ST2("ST2"),ST4("ST4"),ST5("ST5"),STGR("STGR"),GRND("GRND"); //Executors (on this board)      
State BRD2("BRD2"),FLIP("FLIP"); //States (incoming from other boards)
State states2[]={BRD2,FLIP};
int NUM_OF_STATES_BRD2=2;
SerialBroadcast serialBroadcast2(PIN_BROADCAST_TX,PIN_BROADCAST_RX);
bool connectionEstablished;
Timer timerBRD1;
Serial scaleSerial; 

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

//Stepper Declarations
bool steppers_areIdle = 0;
const int STEPSPERREV = 400; //microstep setting on stepper drivers

AccelStepper idxStepper(AccelStepper::HALF4WIRE, PIN_IDX_STEPPER_IN1,PIN_IDX_STEPPER_IN2,
                                                  PIN_IDX_STEPPER_IN3, PIN_IDX_STEPPER_IN4);

//ACM Variable Declarations
bool ACMxCON_IndexOnly_REQ, ACMxCON_CycleOnce_REQ;
char hmiChar;
String hmiString;

//CON Variable Declarations
Timer timerCON;
bool allStations_areIdle,allNestwork_isDone;
int cycleCount;
bool CON_Cycle_REQ;//incoming commands
bool CON_Prework_REQ,CON_Index_REQ=0,CON_Nestwork_REQ;//outgoing Command Bits



//IDX Variable Declarations
Timer timerIDX;
bool IDX_Index_DONE,IDX_Index_REQ;

//CONE Declarations
Timer timerCONE;
bool CONE_indexing_REQ;

//ST2 Declarations
Timer timerST2;

//ST4 Declarations
int ST4tampCount;
Timer timerST4, timerST4motor;
bool ST4_SpinMotor_TURNON, ST4_SpinMotor_isON,ST4_SpinMotor_TURNOFF, 
     ST4_SpinMotor_isOFF,  switchON=0;

//ST5 Declarations
int ST5tampCount;
Timer timerST5;


//STGR Variable Declarations
Timer timerSTGR,timerGRND;
float scaleWeight;
int i_fill=0;
bool _done;
bool STGR_Fill_REQ;//Incoming Requests
Scale GRND_scale(PIN_SCALE_TX,PIN_SCALE_RX);




void setup() {
  
  //BRD1
  Serial.begin(9600);
  Serial.flush();
  Serial1.begin(9600);
  connectionEstablished=0;

  //IDX
  idxStepper.setMaxSpeed(350);
  idxStepper.setAcceleration(200);
  idxStepper.disableOutputs();

  //PHASE & MODE
  phase = PHASE_STOP;
  mode = MODE_INACTIVE;

  //PIN MODES
  
  pinMode(PIN_VALVE_1,OUTPUT);
  pinMode(PIN_VALVE_4,OUTPUT);
  pinMode(PIN_VALVE_5,OUTPUT);
  pinMode(PIN_VALVE_6,OUTPUT);
  pinMode(PIN_VALVE_7,OUTPUT);
  
  pinMode(PIN_ST4_STIR,OUTPUT);
  pinMode(PIN_VALVE_ST4,OUTPUT);

  pinMode(PIN_VALVE_ST5,OUTPUT);
  
  pinMode(PIN_IDX_STEPPER_IN1,OUTPUT);
  pinMode(PIN_IDX_STEPPER_IN2,OUTPUT);
  pinMode(PIN_IDX_STEPPER_IN3,OUTPUT);
  pinMode(PIN_IDX_STEPPER_IN4,OUTPUT);
  pinMode(PIN_INDEXONLY_PB,INPUT);
  pinMode(PIN_CYCLEONCE_PB,INPUT);

  pinMode(PIN_CONE_PINCHER_EN, OUTPUT);
  pinMode(PIN_CONE_STOPPER_EN, OUTPUT);
  pinMode(PIN_CONE_MOTOR_EN, OUTPUT);
  pinMode(PIN_CONE_MOTOR_IN1, OUTPUT);
  pinMode(PIN_CONE_MOTOR_IN2, OUTPUT);
  pinMode(PIN_CONE_STOPPER_EN, OUTPUT);

  pinMode(PIN_GRND_MOTOR_EN, OUTPUT);
  pinMode(PIN_GRND_MOTOR_IN1, OUTPUT);
  pinMode(PIN_GRND_MOTOR_IN2, OUTPUT);
  
  digitalWrite(PIN_VALVE_ST4,HIGH);
  digitalWrite(PIN_VALVE_ST5,HIGH);
  digitalWrite(PIN_VALVE_1,HIGH);
  digitalWrite(PIN_VALVE_4,HIGH);
  digitalWrite(PIN_VALVE_5,HIGH);
  digitalWrite(PIN_VALVE_6,HIGH);
  digitalWrite(PIN_VALVE_7,HIGH);
  digitalWrite(PIN_ST4_STIR,HIGH);

  digitalWrite(PIN_CONE_STOPPER_EN, HIGH);
  digitalWrite(PIN_CONE_TRAP_EN, HIGH);
  digitalWrite(PIN_CONE_PINCHER_EN,HIGH);
 
}//END setup()







void loop() {

  ////////////////////------------------------------------------
  //SYSTEM PROGRAM   ///////////////////////////////////////////
  ////////////////////------------------------------------------
  // Board 1 Executor Routine (BRD1)
  // Serial Broadcast Manager
  // Phase Manager Routine (PHASE)
  // Mode Manager Routine (MODE)
  // Serial Broadcast Manager Routine (BRD1)



  ////BRD1 EXECUTOR ROUTINE   /////////////////
  ////////////////////----------------

while(!connectionEstablished){
  serialBroadcast2.checkAndSend(BRD1);
  serialBroadcast2.listenAndUpdate(states2,NUM_OF_STATES_BRD2);
  BRD2=states2[0];
  delay(3);
  
  if (BRD1.nextStepNumber != BRD1.isStepNumber) {
    Serial.print("BRD1 step: ");
    Serial.println(BRD1.nextStepNumber); 
  }
  
  BRD1.call(PHASE_MODES, MODE_RESET);//necessary to call each loop
  if (BRD1.isFirstScanOfStep){
    timerBRD1.reset();
  }
  timerBRD1.call();

  switch(BRD1.isStepNumber){
    case 0:
      BRD1.nextStepNumber=5;
    break;

    case 5:
    if(BRD1.isFirstScanOfStep){
      timerBRD1.set(2000);
      serialBroadcast2.setSendNextFlag();
    }

    if(BRD2.isStepNumber==10){
      BRD1.nextStepNumber=10;
    }

    break;

    case 6:
    if(BRD1.isFirstScanOfStep){
      timerBRD1.set(2000);
      //Serial.println("BRD2 state is: "+(String)BRD2.isStepNumber);
    }
    if(timerBRD1.isDone){
      if(BRD2.isStepNumber==10){
        BRD1.nextStepNumber=10;
      }
      else{
        BRD1.nextStepNumber=5;
      }
    }
    break;

    case 10://Idle and Connection Established
    if(!BRD1.isFirstScanOfStep){
      connectionEstablished=1;
    }
    break;
  }//switch
  
}//while



  ////SERIAL BROADCAST MANAGER ROUTINE   /////////////////
  ////////////////////----------------

  //receive section
   serialBroadcast2.listenAndUpdate(states2,NUM_OF_STATES_BRD2);
   BRD2=states2[0];
   FLIP=states2[1];
  
  //send section
   //serialBroadcast2.checkAndSend(BRD1);
   serialBroadcast2.checkAndSend(PHASE);
   serialBroadcast2.checkAndSend(MODE);
   serialBroadcast2.checkAndSend(CON);
   serialBroadcast2.checkAndSend(ST2);
  



  ////PHASE MANAGER ROUTINE   /////////////////
  ////////////////////----------------
  
  PHASE.call(PHASE_MODES,MODE_AUTO);
  phase=PHASE.isStepNumber;
  switch (phase){
  
    case PHASE_STOP:
      if(BRD1.isIdle){
        PHASE.nextStepNumber=PHASE_IDLE;
        Serial.println("PHASE is IDLE");
      }
    break;
    
    case PHASE_IDLE:
      if(BRD2.isIdle){
        PHASE.nextStepNumber=PHASE_MODES;
        Serial.println("PHASE is MODES");
      }
    break;
    
    case PHASE_MODES:
    break;
    
    default:
      //PHASE.nextStepNumber=PHASE_STOP;
    break;
    
  }//switch

 
  ////MODE MANAGER ROUTINE   /////////////////
  ////////////////////----------------
  
  MODE.call(PHASE_MODES,MODE_AUTO);
  mode=MODE.isStepNumber;
  switch (mode) {
    case MODE_INACTIVE:
      if (phase==PHASE_MODES){
        MODE.nextStepNumber=MODE_RESET;
        Serial.println("MODE is RESET");
      }
    break;
    
    case MODE_RESET:
      if (CON.isIdle){
        MODE.nextStepNumber=MODE_AUTO;
        Serial.println("MODE is AUTO");
      }
    break;

    case MODE_AUTO:
      if (CON.isIdle&manualREQ){
        MODE.nextStepNumber=MODE_MANUAL;
        Serial.println("MODE is MANUAL");
      }
    break;

    case MODE_MANUAL:
      if (!manualREQ){
        MODE.nextStepNumber=MODE_RESET;
      }
    break;

    default:
      //MODE.nextStepNumber=MODE_INACTIVE;
    break;
    
  }//switch



  ////////////////////------------------------------------------
  //ACTIVTY MANAGER (ACM)  //////////////////////////////////////////
  ////////////////////------------------------------------------
  //MONITOR
  //EXECUTOR

  ACMxCON_IndexOnly_REQ=0;
  ACMxCON_CycleOnce_REQ=0;
      
  if(mode==MODE_AUTO & CON.isIdle){
    if(digitalRead(PIN_CYCLEONCE_PB)){
        ACMxCON_CycleOnce_REQ=1;
    }
      
    if(digitalRead(PIN_INDEXONLY_PB)){  
        ACMxCON_IndexOnly_REQ=1;
    }
  }
  

//  if(mode==MODE_AUTO & CON.isIdle){
//
//    if (Serial1.available()>0){
//      hmiChar=Serial1.read();
//      hmiString+=hmiChar;
//      if (hmiString=="once"){
//        Serial.println("Cycle Once Command Detected");
//        ACMxCON_CycleOnce_REQ=1;
//        inString="";
//      }
//      
//      if (inString=="index"){
//        Serial.println("Index Only Command Detected");
//        ACMxCON_IndexOnly_REQ=1;
//        hmiString="";
//      }
//    }
//    else{
//      hmiString="";
//      ACMxCON_IndexOnly_REQ=0;
//      ACMxCON_CycleOnce_REQ=0;
//    }
//  }





  ////////////////////------------------------------------------
  //CONDUCTOR PROGRAM (CON)  //////////////////////////////////////////
  ////////////////////------------------------------------------
  //MONITOR
  //COMMANDS
  //EXECUTOR

  ////CON MONITOR ROUTINE   /////////////////
  ////////////////////----------------

    if (CON.nextStepNumber != CON.isStepNumber) {
    Serial.print("CON step: ");
    Serial.println(CON.nextStepNumber);
    }

    if(IDX.isIdle & CONE.isIdle & ST4.isIdle & ST5.isIdle){
    allStations_areIdle=1;
    }
    else{
    allStations_areIdle=0;
    }

    if(CONE.isDone & ST4.isDone & ST5.isDone){
    allNestwork_isDone=1;
    }
    else{
    allNestwork_isDone=0;
    }

    CON.call(phase, mode);//necessary to call each loop
      if(CON.isFirstScanOfStep){
        timerCON.reset();
      }
    timerCON.call();


  ////CON COMMAND ROUTINE   /////////////////
  ////////////////////----------------  

  //Incoming Commands
    if(ACMxCON_CycleOnce_REQ){
      CON_Cycle_REQ=1;
    }
    else{CON_Cycle_REQ=0;}
    
  //Outgoing Commands: these get set inside the executor routine
    CON_Prework_REQ=0;
    CON_Index_REQ=0;
    CON_Nestwork_REQ=0;

  //Status Bits
  CON.isIdle = 0;
  CON.isDone = 0;
  
  ////CON EXECUTOR ROUTINE   /////////////////
  ////////////////////----------------
  
  switch (CON.isStepNumber) {

    ///Inactive State///
    case 0: 
      if (mode == MODE_RESET) {
        CON.nextStepNumber = 5;
      }
      break;

    ///Resetting///  
    case 5: 
      if (allStations_areIdle) {
        CON.nextStepNumber = 10;
      }
      break;

    ///Idle: Waiting for Commands///
    case 10: 
      CON.isIdle = 1;
      if (CON_Cycle_REQ) {
        CON.nextStepNumber = 20;
      }

      if (mode == MODE_MANUAL) {
        CON.nextStepNumber = 110;
      }
      break;
      
    ///Indexing and Prework///
    case 20:
    CON_Prework_REQ=1;
    CON_Index_REQ=1;
    
    if (IDX.isDone){
      CON.nextStepNumber=50;
    }
    break;

    ///Nestwork///
    case 50:
    CON_Nestwork_REQ=1;
    if (allNestwork_isDone){
      CON.nextStepNumber=100;
    }
    break;

    ///Done///
    case 100:
    if(CON.isFirstScanOfStep){
      cycleCount++;
    }
    CON.isDone = 1;
    if (allStations_areIdle & !CON_Cycle_REQ){
      
      CON.nextStepNumber=10;
    }
    break;


  }//switchcase


  ////////////////////------------------------------------------
  //INDEXER PROGRAM (IDX)  //////////////////////////////////////////
  ////////////////////------------------------------------------
  //MONITOR
  //EXECUTOR
  //STEPPER MANAGER

  ////IDX MONITOR ROUTINE   /////////////////
  ////////////////////----------------

  if (IDX.nextStepNumber != IDX.isStepNumber) {
    Serial.print("IDX step: ");
    Serial.println(IDX.nextStepNumber);
  }

  //steppers_areIdle logic
  if (idxStepper.distanceToGo() == 0) {
    steppers_areIdle = 1;
  }
  else {
    steppers_areIdle = 0;
  }

  IDX.call(phase, mode);//necessary to call each loop
    if (IDX.isFirstScanOfStep){
      timerIDX.reset();
    }
  timerIDX.call();


  ////IDX COMMAND ROUTINE   /////////////////
  ////////////////////----------------

  //Incoming Commands
    if(ACMxCON_IndexOnly_REQ||CON_Index_REQ){
      IDX_Index_REQ=1;
    }
    else{IDX_Index_REQ=0;}
  
  
  ////IDX EXECUTOR ROUTINE   /////////////////
  ////////////////////----------------

  switch (IDX.isStepNumber) {

    ///Inactive State///
    case 0: 
      idxStepper.disableOutputs();
      
      if (mode == MODE_RESET) {
        IDX.nextStepNumber = 5;
      }
      break;

    ///Resetting: disable the stepper and zero it///  
    case 5: 
      idxStepper.disableOutputs();
      digitalWrite(PIN_VALVE_7,HIGH);
      
      if (steppers_areIdle) {
        idxStepper.setCurrentPosition(0);
        IDX.nextStepNumber = 10;
      }
      break;

    ///Idle: Waiting for Commands///
    case 10: 
      idxStepper.disableOutputs();
    
      if (IDX_Index_REQ) {
        IDX.nextStepNumber = 20;
      }

      if (mode == MODE_MANUAL) {
        IDX.nextStepNumber = 110;
      }
      break;



      case 20: //Lift the work plate
        if(IDX.isFirstScanOfStep){
          timerIDX.set(500);
        }
        idxStepper.enableOutputs();
        digitalWrite(PIN_VALVE_7,LOW);
      
        if (timerIDX.isDone) {
          IDX.nextStepNumber = 30;
        }
      break;

      case 30: //Index Stepper Task: enable stepper
       idxStepper.enableOutputs();
       
       if (steppers_areIdle) {
        IDX.nextStepNumber = 31;
       }
      break;

      case 31: //Index Stepper Task: move command
        idxStepper.move(STEPSPERREV);
        if (steppers_areIdle==0) {
          IDX.nextStepNumber = 32;
        }
      break;

      case 32: //Index Stepper Task: wait for move to finish
        if (steppers_areIdle){
        IDX.nextStepNumber=33;
        }
      break;

      case 33: //Index Stepper Task: disable stepper
       idxStepper.disableOutputs();
       
       if (1==1) {
        IDX.nextStepNumber = 40;
       }
      break;

      case 40: //Lower work plate
        if(IDX.isFirstScanOfStep){
          timerIDX.set(600);
        }
        digitalWrite(PIN_VALVE_7,HIGH);
        if (timerIDX.isDone) {
          IDX.nextStepNumber = 100;
        }
      break;

      case 100: //Done State
           if (!IDX_Index_REQ) {
          IDX.nextStepNumber = 10;
        }
      break;

    case 110: //Manual Mode
    break;

    default:
      Serial.print("IDX unknown Step: " + (String)IDX.isStepNumber);
    break;
  }

  ////ACTUATOR MANAGERS ROUTINE   /////////////////
  ////////////////////----------------

  //Indexer Stepper Motor
  idxStepper.run();
















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

  if(CON_Prework_REQ){
    CONE_indexing_REQ=1;
  }
  else CONE_indexing_REQ=0;

   
////CONE EXECUTOR ROUTINE   /////////////////
////////////////////----------------
  
  CONE.call(phase, mode);//necessary to call each loop
  CONE.isIdle = 0;
  CONE.isDone = 0;
  if (CONE.isFirstScanOfStep){
    timerCONE.reset();
  }
  timerCONE.call();

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

      if (CONE_indexing_REQ == 1) {
        CONE.nextStepNumber = 11;
        //Serial.print("hello jake");
      }

      if (CON_Nestwork_REQ) {
        CONE.nextStepNumber = 20;
      }

      if (mode == MODE_MANUAL) {
        CONE.nextStepNumber = 110;
      }
    break;

    ///Indexing: retracting stopper
    case 11: 

    if(CONE.isFirstScanOfStep){
      timerCONE.set(50);
    }
    digitalWrite(PIN_CONE_STOPPER_EN, LOW);
    if(timerCONE.isDone){
      CONE.nextStepNumber=12;
    }
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
      if(CONE.isFirstScanOfStep){
        timerCONE.set(630);
      }
      digitalWrite(PIN_CONE_STOPPER_EN, HIGH);
      if(timerCONE.isDone){
        CONE.nextStepNumber=15;
      }
    break;

    ///Indexing: Done
    case 15:
      if(CONE.isFirstScanOfStep){
        timerCONE.set(500);
      }
      digitalWrite(PIN_CONE_MOTOR_EN,LOW);
      digitalWrite(PIN_CONE_MOTOR_IN1,LOW);
      digitalWrite(PIN_CONE_PINCHER_EN,HIGH);
     
      if (!CONE_indexing_REQ & timerCONE.isDone) {
        CONE.nextStepNumber=10;
      }
    break;

    //Singulating: opening pincher momentarily  
    case 20: 
      if(CONE.isFirstScanOfStep){
        timerCONE.set(1300);
      }
      digitalWrite(PIN_CONE_PINCHER_EN,LOW);

      if (timerCONE.isDone) {
        CONE.nextStepNumber=21;
      }

    //Singulating: opening pincher momentarily  
    case 21:  
      if(CONE.isFirstScanOfStep){
        timerCONE.set(100);
      }
      digitalWrite(PIN_CONE_PINCHER_EN,HIGH);
      
      if (timerCONE.isDone) {
        CONE.nextStepNumber=23;
      }
    break;

    //Singulating: opening trap door momentarily  
    case 23: 
      if(CONE.isFirstScanOfStep){
        timerCONE.set(700);
      }
      digitalWrite(PIN_CONE_TRAP_EN,LOW);

      if (timerCONE.isDone) {
        CONE.nextStepNumber=100;
      }
    break;

    //Singulating: closing trap door  
    case 25: 
      if(CONE.isFirstScanOfStep){
        timerCONE.set(100);
      }
      digitalWrite(PIN_CONE_TRAP_EN,HIGH);
      if (timerCONE.isDone) {
        CONE.nextStepNumber=100;
      }
    break;

    //Singulating: Done  
    case 100: 
      CONE.isDone=1;
      if (!CON_Nestwork_REQ) {
        CONE.nextStepNumber = 10;
      }
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
          //distance = inString.toInt();
          // clear the string for new input:
          inString = "";
        }
      }
     break;

     default:
      Serial.print("Unknown Step Number:   ");
     break;
  }




  ////////////////////------------------------------------------
  //STATION 2 PROGRAM (ST2)  //////////////////////////////////////////
  ////////////////////------------------------------------------
  //MAIN
  //MONITORS
  //EXECUTOR
  //STEPPER MANAGER

  ////ST2 MONITOR ROUTINE   /////////////////
  ////////////////////----------------

//  if (ST2.nextStepNumber != ST2.isStepNumber) {
//    Serial.print("ST2 next Step Number is ");
//    Serial.println(ST2.nextStepNumber);
//  }


  
  ST2.call(phase, mode);//necessary to call each loop
    if (ST2.isFirstScanOfStep){
      timerST2.reset();
    }
  timerST2.call();
  
  
//  ////ST2 EXECUTOR ROUTINE   /////////////////
//  ////////////////////----------------
//  
  
//  switch (ST2.isStepNumber) {
//
//    ///Inactive State///
//    case 0: 
//      ST2.isInactive = 1;
//      
//      if (mode == MODE_RESET) {
//        IDX.nextStepNumber = 5;
//        IDX.isInactive = 0;
//      }
//      break;
//
//    ///Resetting: manually home the steppers///  
//    case 5: 
//      digitalWrite(EN_PIN_FLIPPER, LOW);
//      digitalWrite(EN_PIN_STOPPER, LOW);
//      //digitalWrite(EN_PIN_IDX, LOW);
//      //flipperStepper.move(10);
//      //stopperStepper.move(-10);
//      //idxStepper.move(10);
//      IDX.nextStepNumber = 6;
//      break;
//
//    ///Resetting Cont'd: waiting until stepper moves are finished
//    case 6: 
//      if (steppers_areIdle) {
//        stopperStepper.setCurrentPosition(0);
//        flipperStepper.setCurrentPosition(0);
//        idxStepper.setCurrentPosition(0);
//        IDX.nextStepNumber = 10;
//        mode = 2;
//       
//      }
//      break;
//
//
//    case 10: //Idle. Waiting for Commands
//      IDX.isIdle = 1;
//      digitalWrite(EN_PIN_FLIPPER, LOW);
//      digitalWrite(EN_PIN_STOPPER, LOW);
//      idxStepper.disableOutputs();
//    
//
//      if (flipCart_REQ == 1) {
//        IDX.nextStepNumber = 11;
//        jiggle_COUNT=0;
//        IDX.isIdle = 0;
//      }
//
//      mode = 2;//puts machine into auto mode
//
//      if (mode == 110) {
//        IDX.nextStepNumber = 110;
//        IDX.isIdle = 0;
//      }
//      break;
//
//
//
//    case 11: //stopper gets into receive pos and flipper goes up to clear pos
//      digitalWrite(EN_PIN_FLIPPER, HIGH);
//      digitalWrite(EN_PIN_STOPPER, HIGH);
//      //digitalWrite(EN_PIN_IDX, HIGH);
//      idxStepper.enableOutputs();
//      flipperStepper.moveTo(FS_Clear_POS);
//      stopperStepper.moveTo(SS_Receive_POS);
//      //idxStepper.move(STEPSPERREV);
//      IDX.nextStepNumber = 12;
//      
//
//      break;
//
//      case 12: //
//        if (steppers_areIdle) {
//          
//          if (jiggle_COUNT<3) {
//          IDX.nextStepNumber = 13;
//          }
//          else
//          IDX.nextStepNumber = 23;
//        }
//      break;
//
//      case 13: //jiggle stopper forward
//      stopperStepper.move(3);
//      jiggle_COUNT=jiggle_COUNT+1;
//      IDX.nextStepNumber=14;
//      break;
//
//      case 14: //
//        if (steppers_areIdle) {
//          jiggle_COUNT++;
//          IDX.nextStepNumber = 11;
//        }
//    
//      break;
//
//
//
//    case 23: //flipCart Task: flipper lowers for cart entering
//      flipperStepper.moveTo(FS_Enter_POS);
//       idxStepper.disableOutputs();
//      IDX.nextStepNumber = 24;
//
//      break;
//
//    case 24: //
//      if (steppers_areIdle) {
//        IDX.nextStepNumber = 26;
//      }
//      break;
//
//    case 26: //flipCart Task: stopper to tip position
//      stopperStepper.moveTo(SS_Tip_POS);
//      IDX.nextStepNumber = 27;
//
//      break;
//
//    case 27: //
//      if (steppers_areIdle) {
//        IDX.nextStepNumber = 28;
//      }
//      break;
//
//
//   case 28: //flipCart Task: flipper flips cart
//      flipperStepper.moveTo(FS_Flip_POS);
//      jiggle_COUNT=0;
//      IDX.nextStepNumber = 29;
//      Serial.println("Flipping cart");
//
//      break;
//
//    case 29: //kill flipper if more flips needed, otherwise jump
//      if (steppers_areIdle) {
//         if (jiggle_COUNT<9){
//         digitalWrite(EN_PIN_FLIPPER, LOW);
//         delay(80);
//         digitalWrite(EN_PIN_FLIPPER, HIGH);
//         delay(20);
//         IDX.nextStepNumber = 30;
//         }
//         else
//         IDX.nextStepNumber=35;
//      }
//  
//      break;
//
//       case 30: //jiggle backwards
//      flipperStepper.move(38);
//      IDX.nextStepNumber = 29;
//      Serial.println("jiggling cart");
//      jiggle_COUNT++;
//
//      break;
//
//      
//   case 35: //Flipper returns cart cart
//      flipperStepper.move(65);
//      stopperStepper.moveTo(SS_SetDown_POS);
//      IDX.nextStepNumber = 36;
//      break;
//
//    case 36: //
//      if (steppers_areIdle) {
//         digitalWrite(EN_PIN_FLIPPER, LOW);
//         delay(100);
//         flipperStepper.setCurrentPosition(FS_Enter_POS);
//         digitalWrite(EN_PIN_FLIPPER, HIGH);
//         delay(100);
//        IDX.nextStepNumber = 37;
//         //flipCart_REQ = 0;
//      }
//      break;
//
//   case 37: //flipCart Task: 
//      flipperStepper.moveTo(FS_Clear_POS);
//      stopperStepper.moveTo(SS_Release_POS);
//      IDX.nextStepNumber = 38;
//
//      break;
//
//    case 38: //
//      if (steppers_areIdle) {
//        delay(800);
//        IDX.nextStepNumber = 40;
//         //flipCart_REQ = 0;
//      }
//      break;
//
//case 40: //flipCart Task: 
//      flipperStepper.moveTo(5);
//      stopperStepper.moveTo(0);
//      IDX.nextStepNumber = 41;
//
//      break;
//
//    case 41: //
//      if (steppers_areIdle) {
//       digitalWrite(EN_PIN_FLIPPER, LOW);
//         delay(300);
//        IDX.nextStepNumber = 100;
//      }
//      break;
//
//    case 100: //
//      flipCart_DONE=1;
//      flipCart_REQ=0;
//    if (!flipCart_REQ) {
//        IDX.nextStepNumber = 5;
//      }
//      break;
//
//
//    case 110: //Manual Mode
//      digitalWrite(EN_PIN_FLIPPER, HIGH);
//      digitalWrite(EN_PIN_STOPPER, HIGH);
//
//      while (Serial.available() > 0) {
//        manualREQ = 1;
//        int inChar = Serial.read();
//        if (isDigit(inChar)) {
//          inString += (char)inChar;
//        }
//        // if you get a newline, print the string, then the string's value:
//        if (inChar == '\n') {
//          Serial.print("Value:");
//          Serial.println(inString.toInt());
//          Serial.print("String: ");
//          Serial.println(inString);
//          distance = inString.toInt();
//          // clear the string for new input:
//          inString = "";
//        }
//      }
//
//      if (manualREQ) {
//        flipperStepper.moveTo(distance);
//        manualREQ = 0;
//        distance = 0;
//      }
//
//
//      break;
//
//    default:
//      Serial.print("Unknown Step Number:   ");
//  }



















 ////////////////////------------------------------------------
  //STATION 4 PROGRAM (ST4)  //////////////////////////////////////////
  ////////////////////------------------------------------------
  //MONITOR
  //EXECUTOR
  //STIR MOTOR MANAGER

  ////ST4 MONITOR ROUTINE   /////////////////
  ////////////////////----------------

  if (ST4.nextStepNumber != ST4.isStepNumber) {
    Serial.print("ST4 step: ");
    Serial.println(ST4.nextStepNumber);
  }
  
  ST4.call(phase, mode);//necessary to call each loop
    if (ST4.isFirstScanOfStep){
    timerST4.reset();
  }
  timerST4.call();
  
  ////ST4 EXECUTOR ROUTINE   /////////////////
  ////////////////////----------------
  
  switch (ST4.isStepNumber) {

    ///Inactive State///
    case 0: 
      digitalWrite(PIN_VALVE_ST4,HIGH);
      ST4_SpinMotor_TURNOFF=1;
      
      if (mode == MODE_RESET) {
        ST4.nextStepNumber = 5;
        ST4.isInactive = 0;
      }
      break;

    ///Resetting: disable the stepper and zero it///  
    case 5: 
      digitalWrite(PIN_VALVE_ST4,HIGH);
      ST4_SpinMotor_TURNOFF=1;
      
      if (FLIP.isIdle) {
        ST4.nextStepNumber = 10;
      }
      break;

    ///Idle: Waiting for Commands///
    case 10: 
    
      if (CON_Nestwork_REQ) {
        ST4.nextStepNumber = 20;
        ST4.isIdle = 0;
      }

      if (mode == MODE_MANUAL) {
        ST4.nextStepNumber = 110;
        ST4.isIdle = 0;
      }
      break;

    //Tamp Task: initialization step
    case 20:
    if(ST4.isFirstScanOfStep){
      ST4tampCount=0;
    }
    ST4.nextStepNumber=21;
    break;

    //Tamp Task: actuate tamper down step
    case 21:
    if(ST4.isFirstScanOfStep){
      timerST4.set(300);
    }
    
    digitalWrite(PIN_VALVE_ST4,LOW);
    if (ST4tampCount>0){
    ST4_SpinMotor_TURNON=1;
    }

 

    if(timerST4.isDone){
    ST4.nextStepNumber=22;
    }
    break; 

    //Tamp Task: actuate tamper up step
    case 22:
    if(ST4.isFirstScanOfStep){
      timerST4.set(100);
    }
    digitalWrite(PIN_VALVE_ST4,HIGH);

    if(timerST4.isDone){
    ST4.nextStepNumber=23;
    }
    break; 

    //Tamp Task: check tampCount
    case 23:
    if(ST4.isFirstScanOfStep){
      ST4tampCount++;
    }

    if(ST4tampCount<11){
    ST4.nextStepNumber=21;
    }
    else{
    ST4_SpinMotor_TURNOFF=1;
    ST4.nextStepNumber=100;
    }
    break; 

    ///Done State///
    case 100:
    if(ST4.isFirstScanOfStep){
    }

    if(!CON_Nestwork_REQ){
    ST4.nextStepNumber=10;
    }
    break; 

    
  }//switchcase


  ////ST4 STIR MOTOR MANAGER ROUTINE   /////////////////
  ////////////////////----------------
 
 if(ST4_SpinMotor_TURNON){
  ST4_SpinMotor_isON=1;
   ST4_SpinMotor_isOFF=0;
   ST4_SpinMotor_TURNON=0;
 }
 
 if (ST4_SpinMotor_TURNOFF||!PHASE_MODES){
  ST4_SpinMotor_isOFF=1;
  ST4_SpinMotor_isON=0;
  switchON=0;
  digitalWrite(PIN_ST4_STIR,HIGH);
     ST4_SpinMotor_TURNOFF=0;
 }

 if(ST4_SpinMotor_isON){
    if(!timerST4motor.isEnabled&!switchON){
    timerST4motor.set(100);
    digitalWrite(PIN_ST4_STIR,LOW);
    //Serial.println("switched on ");
    switchON=1;
    }

    timerST4motor.call();
    if(timerST4motor.isDone){
      timerST4motor.reset();
    }

    if(!timerST4motor.isEnabled&switchON){
    timerST4motor.set(200);
    digitalWrite(PIN_ST4_STIR,HIGH);
     Serial.print("switched off ");
    switchON=0;
    }
    
    timerST4motor.call();
        if(timerST4motor.isDone){
      timerST4motor.reset();
    }

    
}





 ////////////////////------------------------------------------
  //STATION 5 PROGRAM (ST5)  //////////////////////////////////////////
  ////////////////////------------------------------------------
  //MONITORS
  //EXECUTOR
  //STEPPER MANAGER

  ////ST5 MONITOR ROUTINE   /////////////////
  ////////////////////----------------

  if (ST5.nextStepNumber != ST5.isStepNumber) {
    Serial.print("ST5 step: ");
    Serial.println(ST5.nextStepNumber);
  }
  
  ST5.call(phase, mode);//necessary to call each loop
    if (ST5.isFirstScanOfStep){
    timerST5.reset();
  }
  timerST5.call();
  ////ST5 EXECUTOR ROUTINE   /////////////////
  ////////////////////----------------
  


  switch (ST5.isStepNumber) {

    ///Inactive State///
    case 0: 
      digitalWrite(PIN_VALVE_ST5,HIGH);
      
      if (mode == MODE_RESET) {
        ST5.nextStepNumber = 5;
        ST5.isInactive = 0;
      }
      break;

    ///Resetting: disable the stepper and zero it///  
    case 5: 
      digitalWrite(PIN_VALVE_ST5,HIGH);
      
      if (1==1) {
        ST5.nextStepNumber = 10;
      }
      break;

    ///Idle: Waiting for Commands///
    case 10: 
   
      if (CON_Nestwork_REQ) {
        ST5.nextStepNumber = 20;
        ST5.isIdle = 0;
      }

      if (mode == MODE_MANUAL) {
        ST5.nextStepNumber = 110;
        ST5.isIdle = 0;
      }
      break;

    //Tamp Task: initialization step
    case 20:
    if(ST5.isFirstScanOfStep){
      ST5tampCount=0;
    }
    ST5.nextStepNumber=21;
    break;

    //Tamp Task: actuate tamper down step
    case 21:
    if(ST5.isFirstScanOfStep){
      timerST5.set(650);
    }
    
    digitalWrite(PIN_VALVE_ST5,LOW);

    if(timerST5.isDone){
    ST5.nextStepNumber=22;
    }
    break; 

    //Tamp Task: actuate tamper up step
    case 22:
    if(ST5.isFirstScanOfStep){
      timerST5.set(200);
    }
    digitalWrite(PIN_VALVE_ST5,HIGH);

    if(timerST5.isDone){
    ST5.nextStepNumber=23;
    }
    break; 

    //Tamp Task: check tampCount
    case 23:
    if(ST5.isFirstScanOfStep){
      ST5tampCount++;
    }

    if(ST5tampCount<5){
    ST5.nextStepNumber=21;
    }
    else{
    ST5.nextStepNumber=100;
    }
    break; 

    ///Done State///
    case 100:
    if(ST5.isFirstScanOfStep){
    }

    if(!CON_Nestwork_REQ){
    ST5.nextStepNumber=10;
    }
    break; 

    
  }//switchcase


  ////////////////////------------------------------------------
  //STATION GRINDER PROGRAM (STGR)  //////////////////////////////////////////
  ////////////////////------------------------------------------
  //MONITORS
  //EXECUTOR
  //GRND EXECUTOR on Board 2


  ////STGR MONITOR ROUTINE   /////////////////
  ////////////////////----------------

  if (STGR.nextStepNumber != STGR.isStepNumber) {
    Serial.print("STGR Step: ");
    Serial.println(STGR.nextStepNumber);
  }
  
  STGR.call(phase, mode);//necessary to call each loop
  if (STGR.isFirstScanOfStep){
    timerSTGR.reset();
  }
  timerSTGR.call(); 

  STGR.isIdle = 0;
  STGR.isDone = 0;
  
////STGR EXECUTOR ROUTINE   /////////////////
  ////////////////////----------------
  
  switch (STGR.isStepNumber) {

    ///Inactive State///
    case 0: 
      if (mode == MODE_RESET) {
        STGR.nextStepNumber = 5;
      }
      break;

    ///Resetting///  
    case 5: 
      if (GRND.isStepNumber==10) {
        STGR.nextStepNumber = 10;
      }
      break;

    ///Idle: Waiting for Commands///
    case 10: 
      STGR.isIdle=1;
      if (CON_Nestwork_REQ) {
        STGR.nextStepNumber = 20;

      }

      if (mode == MODE_MANUAL) {
        STGR.nextStepNumber = 110;
  
      }
      break;
      
    ///Request Fill Task from GRND on Board 2///
    case 20:
    //STGR_Fill_REQ=1; this is actually set based on the active-read step number of STGR in board 2

    if (GRND.isStepNumber==29){//if GRND is at step 29, the Fill task is DONE
      STGR.nextStepNumber=50;
    }

    if (GRND.isStepNumber==911){//if GRND is at step 911, the Fill task errored-out prior to completion
      STGR.nextStepNumber=911;
    }
    break;

    ///Nestwork///
    case 50:
      STGR.nextStepNumber=100;
    break;

    ///Done///
    case 100:
      STGR.isDone=1;
    break;


  }//switchcase




  ////////////////////------------------------------------------
  //GRINDER-SCALE PROGRAM (GRND)  //////////////////////////////////////////
  ////////////////////------------------------------------------
  //MONITORS
  //EXECUTOR

  ////GRND MONITOR ROUTINE   /////////////////
  ////////////////////----------------

  if (GRND.nextStepNumber != GRND.isStepNumber) {
    Serial.print("GRND next Step Number is ");
    Serial.println(GRND.nextStepNumber);
  }

   if(GRND.isStepNumber!=20){
   scaleWeight=GRND_scale.read();
   //Serial.println(scaleWeight);
   }

   //Fill Request Mapping
   if(STGR.isStepNumber==20){
   STGR_Fill_REQ=1;
   }
   else{
    STGR_Fill_REQ=0;
   }


  GRND.call(phase, mode);//necessary to call each loop
  if (GRND.isFirstScanOfStep){
    timerGRND.reset();
  }
  timerGRND.call();
  
  ////GRND EXECUTOR ROUTINE   /////////////////
  ////////////////////----------------
  
  switch (GRND.isStepNumber) {

    ///Inactive State///
    case 0: 
      
      if (mode == MODE_RESET) {
        GRND.nextStepNumber = 5;
      }
      break;

    ///Resetting: disable the stepper and zero it///  
    case 5: 
      grindMotor(0,0);
      if (1==1) {
        GRND.nextStepNumber = 10;
      }
      break;

    ///Idle: Waiting for Commands///
    case 10: 
   
      if (STGR_Fill_REQ) {
        GRND.nextStepNumber = 20;
        GRND.isIdle = 0;
      }

      if (mode == MODE_MANUAL) {
        GRND.nextStepNumber = 110;
        GRND.isIdle = 0;
      }
      break;

    //Grind to Weight Task: Taring step
    case 20:
      if(GRND.isFirstScanOfStep){
        timerGRND.set(500);
        GRND_scale.tare();
        scaleWeight=-999.9;
      }
      //Serial.println(scaleWeight);
      if(scaleWeight==0.0 & timerGRND.isDone){
        GRND.nextStepNumber=25;
      }
    break;

    //Grind to Weight Task: grinding step
    case 25:
      if(GRND.isFirstScanOfStep){
        timerGRND.set(15000);
      }
      _done=grindLaw(scaleWeight);
      if(_done){
        GRND.nextStepNumber=29;
      }
      else if(timerGRND.isDone){
        GRND.nextStepNumber=911;
        Serial.println("Grind-Fill Task Timed Out: hopper is probably empty or grinder is jammed");
      }
    break;

  
    //Grind to Weight Task: Done step
    case 29:
      if(GRND.isFirstScanOfStep){
      //Serial.println("GRND: Done");
      }
  
      if(!STGR_Fill_REQ){
      GRND.nextStepNumber=10;
      }
    break; 

    ///Error State///
    case 911:
    break;
 
  }//switchcase GRND



  

}//loop






/////////////////////////////////////////////////////////////////////////////////////
////HELPER METHODS///////
////////////////////////////////////////////////////////////////////////////////////

// grindLaw()
// grindMotor()


//////////////////////////////////////////////////////////////
bool grindLaw(float _scaleWeight){
  if (_scaleWeight<0.6){
    grindMotor(1,155);
    //Serial.println("grinding");
    return 0;
  }
  else if(_scaleWeight<1.0){
     grindMotor(1,85);
  }
  else {
    grindMotor(0,0);
    return 1;
  }
}//end grindLaw


//////////////////////////////////////////////////////////
void grindMotor(bool onState, int motorSpeed) {

if (onState){
 digitalWrite(PIN_GRND_MOTOR_IN1, HIGH);
 digitalWrite(PIN_GRND_MOTOR_IN2, LOW);
 // set speed to 200 out of possible range 0~255
 analogWrite(PIN_GRND_MOTOR_EN, motorSpeed);
 //Serial.println("motor on");
}

else{
 analogWrite(PIN_GRND_MOTOR_EN, 0);
 digitalWrite(PIN_GRND_MOTOR_IN1, LOW);
 digitalWrite(PIN_GRND_MOTOR_IN2, LOW);
 }
}//end grindMotor

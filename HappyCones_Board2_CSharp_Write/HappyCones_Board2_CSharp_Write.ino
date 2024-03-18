////////////////////////////////////////////////////////////////////////////
//BOARD 2 (SLAVE)//
//This board is a slave controller, and it controls the folllowing items:
///////////////////////////////////////////////////////////////////////////

//STATION 2 PROGRAM
//ST2 Flipper Stepper
//ST2 Stopper Stepper

//Library Declarations
#include <Timer.h>

//Fast ADC
#define FASTADC 1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif












bool connectionEstablished,flag;
Timer timerBRD2;


//Diagnostics
int thisScanTime=0, maxScanTime=0,ctr=0,t=0;
long timeStart, timeStop;
double avgScanTime=0;
//char inChar;
//String inString;
bool isMatch;
String inString="";
double sensorDeg,refRad,maxAngle, zeroOffset, refDeg, errDeg,errDeg_prev, errDeg_dt, Kp, Kd, Ki,Kol, ctrlVal,outVal;  // variable to store the value coming from the sensor
double thetaDot, thetaTorque;
int sensorPin = A0;    // select the input pin for the potentiometer
int valvePin = 22;      // select the pin for the LED
int outPin = 11;


void setup() 
{

 
#if FASTADC
 // set prescale to 16
 sbi(ADCSRA,ADPS2) ;
 cbi(ADCSRA,ADPS1) ;
 cbi(ADCSRA,ADPS0) ;
#endif
  
  Serial.begin(115200);
  connectionEstablished=false;
}//END setup()



void loop() 
{
  Serial.flush();
  delay(3000);
  ctr=0;
  flag=false;
  timeStart=micros();
  while(!StringCheck("READY")||true)
  {
    //delayMicroseconds(40);
    //Serial.print("HELLO\n");
    while(!Serial.availableForWrite()||flag||ctr>=1000)
    {
      flag=true;
    }
    if(micros()>(timeStart+100))
    {
      //delay(10);
      timeStart=micros();
      sensorDeg = analogRead(sensorPin)*360.0/1023.0;;
      Serial.write(byte(sensorDeg));
      ctr++;
    }
   
  }
  Serial.flush();
  Serial.print("START\n");
  delay(1300);
  ctr=0;
  timeStart=micros();
  sensorDeg = 255;
  while(ctr<1000)
  {
      
      //sensorDeg = (analogRead(sensorPin)-zeroOffset)*360.0/1023.0;
      //sensorDeg = 255;
      while(!Serial.availableForWrite())
      {
      sensorDeg = 133;
      }
  
      
      sensorDeg=Serial.write(byte(sensorDeg));
      //delayMicroseconds(100);
      ctr++;
      
//      Serial.print(String(int(thetaDot))+'\n' );
//      ctr++;
//      if(CalcThetaDot(sensorDeg,1000)&& false)
//      {
//       Serial.print(String(int(thetaDot))+'\n' );
//        ctr++;
//      }

      //analogRead(A0);
      //Serial.print(String(ctr) + '\n' );
      //delayMicroseconds(1);

//    ctr++;
//    isMatch=false; 
//    Serial.print(String(ctr) + '\n' );
//    while(!isMatch)
//    {
//      isMatch = StringCheck(String(ctr));
//    }
//    thisScanTime=micros()-timeStart;
//    avgScanTime= avgScanTime + thisScanTime;
//    if(thisScanTime>maxScanTime)
//    {
//      maxScanTime=thisScanTime;
//      t=ctr;
//    }   
  }//end while

Serial.print("END\n");
  
  avgScanTime = int(float(avgScanTime)/1000.0);
  Serial.print('T'+String(maxScanTime) + "time" + String(t)+'\n');
  Serial.print('T'+String(avgScanTime) + '\n');
  while(1)
  {
  }

}// end loop()


bool StringCheck(String checkString)
{

  char inChar;
  bool isEqual=false;
  while(Serial.available()) 
  {
    inChar = Serial.read();
    //Serial.println(inChar);
    if(inChar == '\n' | inChar == '\r')
    {
      //Serial.print(inString);
      if(inString==checkString)
      {
        isEqual = true;

      }     
      inString="";      
      break;
    }
    else
    {
      inString = inString + inChar;
    }
  }//while
  
    if(isEqual)
    {
      return true;
    }
    else
    {
      return false;
    }
}

//CalcThetaDot returns true if there is a new value ready, and is only true for that scan.
bool CalcThetaDot(float sensorDeg, double timeStepMicros)
{
  const int LENGTH_OF_ARRAY = 25;
  static double thetaDotVals[LENGTH_OF_ARRAY]={0,0,0,0,0};
  static double thetaDotTimes[LENGTH_OF_ARRAY]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
  static double lastRecordTime=0;
  static float tempDiffVal,tempDiffDiv;
  static float tempDiffTime;
  double tempSum;
  //static float thetaDot;
  
  if(micros()-lastRecordTime >= timeStepMicros)
  {
    lastRecordTime=micros();
    for(int i = 0; i<LENGTH_OF_ARRAY; i++)
    { 
   
      
      if(i<LENGTH_OF_ARRAY-1)
      {
        thetaDotVals[i]=thetaDotVals[i+1];
        thetaDotTimes[i]=thetaDotTimes[i+1];
      }
      else if (i==LENGTH_OF_ARRAY-1)
      {
        thetaDotVals[i]=sensorDeg;
        thetaDotTimes[i]= micros();
        //Serial.println(String(thetaDotTimes[i])+"time(i)");
      }

      if(i==1)
      {
        
      }
    }

    tempSum=0;
    for(int i = 1; i<LENGTH_OF_ARRAY; i++)
    {  
      
        tempDiffVal=(thetaDotVals[i]-thetaDotVals[i-1]);
        tempDiffTime=(thetaDotTimes[i]-thetaDotTimes[i-1]);
        tempDiffDiv=tempDiffVal/tempDiffTime;
        tempSum=tempDiffDiv + tempSum;
        //Serial.println(tempSum);
    }
    
    
    thetaDot=tempSum*1000000.0/double(LENGTH_OF_ARRAY-1);
    //Serial.print("thetaDot: " + String(thetaDot)+'\n' );
    return true;
  }
  else
  {

    thetaDot=0.0;
    tempDiffDiv=0.0;
    tempSum=0.0;
    return false;
  }
}





  

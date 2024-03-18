////////////////////////////////////////////////////////////////////////////
//BOARD 2 (SLAVE)//
//This board is a slave controller, and it controls the folllowing items:
///////////////////////////////////////////////////////////////////////////

//STATION 2 PROGRAM
//ST2 Flipper Stepper
//ST2 Stopper Stepper

//Library Declarations
#include <Timer.h>
#include <SerialMarino.h>

bool connectionEstablished;
Timer timerBRD2, timerHELLO;

//Diagnostics
int timeStart, timeStop, thisScanTime = 0, maxScanTime = 0, ctr = 0, t = 0;
double avgScanTime = 0;
//char inChar;
//String inString;
bool isMatch;
String inString = "";
double sensorDeg, refRad, maxAngle, zeroOffset, refDeg, errDeg, errDeg_prev, errDeg_dt, Kp, Kd, Ki, Kol, ctrlVal, outVal; // variable to store the value coming from the sensor
double thetaDot, thetaTorque;
int sensorPin = A0;    // select the input pin for the potentiometer
int valvePin = 22;      // select the pin for the LED
int outPin = 11;
bool isHelloReceived, isNewThrowReceived;
byte throwCommandData[100];

void setup()
{
  Serial.begin(115200);
  connectionEstablished = false;
  timerHELLO.set(0);
  Serial.flush();
  
  //delay(5000);
}//END setup()



void loop()
{
  //STEP 1: Sending Hello and Waiting for one Back from
  while (!isHelloReceived)
  {
    if (ListenAndCheck("HELLO"))
    {
      isHelloReceived = true;
      break;
    }
    timerHELLO.call();
    if (timerHELLO.isDone)
    {
      timerHELLO.set(2000);
      Serial.print("HELLO\n");
    }
  }


  //STEP 2: Receving New Throw Data
  while (!ListenAndCheck("NEWTHROW")) {}
  inString="";
  while (!CollectNewThrowData()) {}


  delay(1300);
  Serial.flush();
  Serial.print("START\n");
  delay(1300);
  ctr = 0;
  timeStart = micros();
  while (ctr < 100)
  {
    Serial.write(throwCommandData[ctr]);
    ctr++;
    //sensorDeg = (analogRead(sensorPin)-zeroOffset)*360.0/1023.0;
    sensorDeg = micros();
    //Serial.print(String(int(thetaDot))+'\n' );
    //Serial.write(byte(ctr));


    //analogRead(A0);
    //Serial.print(String(ctr) + '\n' );
    //delayMicroseconds(1);

    //    ctr++;
    //    isMatch=false;
    //    Serial.print(String(ctr) + '\n' );
    //    while(!isMatch)
    //    {
    //      isMatch = ListenAndCheck(String(ctr));
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
  delay(3000);

  avgScanTime = int(float(avgScanTime) / 1000.0);
  Serial.print('T' + String(maxScanTime) + "time" + String(t) + '\n');
  Serial.print('T' + String(avgScanTime) + '\n');
  while (1)
  {
  }

}// end loop()


bool CollectNewThrowData()
{
  static byte byteArray[100];
  char inChar;
  bool done=false;
  static int j = 0;
  while ( Serial.available() )
  { 
    inChar = Serial.read();
    byte inByte = byte(inChar);
    if ((inChar == '\n' & inString.endsWith("END")))
    {
      done = true;
      inString = "";
      for (int i = 0; i < 100; i++)
      {
        throwCommandData[i] = byteArray[i];
      }
      break;
    }
    else
    {
      if (inString.length() >= 3)
      {
         inString = inString.substring(1, 3) + String(inChar);
      }
      else
      {
         inString = inString + inChar;
      }
      if(j<100)
      {
        byteArray[j] = inByte;
      }
      j++;
     }
     
    }
  
  if (done)
  {
    return true;
  }
  else
  {
    return false;
  }

}

bool ListenAndCheck(String checkString)
{

  char inChar;
  bool isEqual = false;
  while (Serial.available())
  {
    inChar = Serial.read();
    //Serial.println(inChar);
    if (inChar == '\n')
    {

      //Serial.print(inString);
      if (inString.endsWith(checkString))
      {
        isEqual = true;
      }
      else
      {
        //Serial.print(inString + "\n");
      }
      inString = "";
      break;
    }
    else
    {
      inString = inString + inChar;
    }
  }//while

  if (isEqual)
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
  static double thetaDotVals[LENGTH_OF_ARRAY] = {0, 0, 0, 0, 0};
  static double thetaDotTimes[LENGTH_OF_ARRAY] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  static double lastRecordTime = 0;
  static float tempDiffVal, tempDiffDiv;
  static float tempDiffTime;
  double tempSum;
  //static float thetaDot;

  if (micros() - lastRecordTime >= timeStepMicros)
  {
    lastRecordTime = micros();
    for (int i = 0; i < LENGTH_OF_ARRAY; i++)
    {


      if (i < LENGTH_OF_ARRAY - 1)
      {
        thetaDotVals[i] = thetaDotVals[i + 1];
        thetaDotTimes[i] = thetaDotTimes[i + 1];
      }
      else if (i == LENGTH_OF_ARRAY - 1)
      {
        thetaDotVals[i] = sensorDeg;
        thetaDotTimes[i] = micros();
        //Serial.println(String(thetaDotTimes[i])+"time(i)");
      }

      if (i == 1)
      {

      }
    }

    tempSum = 0;
    for (int i = 1; i < LENGTH_OF_ARRAY; i++)
    {

      tempDiffVal = (thetaDotVals[i] - thetaDotVals[i - 1]);
      tempDiffTime = (thetaDotTimes[i] - thetaDotTimes[i - 1]);
      tempDiffDiv = tempDiffVal / tempDiffTime;
      tempSum = tempDiffDiv + tempSum;
      //Serial.println(tempSum);
    }


    thetaDot = tempSum * 1000000.0 / double(LENGTH_OF_ARRAY - 1);
    //Serial.print("thetaDot: " + String(thetaDot)+'\n' );
    return true;
  }
  else
  {

    thetaDot = 0.0;
    tempDiffDiv = 0.0;
    tempSum = 0.0;
    return false;
  }
}

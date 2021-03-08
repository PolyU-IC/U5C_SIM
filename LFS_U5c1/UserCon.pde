/* UserCon - user controller code, read sensors, possibly decode features, update speed and turn rate

   A challenge course controller will most likely need a much better line detector than
   presented here. Consider scrapping this one and writing one that includes ability to detect
   multiple line intersections. Also, changes in course polarity (white lines on black...)
   could be detected OR you could go with edge detection approach...
    
   The PD controller implemented here is good enough for the advanced course, but
   you will probably want to think about how to handle all the special patterns present in challenge
   course. (If you are trying to solve the DPRG Challenge Course)
   
   
*/

float ePrev;      // global previous error 

float Kp;         // PD Controller parameters  see UserParEdit 
float Kd;

float maxSpeed;

String controllerMessageString = ""; // message that gets displayed on Panel 1
                                     // see code below that populates this string 
String s1,s2,s3;

int lstep=-1;
int step = 0;            //Robot step
int right_trigger = 18;   //Threshold sensor value for turning
int left_trigger = -18;   //Threshold sensor value for turning
int counter = 0;          //Software delay during turn
int counter2 = 0;         //Software delay after turn
Boolean lock = false;     //Enable / disable PID
Boolean dir = false;      //False:right | True:left
Boolean finish = false;   //Finish 
int Act_stop=0;
int Act_fwd=1;
int Act_skip=2;
int Act_turn_lf=3;
int Act_turn_rg=4;
int Action= Act_fwd;

int k_SKIP=14;
int k_TURN=22;
int cycle=0;

int LM_Line=0;
int LM_LeftJoint=1;
int LM_RightJoint=2;
int LM_TJoint=3;

int findLM()                  //find landmark
{
  int lm = LM_Line; //default Line
    if ((sensorL.read()<=0.2) & (sensorR.read()<=0.2))
    {
       lm= LM_TJoint; //T-joint 
    }
    else if ((sensorL.read()<=0.4) & (sensorR.read()>=1))
    {
       lm = LM_LeftJoint;  //LF-Branch 
      
    } 
    else if ((sensorL.read()>=1) & (sensorR.read()<=0.4))
    {
      lm = LM_RightJoint;  //RG-Branch
    }
   return lm;

}

void userControllerUpdate ()    
{
    
  // A new, optional feature of LFS (lib 1.4.4), is to provide means of saving robot 
  // state variables when robot is running in non-contest mode and a marker is created.
  // See LFS_RS tab header for more information.
  
  
  //RobotState cs = currentRobotState;  // get shorthand reference to current state
                                        // note this is not a "new" instance. 
  //cs.sampleCounter = 1;               // example access to current RobotState with short name.  
                                        // This is also done in userReset.
                                        
                                        
  
  // your robot controller code here - this method called every time step
  // crude attempt to control velocity as function of current centroid error 
  Kp=2; //was 10
  Kd=30;
  float[] sensor =  sensor1.readArray();   // readArray() returns reference to sensor array of floats 
  
  float e = calcSignedCentroidSingleDarkSpan(sensor) ;   //error in pixels of line intersection with sensor
  
  
  // Added example code of accessing line sensor color table and modifying it for display ---------
  // This code is for display only, no impact on robot run
    
  int[] colorTable = sensor1.getColorArray();  // get reference to sensor color table
  int n = sensor1.getSensorCellCount();
  for (int i=0; i<n; i++) 
  {
    color c =  color (0,0,50); // dark blue
    if ((i > n/2-e-3) && (i < n/2-e+3)) c = color (255,0,0); // red 
    colorTable[i] = c;
  }
  
  // update sensor colors on spot sensors even though not used in this demo
  
  if (sensorL.read() > 0.5) sensorL.setColor (color(0,0,100));
  else sensorL.setColor(color(255,0,0));
 
  if (sensorM.read() > 0.5) sensorM.setColor (color(0,0,100));
  else sensorM.setColor(color(255,0,0));
 
  if (sensorR.read() > 0.5) sensorR.setColor (color(0,0,100));
  else sensorR.setColor(color(255,0,0));  

  // -------------------------------------------------------------------------------------------

  s1=String.format("sensed line position error %3.1f",e);
  s2=String.format("sensorL %3.1f",sensorL.read());
  s3=String.format("sensorR %3.1f",sensorR.read());
  
  controllerMessageString = s1+"\n"+s2+"\n"+s3+"\n";
   
  // note you can dynamically change sensor positions if you wish
  // in this example sensor radius is varied - just to show it can be done
  // now, commented out here
  // sensor1.setRotation(10);  // now theta
  //sensor1.setArcRadius(1.0+ (frameCount%100)/100.0);
  //sensor1.setRotation (90*mouseX/width);
  
  // generally your robot will up updating TargetTurnRate and possibly TargetSpeed
  
  
  //Status display
  cycle=cycle+1;
 
  if (step != lstep)
  {
    print("Sensor: "+controllerMessageString+"\r\n");
    print("step:" +step+ "\r\n");
    print("cycle:" +cycle+ "\r\n");
    print("Action:" +Action+ "\r\n");
    print("Counter:"+counter+"\r\n");
    lstep=step;
    print("\r\n");
  }
  if (step == 0)    //0=LM_Line
  {
    Action = Act_fwd;
     if (findLM()==LM_RightJoint) // LM_RightJoint
     {
        step=1;          
     }
  } else if (step == 1) {      
   Action = Act_turn_rg;
   if (counter > k_TURN+5)
      step=2;
  
  } else if (step == 2) {      
    Action = Act_fwd;
    if (findLM()==LM_LeftJoint) // detect the left joint
    {
      step=3;
    }
  } else if (step == 3) {      
   Action = Act_turn_lf;
//   Action = Act_stop;
   if (counter > k_TURN+5)
      step=4;
  } else if (step == 4) {
    Action = Act_fwd;
    if (findLM()==LM_LeftJoint) // detect the left joint
    {
      step=5;
    }
  } else if (step == 5) {
    Action = Act_skip;
    if (counter > k_SKIP)
    {
      step=6; 
    }
  } else if (step == 6) {
    Action = Act_fwd;
   if (findLM()==LM_LeftJoint) // detect the left joint
    {
      step=7;
    }
  } else if (step == 7) {
    Action = Act_skip;
    if (counter > k_SKIP)
    {
      step=8; 
    }
  } else if (step == 8) {
    Action = Act_fwd;
    if (findLM()==LM_TJoint) // detect the T-joint
    {
   Action = Act_stop;
    }
 }      
        

/**
* Motion Controller
*/
 if (Action==Act_stop)
 {
    //STOP
    counter=0;
    lfs.setTargetTurnRate(0);
    lfs.setTargetSpeed (0);
 } else if ((Action==Act_turn_lf) || (Action==Act_turn_rg)) {
    //Turn right
    counter++;
    if(counter <= k_TURN)
    {
      if (Action==Act_turn_rg)
      {
        lfs.setTargetSpeed (2);
        lfs.setTargetTurnRate(180);
      }  else {
        lfs.setTargetSpeed (2);
        lfs.setTargetTurnRate(-180);
      } 
        
     } else {
        lfs.setTargetSpeed (2);
        lfs.setTargetTurnRate(0);
      }
 
  } else if (Action==Act_fwd) {
    counter=0;
    //PID Controller
    lfs.setTargetTurnRate(-e * Kp + (e - ePrev) * Kd);   // turn rate in degrees per second
    ePrev = e;  
    lfs.setTargetSpeed (maxSpeed);    //0.8
  } else if (Action==Act_skip) {
    counter++;
  }

}


//very simple line detector, given sensor array from line (or 1/2 circle) sensor
//you will probably want to enhance the line detector if you are creating a challenge course robot

float calcSignedCentroidSingleDarkSpan(float[] sensor)  // 0=line centered under sensor array  + to right , - to left
{

  int n = sensor.length; // total number of sensor samples
  
  // calculate centroid of single black line 
  
  float sum = 0;
  int count = 0;
  float centroid = 0;
  for (int i=0; i<n; i++)
  if (sensor[i]<0.5) { sum+= i;  count++; }
  if (count>0)
  centroid = (0.5*n)-(sum/count);  // make centroid signed value  0 at center
  
  return centroid; 
}

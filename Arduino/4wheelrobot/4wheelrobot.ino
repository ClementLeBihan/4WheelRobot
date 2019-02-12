#include <PID_v1.h>
#include <Wire.h>
#include <Encoder.h>

// Slave Adress of the I2C Communcation
#define SLAVE_ADDRESS 0x12

// Two motors encoder
Encoder rightEnc(2,4), leftEnc(18,19);

// Motors Connexions
int PMW_A = 3;
int PMW_B = 11;
int DIR_A = 12;
int DIR_B = 13;

// PWM Command on each motors
double PWM_A_cmd, PWM_B_cmd;

// Timers
unsigned long lastTimerUpdate;
unsigned long lastMeasUpdate;
unsigned long lastCmdUpdate;

// Speed Measurement and encoder Position
long lastSpeedMesurement = 0;
long lastPosRight = 0, lastPosLeft = 0;


double speedLeft = 0, speedRight = 0;
double speedLeftTarget, speedRightTarget;
float vright, vleft; // FROM -1 to 1

// PID Parameters
double consKp=1, consKi=5, consKd=0.025;
PID rightPID(&speedRight, &PWM_A_cmd, &speedRightTarget,consKp,consKi,consKd, DIRECT);
PID leftPID(&speedLeft, &PWM_B_cmd, &speedLeftTarget,consKp,consKi,consKd, DIRECT);

bool stop = false;

void setup() {
 // Setup Serial communication
 Serial.begin(9600);
 
 lastTimerUpdate = millis();
  
 // Initialize i2c as slave
 Wire.begin(SLAVE_ADDRESS);
 
 // Define callbacks for i2c communication
 Wire.onReceive(receiveData);
 
 pinMode(PMW_A, OUTPUT);
 pinMode(PMW_B, OUTPUT);

 pinMode(DIR_A, OUTPUT);
 pinMode(DIR_B, OUTPUT);

  // Setup the PID
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(10);
  rightPID.SetOutputLimits(-255,255);
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(10);
  leftPID.SetOutputLimits(-255,255);

}

void loop() {
  // Every 10 ms (100Hz)
  if(millis() - lastMeasUpdate>10)
  {    
    // Compute Right Encoder Speed
    double dPos = rightEnc.read()-lastPosRight;
    double dT = (micros()-lastSpeedMesurement)*10e-6; //dT in seconds
    speedRight = 0.4*dPos/dT;
    rightPID.Compute();

    // Compute Left Encoder Speed
    dPos = leftEnc.read()-lastPosLeft;
    dT = (micros()-lastSpeedMesurement)*10e-6; //dT in seconds
    speedLeft = 0.4*dPos/dT;
    leftPID.Compute();

    lastPosRight = rightEnc.read();
    lastPosLeft = leftEnc.read();
    
    lastSpeedMesurement = micros();
    lastMeasUpdate = millis();
  }  
  // If the PID has been updated
  if((lastMeasUpdate >= lastTimerUpdate) && !stop)
  {
    digitalWrite(DIR_A, PWM_A_cmd > 0);
    digitalWrite(DIR_B, PWM_B_cmd > 0);
    
    analogWrite(PMW_A, abs(PWM_A_cmd));
    analogWrite(PMW_B, abs(PWM_B_cmd));

    lastTimerUpdate = millis();
  }
  // If the PID hasn't been updated for more than 500 ms (2Hz)
  if(millis()-lastCmdUpdate > 500)
  {
    // Stop the motors
    stop = true;

    // Reset I2C communication
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveData);

    rightPID.SetOutputLimits(0.0, 1.0);
    rightPID.SetOutputLimits(-1.0, 0.0);
    rightPID.SetOutputLimits(-255,255);

    leftPID.SetOutputLimits(0.0, 1.0);
    leftPID.SetOutputLimits(-1.0, 0.0);
    leftPID.SetOutputLimits(-255,255);
  }
  if(stop)
  {
    // Set the Velocities to zero
    speedRightTarget = 0;
    speedLeftTarget = 0;

    // Set motors command to 0
    analogWrite(PMW_A, 0);
    analogWrite(PMW_B, 0);
  }
}

// callback for received data from I2C
void receiveData(int byteCount)
{
          // Copy data in tmp buffer
          byte tmp[4];
          
          // Extract VRight from buffer
          for(int i = 0; i < 4; i++)
          {
            tmp[i] = Wire.read();
          }
          speedRightTarget = *((float*)(tmp));
  
          // Extract VLeft from buffer
          for(int i = 0; i < 4; i++)
          {
            tmp[i] = Wire.read();
          }
          speedLeftTarget = *((float*)(tmp));      

          for(int i = 0; i < 4; i++)
          {
            tmp[i] = Wire.read();
          }
          bool CS = (*((float*)(tmp)) == speedRightTarget+speedLeftTarget);
        
          // Read the end of the message
          while(Wire.available())
            Wire.read(); 

         if(CS)
         {
            // If Both Speed target are null, stop
            stop = (speedRightTarget == 0 && speedLeftTarget == 0);

            lastCmdUpdate = millis();
          }
}

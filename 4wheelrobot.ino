#include <PID_v1.h>
#include <Wire.h>
#include <Encoder.h>

// Slave Adress of the I2C Communcation
#define SLAVE_ADDRESS 0x12

// Two motors encoder
Encoder rightEnc(18,19), leftEnc(2,4);

// Motors Connexions
int PMW_A = 3;
int PMW_B = 11;
int DIR_A = 12;
int DIR_B = 13;

// PWM Command on each motors
double PWM_A_cmd, PWM_B_cmd;

// Timers
unsigned long timer;
unsigned long lastTimerUpdate;

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
 timer = millis();
  
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
  rightPID.SetOutputLimits(0,255);
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(10);
  leftPID.SetOutputLimits(0,255);

}

void loop() {
  // Every 10 ms (100Hz)
  if(micros() - lastSpeedMesurement>10000)
  {    
    // Compute Right Encoder Speed
    double dPos = rightEnc.read()-lastPosRight;
    double dT = (micros()-lastSpeedMesurement)*10e-6; //dT in seconds
    speedRight = 0.4*abs(dPos/dT);
    rightPID.Compute();

    // Compute Left Encoder Speed
    dPos = leftEnc.read()-lastPosLeft;
    dT = (micros()-lastSpeedMesurement)*10e-6; //dT in seconds
    speedLeft = 0.4*abs(dPos/dT);
    leftPID.Compute();

    lastPosRight = rightEnc.read();
    lastPosLeft = leftEnc.read();
    lastSpeedMesurement = micros();

    timer = millis();
  }  
  // If the PID has been updated
  if(timer != lastTimerUpdate && !stop)
  {
    digitalWrite(DIR_A, vleft <  0);
    digitalWrite(DIR_B, vright < 0);
    
    analogWrite(PMW_A, abs(PWM_A_cmd));
    analogWrite(PMW_B, abs(PWM_B_cmd));

    lastTimerUpdate = timer;
  }
  // If the PID hasn't been updated for more than 100 ms (10Hz)
  else if(millis()-lastTimerUpdate > 100)
  {
    // Stop the motors
    stop = true;

    // Reset I2C communication
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveData);
    
    timer = millis();
  }
  if(stop)
  {
    // Set the Velocities to zero
    speedRightTarget = 0;
    speedLeftTarget = 0;

    // Reset the PIDs
    rightPID = PID(&speedRight, &PWM_A_cmd, &speedRightTarget,consKp,consKi,consKd, DIRECT);
    leftPID = PID(&speedLeft, &PWM_B_cmd, &speedLeftTarget,consKp,consKi,consKd, DIRECT);

    // Set motors command to 0
    analogWrite(PMW_A, 0);
    analogWrite(PMW_B, 0);
  }
}

// callback for received data from I2C
void receiveData(int byteCount){

        // Copy data in tmp buffer
        byte tmp[4];

        // Extract VRight from buffer
        for(int i = 0; i < 4; i++)
        {
          tmp[i] = Wire.read();
          vright = *((float*)(tmp));
        }

        // Extract VLeft from buffer
        for(int i = 0; i < 4; i++)
        {
          tmp[i] = Wire.read();
          vleft = *((float*)(tmp));
        }

        // Speed in RPM
        speedRightTarget = abs(vright);
        speedLeftTarget = abs(vleft);

        // If Both Speed target are null, stop
        stop = (speedRightTarget == 0 && speedLeftTarget == 0);

        // Read the end of the message
        while(Wire.available())
          Wire.read(); 

        // Compute the new PID command
        rightPID.Compute();
        leftPID.Compute();

        timer = millis();
}

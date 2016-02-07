/*
Written by Cyrus Salem 2016/02/06
*/

#include <PinChangeInt.h>    // http://playground.arduino.cc/Main/PinChangeInt
#include <TimerOne.h>        // http://playground.arduino.cc/Code/Timer1
#include <Wire.h>
#include <PID_v1.h>

#define NO_PORTB_PINCHANGES //PinChangeInt setup
#define NO_PORTC_PINCHANGES    //only port D pinchanges (see: http://playground.arduino.cc/Learning/Pins)
#define PIN_COUNT 4    //number of channels attached to the reciver
#define MAX_PIN_CHANGE_PINS PIN_COUNT

//arduino pins attached to the reciver
#define RC_THROTTLE A2
#define RC_YAW A3
#define RC_PITCH A1
#define RC_ROLL A0
byte pin[] = {RC_THROTTLE, RC_YAW, RC_PITCH, RC_ROLL};    //for maximum efficency thise pins should be attached
unsigned int time[] = {0, 0, 0, 0};             // to the reciver's channels in the order listed here

byte state = 0;
byte burp = 0;  // a counter to see how many times the int has executed
byte cmd = 0;   // a place to put our serial data
byte i = 0;     // global counter for tracking what pin we are on


int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

//Analog output for motor drivers
int frontRightMotorOutput = 5;
int frontLeftMotorOutput = 3;
int rearRightMotorOutput = 6;
int rearLeftMotorOutput = 11;


//Radio receiver values
int radioThrottle, radioYaw, radioPitch, radioRoll;

double rollCompassInput, rollAnalogOutput, rollSetpoint, pitchCompassInput, pitchAnalogOutput, pitchSetpoint;

double Kp = 0.7, Ki = 0.2 , Kd = 0.08 ;
PID rollPID(&rollCompassInput, &rollAnalogOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);
PID pitchPID(&pitchCompassInput, &pitchAnalogOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);

#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002
#define OUTPUTMODE 1
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

float G_Dt = 0.02;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0; //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; //Omega Integrator
float Omega[3] = {0, 0, 0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] = {
  {
    1, 0, 0
  }
  , {
    0, 1, 0
  }
  , {
    0, 0, 1
  }
};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; //Gyros here


float Temporary_Matrix[3][3] = {
  {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
};

void setup()
{
  Serial.begin(115200);

  Serial.print("PinChangeInt ReciverReading test");
  Serial.println();            //warm up the serial port

  Timer1.initialize(2200);    //longest pulse in PPM is usally 2.1 milliseconds,
  //pick a period that gives you a little headroom.
  Timer1.stop();                //stop the counter
  Timer1.restart();            //set the clock to zero

  for (byte i = 0; i < 3; i++)
  {
    pinMode(pin[i], INPUT);     //set the pin to input
    digitalWrite(pin[i], HIGH); //use the internal pullup resistor
  }
  PCintPort::attachInterrupt(pin[i], rise, RISING); // attach a PinChange Interrupt to our first pin



  pinMode (STATUS_LED, OUTPUT); // Status LED

  pinMode(frontRightMotorOutput, OUTPUT);
  pinMode(frontLeftMotorOutput, OUTPUT);
  pinMode(rearRightMotorOutput, OUTPUT);
  pinMode(rearLeftMotorOutput, OUTPUT);

  I2C_Init();

  Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  digitalWrite(STATUS_LED, LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for (int i = 0; i < 32; i++) // We take some readings...
  {
    Read_Gyro();
    Read_Accel();
    for (int y = 0; y < 6; y++) // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
  }

  for (int y = 0; y < 6; y++)
    AN_OFFSET[y] = AN_OFFSET[y] / 32;

  AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for (int y = 0; y < 6; y++)
    Serial.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED, HIGH);

  timer = millis();
  delay(20);
  counter = 0;

  rollPID.SetSampleTime(10);
  pitchPID.SetSampleTime(10);

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-500, 500);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-500, 500);
}

void loop() //Main Loop
{
  for (byte i = 0; i < PIN_COUNT; i++)
  {
    if (i == 0) {
      radioThrottle = time[i];
    } else if (i == 1) {
      radioYaw = time[i];
    } else if (i == 2) {
      radioPitch = time[i];
    } else if (i == 3) {
      radioRoll = time[i];
    }
  }
  counter++;
  timer_old = timer;
  timer = millis();
  if (timer > timer_old)
    G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
  else
    G_Dt = 0;

  // *** DCM algorithm
  // Data adquisition
  Read_Gyro();   // This read gyro data
  Read_Accel();     // Read I2C accelerometer

  // if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
  //  {
  //   counter=0;
  //   Read_Compass();    // Read I2C magnetometer
  //  Compass_Heading(); // Calculate magnetic heading
  //   }

  // Calculations...
  Matrix_update();
  Normalize();
  Drift_correction();
  Euler_angles();
  
  //printdata();

  rollCompassInput = ToDeg(roll);
  pitchCompassInput = ToDeg(pitch);

  rollPID.Compute();
  pitchPID.Compute();

  /*  cmd=Serial.read();
    if (cmd=='p')
    {
        Serial.print("time:\t");
        for (byte i=0; i<PIN_COUNT;i++)
        {
            Serial.print(i,DEC);
            Serial.print(":");
            Serial.print(time[i],DEC);
            Serial.print("\t");
        }
        Serial.print(burp, DEC);
        Serial.println();
    }
    cmd=0;*/

  radioThrottle = map(radioThrottle, 1000, 1600, 0, 220);
  radioYaw = map(radioYaw, 1281, 1741, -300, 300);
  radioPitch = map(radioPitch, 1266, 1713, 100, -100);
  radioRoll = map(radioRoll, 1307, 1722, 100, -100);

  rollSetpoint = radioRoll;
  pitchSetpoint = radioPitch;

  int frontRightMotorOutputValue = radioThrottle - rollAnalogOutput + pitchAnalogOutput + radioYaw;
  int frontLeftMotorOutputValue = radioThrottle + rollAnalogOutput + pitchAnalogOutput - radioYaw;
  int rearRightMotorOutputValue = radioThrottle - rollAnalogOutput - pitchAnalogOutput - radioYaw;
  int rearLeftMotorOutputValue = radioThrottle + rollAnalogOutput - pitchAnalogOutput + radioYaw;

  if (frontRightMotorOutputValue > 255) frontRightMotorOutputValue = 255;
  if (frontLeftMotorOutputValue > 255) frontLeftMotorOutputValue = 255;
  if (rearRightMotorOutputValue > 255) rearRightMotorOutputValue = 255;
  if (rearLeftMotorOutputValue > 255) rearLeftMotorOutputValue = 255;

  if (frontRightMotorOutputValue < 0) frontRightMotorOutputValue = 0;
  if (frontLeftMotorOutputValue < 0) frontLeftMotorOutputValue = 0;
  if (rearRightMotorOutputValue < 0) rearRightMotorOutputValue = 0;
  if (rearLeftMotorOutputValue < 0) rearLeftMotorOutputValue = 0;

  if (radioThrottle > 2) {
    analogWrite(frontRightMotorOutput, frontRightMotorOutputValue);

    analogWrite(frontLeftMotorOutput, frontLeftMotorOutputValue);

    analogWrite(rearRightMotorOutput, rearRightMotorOutputValue);

    analogWrite(rearLeftMotorOutput, rearLeftMotorOutputValue);
  } else {
    analogWrite(frontRightMotorOutput, 0);

    analogWrite(frontLeftMotorOutput, 0);

    analogWrite(rearRightMotorOutput, 0);

    analogWrite(rearLeftMotorOutput, 0);
  }

}

void rise()        //on the rising edge of the currently intresting pin
{
  Timer1.restart();        //set our stopwatch to 0
  Timer1.start();            //and start it up
  state = RISING;
  //  Serial.print('r');
  burp++;

  switch (state)
  {
    case RISING: //we have just seen a rising edge
      PCintPort::detachInterrupt(pin[i]);
      PCintPort::attachInterrupt(pin[i], fall, FALLING); //attach the falling end
      state = 255;
      break;
    case FALLING: //we just saw a falling edge
      PCintPort::detachInterrupt(pin[i]);
      i++;                //move to the next pin
      i = i % PIN_COUNT;  //i ranges from 0 to PIN_COUNT
      PCintPort::attachInterrupt(pin[i], rise, RISING);
      state = 255;
      break;
  }
}

void fall()        //on the falling edge of the signal
{

  state = FALLING;
  time[i] = readTimer1();  // read the time since timer1 was restarted
  //  time[i]=Timer1.read();    // The function below has been ported into the
  // the latest TimerOne class, if you have the
  // new Timer1 lib you can use this line instead
  Timer1.stop();
  //  Serial.print('f');

  switch (state)
  {
    case RISING: //we have just seen a rising edge
      PCintPort::detachInterrupt(pin[i]);
      PCintPort::attachInterrupt(pin[i], fall, FALLING); //attach the falling end
      state = 255;
      break;
    case FALLING: //we just saw a falling edge
      PCintPort::detachInterrupt(pin[i]);
      i++;                //move to the next pin
      i = i % PIN_COUNT;  //i ranges from 0 to PIN_COUNT
      PCintPort::attachInterrupt(pin[i], rise, RISING);
      state = 255;
      break;
  }
}

unsigned long readTimer1()        //returns the value of the timer in microseconds
{ //rember! phase and freq correct mode counts
  //up to ICR1 then down again
  unsigned int tmp = TCNT1;
  char scale = 0;
  switch (Timer1.clockSelectBits)
  {
    case 1:// no prescalse
      scale = 0;
      break;
    case 2:// x8 prescale
      scale = 3;
      break;
    case 3:// x64
      scale = 6;
      break;
    case 4:// x256
      scale = 8;
      break;
    case 5:// x1024
      scale = 10;
      break;
  }
  while (TCNT1 == tmp) //if the timer has not ticked yet
  {
    //do nothing -- max delay here is ~1023 cycles
  }
  tmp = (  (TCNT1 > tmp) ? (tmp) : (ICR1 - TCNT1) + ICR1  ); //if we are counting down add the top value
  //to how far we have counted down
  return ((tmp * 1000L) / (F_CPU / 1000L)) << scale;
}

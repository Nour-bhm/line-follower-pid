#include <EEPROM.h>
// ===== SENSOR CONFIGURATION =====
#define SENSOR_COUNT 8
const int ANALOG[SENSOR_COUNT] = { A7, A6, A5, A4, A3, A2, A1, A0 };
int           IR[SENSOR_COUNT] = { 0 };
int        IRMAX[SENSOR_COUNT] = { 0 };
int        IRMIN[SENSOR_COUNT] = { 0 };
int        IRMID[SENSOR_COUNT] = { 0 };
int           ir[SENSOR_COUNT] = { 0 };

// ===== MOTOR CONFIGURATION =====
#define LEFT__IN1 3
#define LEFT__IN2 5
#define RIGHT_IN3 6
#define RIGHT_IN4 9

// ===== PID CONFIGURATION =====
#define BASE_SPEED 400    // Base motor speed500
#define MAX__SPEED 1000   // Max motor speed
#define MAX_PWM    255     // Maximum PWM value
#define PID_CALC() (Kp*linpos + Ki*linint + Kd*linspd)
#define EPS        1e-6f
#define DT         (T*1e-6f)
float linpos   =   EPS;
float corr     =   0.0f;
float linint   =   0.0f;
float exlinpos =   0.0f;
float linspd   =   0.0f;
float Kp       =  45.0f;
float Ki       =   0.0f;
float Kd       =   7.0f;
int   basespd  =   BASE_SPEED;
unsigned long time = 0L;
unsigned long T = 0L;
unsigned long turntimer = 0L;
#define FOLW 0
#define RGHT 1
#define ENND 2
int state = FOLW;

// Path optimization
#define TURNTIME 1500000
char stack[512] { 0 };
int itr = 0;
int pthladdr = 0x00;
int pathaddr = 0x04;
int thend    = 0;



// ===== SETUP FUNCTION =====
void setup()
{
  Serial.begin(9600);
  while (!Serial) { delay(10); }

  EEPROM.get(pthladdr, itr);
  Serial.println(itr);
  for (int i = 0; i < itr; i++)
  {
    char c;
    EEPROM.get(pathaddr + i, c);
    Serial.println(c);
  }
  itr = 0;
  Serial.println("\n");
  
  Serial.println("QTR-8RC PID Line Follower Starting...");
  
  // Initialize motor pins
  pinMode(LEFT__IN1, OUTPUT);
  pinMode(LEFT__IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  
  IRCALIBRAIT();
  
  Serial.println("Ready! Place robot on line...");
  delay(200);
}

// ===== MAIN LOOP =====
void loop()
{
  // Read sensors, get line position and speed, update state
  LINPOS();
  
  switch (state)
  {
    case FOLW:
      MOTORING(corr = PID_CALC());
      break;
    case LEFT:
      MOTORING(MAX__SPEED);
      delay(200);
      MOTORING(0);
      delay(100);
      linpos = abs(linpos)/linpos;
      break;
    case ENND:
      THEND();
      while (1);
  }
  
  // Print debug info (optional - slows down loop)
  // PRINTDEBUG(linpos, corr);
}

// ===== SENSOR CALIBRATION FUNCTIONS =====
void IRCALIBRAIT()
{
  for(int A = 0; A < SENSOR_COUNT; A++)
  {
    IR[A]    = analogRead(ANALOG[A]);
    IRMIN[A] = IR[A];
    IRMAX[A] = IR[A];
  }

  for(int i = 0; i < 500; i++)
  {
    for(int A = 0; A < SENSOR_COUNT; A++)
    {
      IR[A] = analogRead(ANALOG[A]);
      if(IR[A] > IRMAX[A]) IRMAX[A] = IR[A];
      if(IR[A] < IRMIN[A]) IRMIN[A] = IR[A];
    }
    delay(10);
  }

  for(int A = 0; A < SENSOR_COUNT; A++)
  {
      IRMID[A] = (IRMAX[A] + IRMIN[A])*0.5;
      Serial.print(IRMIN[A]);
      Serial.print("   ");
      Serial.print(IRMAX[A]);
      Serial.print("   ");
      Serial.println(IRMID[A]);
  }
}

void READQTR()
{
  for(int A = 0; A < SENSOR_COUNT; A++)
  {
    IR[A] = analogRead(ANALOG[A]);
    if(IR[A] < IRMID[A]) ir[A] = 0;
    else                 ir[A] = 1;
  }
}

// ===== STATE HANDLING FUNCTIONS =====
#define cm 80.0f
void LINPOS()
{
  for(int A = 0; A < SENSOR_COUNT; A++)
  {
    IR[A] = analogRead(ANALOG[A]);
    if(IR[A] < IRMID[A]) ir[A] = 0;
    else                 ir[A] = 1;
  }

  T    = -time;
  time = micros();
  T    = time + T;

  exlinpos = linpos;
  int r90   = ir[0] + ir[1] + ir[2] + ir[3];
  int l90   = ir[4] + ir[5] + ir[6] + ir[7];
  int allir = l90 + r90; 
  if (allir != 0 && allir != 8)
  {
    if      (l90 == 4)
    {
      linpos = cm;
      state = LEFT;
      thend = 0;
      if ((time - turntimer) > TURNTIME)
      {
        stack[itr++] = 'L';
        turntimer = time;
      }
    }
    else if (r90 == 4)
    {
      linpos = -cm;
      state = FOLW;
      thend = thend*2;
      if ((time - turntimer) > TURNTIME)
      {
        stack[itr++] = 'R';
        turntimer = time;
      }
    }
    else
    {
      linpos = (-16*ir[0] - 9*ir[1] - 4*ir[2] - 1*ir[3] + 1*ir[4] + 4*ir[5] + 9*ir[6] + 16*ir[7] + EPS)/(allir + EPS);
      state = FOLW;
      thend = 0;
    }
  }
  else
  {
    if (allir == 0)
    {
      linpos = cm*abs(linpos)/linpos;
      state = FOLW;
      thend = 0;
      if ((time - turntimer) > TURNTIME)
      {
        stack[itr++] = 'U';
        turntimer = time;
      }
    }
    else
    {
      linpos = cm;
      state = LEFT;
      thend++;
      if ((time - turntimer) > TURNTIME)
      {
        stack[itr++] = 'L';
        turntimer = time;
      }
    }
  }

  if (thend > 2)
  {
    state = ENND;
    return;
  }

  // linint = constrain(linint + linpos*DT, -10.0f, 10.0f);
  linspd = (linpos - exlinpos)/DT;
}

// ===== MOTOR CONTROL FUNCTIONS =====
void MOTORING(float corr)
{
  int lmot = basespd + corr;
  int rmot = basespd - corr;

  lmot = map(constrain(lmot, -MAX__SPEED, MAX__SPEED), -MAX__SPEED, MAX__SPEED, -MAX_PWM, MAX_PWM);
  rmot = map(constrain(rmot, -MAX__SPEED, MAX__SPEED), -MAX__SPEED, MAX__SPEED, -MAX_PWM, MAX_PWM);

  analogWrite(LEFT__IN2, constrain(-lmot, 0, MAX_PWM));
  analogWrite(LEFT__IN1, constrain( lmot, 0, MAX_PWM));

  analogWrite(RIGHT_IN4, constrain(-rmot, 0, MAX_PWM));
  analogWrite(RIGHT_IN3, constrain( rmot, 0, MAX_PWM));
}

void THEND()
{
  analogWrite(LEFT__IN2, 0);
  analogWrite(LEFT__IN1, 0);

  analogWrite(RIGHT_IN4, 0);
  analogWrite(RIGHT_IN3, 0);

  //TODO: Optimise path
  EEPROM.put(pthladdr, itr);
  for (int i = 0; i < itr; i++)
  {
    EEPROM.put(pathaddr + i, stack[i]);
  }
}

// ===== DEBUG FUNCTIONS =====
void PRINTDEBUG(int err, int corr)
{
  Serial.print("Err:\t");
  Serial.print(err);
  Serial.print("\t\t| Corr:\t");
  Serial.print(corr);
  
  // Print sensor values
  Serial.print("\t\t| IR:\t");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print(ir[i]);
    Serial.print(' ');
  }
  Serial.println();
}
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>
 
#define STEP_PIN A4
#define DIR_PIN A3
#define TURNTABLE_PORT 2

#define STEP_MAX_SPEED 100
#define STEP_MIN_SPEED 4000
#define CAM_HEIGHT 103000

AF_DCMotor turntableMotor(TURNTABLE_PORT); 
SoftwareSerial bluetoothMate(A1, A0);

boolean homing = false;
boolean stepperOn = false;
boolean stepSpeed = 1;
boolean stepDirection = HIGH;
long stepCount = 0;

void doStep()
{
   if (homing) {
     digitalWrite(DIR_PIN, HIGH);
     if (digitalRead(A2) == HIGH) {
       homing = false;
       stepperOn = false;
       stepCount = 0;
     }
     
     digitalWrite(STEP_PIN, LOW);
     digitalWrite(STEP_PIN, HIGH);
    return;     
   }
  
   if (!stepperOn)
     return; 
 
   if (stepCount >= CAM_HEIGHT && stepDirection == LOW) {
     stepperOn = false;
     return;
  }
 
   if (stepCount <= 0 && stepDirection == HIGH) {
     stepperOn = false;
     return;
   }
   
   if (stepDirection == HIGH && digitalRead(A2) == HIGH) {     
     // Endstop pressed.
     stepperOn = false;
     stepCount = 0;
   } else {
     if (stepDirection == HIGH)
       stepCount--;
     else
       stepCount++;
     
     digitalWrite(STEP_PIN, LOW);
     digitalWrite(STEP_PIN, HIGH);
   }
}

void setup()
{
  // Serial over USB
  Serial.begin(9600);        
 
  // Software Serial for RN42 Bluetooth
  bluetoothMate.begin(115200);
  bluetoothMate.print("$");
  bluetoothMate.print("$");
  bluetoothMate.print("$");
  delay(500);
  bluetoothMate.println("U,9600,N");
  bluetoothMate.begin(9600); 
 
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(A2, INPUT);
  digitalWrite(A2, HIGH); // Enable pullup

  // Set initial direction
  digitalWrite(STEP_PIN, stepDirection);
 
  // Setup Stepper Timer Handler
  Timer1.initialize(100);
  Timer1.attachInterrupt(doStep);
 
  turntableMotor.setSpeed(255);
  turntableMotor.run(RELEASE);    
}

int sCmd;
int lCmd;
int cameraDir = FORWARD;
int turntableDir = FORWARD;
int turntableSpeed = 255;

void loop()
{   
  if (bluetoothMate.available()) {
    lCmd = sCmd;
    sCmd = bluetoothMate.read();
  } else {
    sCmd = 0;
  }
  
  switch (sCmd) {
    case '?':
      bluetoothMate.print("Available Commands:\r\n");
      bluetoothMate.print("\r\n");
      bluetoothMate.print("p - Display current step count.\r\n");
      bluetoothMate.print("- - Decrease turntable speed.\r\n");
      bluetoothMate.print("= - Increase turntable speed.\r\n");
      bluetoothMate.print("] - Increase camera drive speed.\r\n");
      bluetoothMate.print("[ - Decrease camera drive speed.\r\n");
      bluetoothMate.print("H - Home camera drive.\r\n");
      bluetoothMate.print("w - Move camera drive up.\r\n");
      bluetoothMate.print("s - Move camera drive down.\r\n");
      bluetoothMate.print("x - Stop camera drive.\r\n");
      bluetoothMate.print("a - Move turntable forward.\r\n");
      bluetoothMate.print("d - Move turntable backwards.\r\n");
      bluetoothMate.print("z - Stop turntable drive.\r\n");
      bluetoothMate.print("\r\n");
      break;
    case 'p':
      bluetoothMate.print("Step Count: ");
      bluetoothMate.print(stepCount);
      bluetoothMate.print("\r\n");
      break;
    case '-':
      if (turntableSpeed <= 5)
        turntableSpeed = 0;
      else
        turntableSpeed -= 5;
       
      bluetoothMate.print("Speed now ");
      bluetoothMate.print(turntableSpeed);
      bluetoothMate.print("\r\n");
      
      turntableMotor.setSpeed(turntableSpeed);
      break;
    case ']':
      if (stepSpeed <= 1)
        stepSpeed = 1;
      else 
        stepSpeed -= 1;

      Timer1.setPeriod(100 * stepSpeed);
      bluetoothMate.print("Stepper speed: ");
      bluetoothMate.print(stepSpeed);
      bluetoothMate.print("\r\n");
      break;
    case '[':
      if (stepSpeed >= 30)
        stepSpeed = 30;
      else
        stepSpeed += 1;

      Timer1.setPeriod(100 * stepSpeed);
      bluetoothMate.print("Stepper speed: ");
      bluetoothMate.print(stepSpeed);
      bluetoothMate.print("\r\n");
      break;
    case 'H':
      homing = true;
      stepSpeed = 1;
      break;    
    case 'w':
      stepDirection = LOW;
      digitalWrite(DIR_PIN, LOW);
      stepperOn = true;
      break;
    case 's':
      stepDirection = HIGH;
      digitalWrite(DIR_PIN, HIGH);
      stepperOn = true;
    
      break;
    case 'x':
      stepperOn = false;
      break;
    case '=':
      if (turntableSpeed >= 250)
        turntableSpeed = 255;
      else
        turntableSpeed += 5;
      
      bluetoothMate.print("Speed now ");
      bluetoothMate.print((int)turntableSpeed, DEC);
      bluetoothMate.print("\r\n");
      
      turntableMotor.setSpeed(turntableSpeed);
      break;
    case 'a':
      turntableDir = BACKWARD;
      turntableMotor.run(BACKWARD);
      break;
    case 'd':
      turntableDir = FORWARD;
      turntableMotor.run(FORWARD);
      break;
    case 'z':
      turntableMotor.run(RELEASE);
      break;
  }
}

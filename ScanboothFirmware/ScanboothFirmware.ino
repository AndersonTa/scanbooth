
// Commented heavily for teaching purposes.

#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>
 
// Pin definitions
#define STEP_PIN A4
#define DIR_PIN A3
#define ENDSTOP_PIN A2

// AF Motor Shield Port
#define TURNTABLE_PORT 2

// Various constraints
#define STEP_MAX_SPEED 100
#define STEP_MIN_SPEED 4000
#define CAM_HEIGHT 103000

// Define classes for controlling the turntable motor and the bluetooth module.
AF_DCMotor turntableMotor(TURNTABLE_PORT); 
SoftwareSerial bluetoothMate(A1, A0);

// Some misc state variables
boolean homing = false;
boolean stepperOn = false;
boolean stepSpeed = 1;
boolean stepDirection = HIGH;
long stepCount = 0;

// This is the core of the stepping routine. This is called by the timer
// interrupt service routine which is initialized using the Timer1 helper
// library.
void doStep()
{
   // If we're in a homing state, ignore all other stepping function and
   // and just keep going until we hit an endstop.
   if (homing) {
     // Forces the direction (up in our case)
     digitalWrite(DIR_PIN, HIGH);
     // Reads 
     if (digitalRead(ENDSTOP_PIN) == HIGH) {
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
   
   if (stepDirection == HIGH && digitalRead(ENDSTOP_PIN) == HIGH) {     
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
  // The normal Arduino serial connection over it's USB port.
  Serial.begin(9600);        
 
  // Software Serial for RN42 Bluetooth
  // By default the SparkFun RN42 Bluetooth module works at 115200 baud.
  // We then throw three $ into it in order to trigger command mode.
  bluetoothMate.begin(115200);
  bluetoothMate.print("$");
  bluetoothMate.print("$");
  bluetoothMate.print("$");
  // Small delay while it enters command mode.
  delay(500);
  // Send the command to set it's baud to 9600 no parity.
  bluetoothMate.println("U,9600,N");
  // Change baud to 9600.
  bluetoothMate.begin(9600); 
 
  // Setup the pins we use as either OUTPUT or INPUT.

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENDSTOP_PIN, INPUT);

  // In Arduino, output HIGH to a pin thats set to be an INPUT triggers a special
  // feature that turns on the atmega chips internal pullup resistor. The pullup
  // keeps the logic high when there is no active input, it prevents noise/interference
  // from causing the input to flip high/low randomly. 
  digitalWrite(ENDSTOP_PIN, HIGH);

  // Set initial direction
  digitalWrite(STEP_PIN, stepDirection);
 
  // Setup Stepper Timer Handler
  // We initialize it at 100 microseconds. (uS)
  Timer1.initialize(100);
  // We tell it to call the function doStep() everytime the timer is reached.
  Timer1.attachInterrupt(doStep);
 
  // Set the motor to full duty cycle for the PWM (255)
  turntableMotor.setSpeed(255);
  // Make sure motor is stopped and released.
  turntableMotor.run(RELEASE);    
}

int sCmd;
int lCmd;
int cameraDir = FORWARD;
int turntableDir = FORWARD;
int turntableSpeed = 255;

// The main program loop. The Arduino environment constructs its own main()
// that does initializations and calls setup() and then ultimately runs loop()
// in a loop.
void loop()
{   
  // Here we check if there is any data available on the bluetooth channel.
  // If there is, we read it in.
  if (bluetoothMate.available()) {
    lCmd = sCmd;
    sCmd = bluetoothMate.read();
  } else {
    sCmd = 0;
  }
  
  // Single command handler. Each command is listed in the case statement.
  // Pressing the character contained in it will do the resulting functionality.
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

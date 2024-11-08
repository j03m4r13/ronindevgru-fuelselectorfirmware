#include <Servo.h>
#include <RotaryEncoder.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

#define AUTO_SELECT 1
#define MANUAL_SELECT 2

#define STALL_VALUE     100 // [0..255]
#define R_SWITCH 10
#define L_SWITCH 14

#define EN_PIN           6 // Enable
#define DIR_PIN          7 // Direction
#define STEP_PIN         8 // Step
#define SW_RX            4 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            5 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERVO_PIN        9
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);

int LButtonState = 0;
int lastLButtonState = 0;
unsigned long lastLDebounceTime = 0;  // the last time the output pin was toggled
int RButtonState = 0;
int lastRButtonState = 0;
unsigned long lastRDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
bool shaft = false;
double currSelectorPosition = 0.0;
double prevSelectorPosition = -1.0;
float midPoint = 0.07;
long fullRevolutionSteps = 450;
long midStepper = fullRevolutionSteps / 2;
float pointer = 0.0;
String planeTrimData;
String inputString = "";  // String to hold incoming data
bool stringComplete = false;  // Flag to indicate message completion
int state = 0;
int autoSelectMode = MANUAL_SELECT;
bool manualSwitchOn = false;
bool hasSentAPOff = false;
int currSelectorDirection = 0.0;
int prevSelectorDirection = 0.0;
bool hasHomed = false;

long mapInputToSteps(double fuelSelector){
  if (fuelSelector >= 2.0) fuelSelector = 2.0;
  if (fuelSelector <= 0.0) fuelSelector = 0.0;
  long steps = fuelSelector * fullRevolutionSteps;
  return steps;
}

void setup() {
Serial.begin(250000);         // Init serial port and set baudrate
while(!Serial);               // Wait for serial port to connect
  //SERIAL_PORT.begin(115200);
  //driver.beginSerial(115200);
  pinMode(L_SWITCH, INPUT);
  pinMode(R_SWITCH, INPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  driver.begin();
  driver.toff(4);          // Enable driver (off_time)
  driver.rms_current(800); // Set motor current (mA)
  driver.microsteps(16);   // Set microstepping (16 microsteps per step)
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(1000);



  LButtonState = 0;
  lastLButtonState = 0;
  lastLDebounceTime = 0;  // the last time the output pin was toggled
  RButtonState = 0;
  lastRButtonState = 0;
  lastRDebounceTime = 0;  // the last time the output pin was toggled
  autoSelectMode = MANUAL_SELECT;
  currSelectorPosition = 0.0;
  prevSelectorPosition = -1.0;

}
bool isFloat(String str) {
    if (str.length() == 0) return false;
    int dotCount = 0;
    for (int i = 0; i < str.length(); i++) {
        if (i == 0 && (str[i] == '+' || str[i] == '-')) {
            continue; // Allow leading '+' or '-'
        }
        if (str[i] == '.') {
            dotCount++;
            if (dotCount > 1) return false; // Only one dot allowed
        } else if (!isDigit(str[i])) {
            return false; // Non-numeric character found
        }
    }
    return true;
}

void homeStepper(){
        long steps = mapInputToSteps(2.0);
        int LButton;
        int RButton;
        stepper.setCurrentPosition(0.0);
        stepper.moveTo(-1800);
        // Keep running the stepper
        while (stepper.distanceToGo() != 0) {
          
          LButton = digitalRead(L_SWITCH);
          RButton = digitalRead(R_SWITCH);
          if (lastLButtonState != LButton){
            lastLDebounceTime = millis();
          }
          if (lastRButtonState != RButton){
            lastRDebounceTime = millis();
          }

          if ((millis() - lastLDebounceTime) > debounceDelay) {
              if (RButton == LOW) { // found LEFT position

                stepper.stop();
                stepper.setCurrentPosition(mapInputToSteps(1.0)*-1); // this is the LEFT Position
                lastLButtonState = LButton;
                lastRButtonState = RButton;
                return;

              }
              RButtonState = RButton;
          }

          stepper.run();  // Run the motor to the target position
          lastLButtonState = LButton; 
          lastRButtonState = RButton;
        }
        stepper.setCurrentPosition(mapInputToSteps(1.0)*-1); // JUST SET TO R FOR NOW
}
void loop() {
  static int dir = 0;
  static int manual_dir = 0;
  int LButton;
  int RButton;

  int currManualDir = 0;

  if (autoSelectMode == MANUAL_SELECT){
    LButton = digitalRead(L_SWITCH);
    RButton = digitalRead(L_SWITCH);
    if (lastLButtonState != LButton){
      lastLDebounceTime = millis();
    }
    if (lastRButtonState != RButton){
      lastRDebounceTime = millis();
    }

    if ((millis() - lastLDebounceTime) > debounceDelay) {
        if (LButton == LOW) {
          digitalWrite(EN_PIN, LOW);
          stepper.setSpeed(900);
          stepper.runSpeed();
          Serial.println("-1");
        } else {
          digitalWrite(EN_PIN, HIGH);
        }
        LButtonState = LButton;
    }
    if ((millis() - lastRDebounceTime) > debounceDelay) {
        if (RButton == LOW) {
          if (autoSelectMode == MANUAL_SELECT) {
            digitalWrite(EN_PIN, LOW);
            stepper.setSpeed(-900);
            stepper.runSpeed();
            //Serial.println("1");

          }
        } else {
            digitalWrite(EN_PIN, HIGH);
        }
        RButtonState = RButton;
    }
  }
    // Check if a message has been completed
  if (stringComplete) {
    if (inputString == "R\n"){
      hasSentAPOff = false;
      hasHomed = false;
      autoSelectMode = AUTO_SELECT;
      digitalWrite(EN_PIN, LOW);
      homeStepper();
      currSelectorPosition = 1.0; //RESET TO LEFT;
      prevSelectorPosition = 1.0;
      // long steps = mapInputToSteps(currSelectorPosition);
      // stepper.setCurrentPosition(steps); // no physical movements, just setting the position internally
    } else if (inputString == "T\n"){
      autoSelectMode = MANUAL_SELECT;
      digitalWrite(EN_PIN, HIGH);
    } else {
      inputString.trim();
      if(isFloat(inputString)){
        currSelectorPosition = inputString.toDouble();
      }
    }
    // Clear the string for the next input
    inputString = "";
    stringComplete = false;
  }
  serialEvent();
  if (currSelectorPosition != prevSelectorPosition) {

      if (autoSelectMode == AUTO_SELECT){
        long steps = mapInputToSteps(currSelectorPosition);
        stepper.moveTo(steps*-1);
        // Keep running the stepper
        while (stepper.distanceToGo() != 0) {
          stepper.run();  // Run the motor to the target position
          serialEvent();
          if (stringComplete){
            stepper.stop();  // Decelerates to stop
            break;           // Exit the while loop after stopping
          }
        }
      } else if (autoSelectMode == MANUAL_SELECT) {
        long steps = mapInputToSteps(currSelectorPosition);
        stepper.setCurrentPosition(steps*-1); // no physical movements, just setting the position internally
      }
    prevSelectorPosition = currSelectorPosition;
  }

  lastLButtonState = LButton;
  lastRButtonState = RButton;
}
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
#include <Servo.h>
#include <RotaryEncoder.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

#define AUTO_SELECT 1
#define MANUAL_SELECT 2

#define LEFT 1
#define RIGHT 2
#define OFF 3

#define STALL_VALUE     100 // [0..255]
#define R_SWITCH 10
#define L_SWITCH 14
#define O_SWITCH 16
#define MICRO_SWITCH 9

#define EN_PIN           6 // Enable
#define DIR_PIN          7 // Direction
#define STEP_PIN         8 // Step
#define SW_RX            4 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            5 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SHIFT_PIN        15
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

int OButtonState = 0;
int lastOButtonState = 0;
unsigned long lastODebounceTime = 0;  // the last time the output pin was toggled

unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

int MButtonState = 0;
int lastMButtonState = 0;
unsigned long lastMDebounceTime = 0;  // the last time the output pin was toggled

bool shaft = false;
double currSelectorPosition = 0.0;
double prevSelectorPosition = -1.0;
float midPoint = 0.07;
long fullRevolutionSteps = 3200;
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
bool masterTriggered = false;
int lastDirection = 0.0;
int currDirection = 0.0;

long mapInputToSteps(double fuelSelector){
  if (fuelSelector >= 2.0) fuelSelector = 2.0;
  if (fuelSelector <= 0.0) fuelSelector = 0.0;
  fuelSelector = fuelSelector * (1.39 / 2.0); // for remapping in case of falling short for knob travel
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
  pinMode(O_SWITCH, INPUT);
  pinMode(MICRO_SWITCH, INPUT);
  pinMode(SHIFT_PIN, INPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  driver.begin();
  driver.toff(4);          // Enable driver (off_time)
  driver.rms_current(800); // Set motor current (mA)
  driver.microsteps(16);   // Set microstepping (16 microsteps per step)
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(3000);



  LButtonState = 0;
  lastLButtonState = 0;
  lastLDebounceTime = 0;  // the last time the output pin was toggled
  RButtonState = 0;
  lastRButtonState = 0;
  lastRDebounceTime = 0;  // the last time the output pin was toggled
  OButtonState = 0;
  lastOButtonState = 0;
  lastODebounceTime = 0;  // the last time the output pin was toggled
  MButtonState = 0;
  lastMButtonState = 0;
  lastMDebounceTime = 0;  // the last time the output pin was toggled
  autoSelectMode = MANUAL_SELECT;
  currSelectorPosition = 0.0;
  prevSelectorPosition = -1.0;
  hasHomed = false;
  masterTriggered = false;
  lastDirection = 0.0;
  currDirection = 0.0;

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
        
        int LButton;
        int RButton;
        int OButton;
        long stepsTaken = 0;
        long currSteps = 0;
        bool rightFound = false;
        int counter = 0;
        stepper.setCurrentPosition(0.0);
        stepper.moveTo(3200); // rotate full to search
        // Keep running the stepper
        while (stepper.distanceToGo() != 0) {
          

          RButton = digitalRead(R_SWITCH);

              if (RButton == LOW) { // found RIGHT position

                stepper.stop();
                stepper.setCurrentPosition(mapInputToSteps(1.0)*-1);
                return;
              }


          stepper.run();  // Run the motor to the target position

        }
        stepper.setCurrentPosition(mapInputToSteps(1.0)*-1); // JUST SET TO R FOR NOW
}
void loop() {
  static int dir = 0;
  static int manual_dir = 0;
  int LButton;
  int RButton;
  int OButton;
  int MButton;

  int currManualDir = 0;
  MButton = digitalRead(MICRO_SWITCH);
  if (lastMButtonState != MButton){
    lastMDebounceTime = millis();
  }
  if ((millis() - lastMDebounceTime) > debounceDelay) {
      if (MButton == LOW) { // microswitch triggered
        masterTriggered = true;
              

        // LButtonState = 0;
        // lastLButtonState = 0;
        // lastLDebounceTime = 0;  // the last time the output pin was toggled
        // RButtonState = 0;
        // lastRButtonState = 0;
        // lastRDebounceTime = 0;  // the last time the output pin was toggled
        // OButtonState = 0;
        // lastOButtonState = 0;
        // lastODebounceTime = 0;  // the last time the output pin was toggled
        // MButtonState = 0;
        // lastMButtonState = 0;
        // lastMDebounceTime = 0;  // the last time the output pin was toggled
        // lastDirection = -1.0;

      } else {
        masterTriggered = false;
        digitalWrite(EN_PIN, HIGH); // disable stepper when master is disengaged
      }
      MButtonState = MButton;
  }

  if (autoSelectMode == MANUAL_SELECT && masterTriggered ){
    LButton = digitalRead(L_SWITCH);
    RButton = digitalRead(R_SWITCH);


    if (lastLButtonState != LButton){
      lastLDebounceTime = millis();
      currDirection = LEFT;
    }
    if (lastRButtonState != RButton){
      lastRDebounceTime = millis();
      currDirection = RIGHT;
    }


    if ((millis() - lastLDebounceTime) > debounceDelay) {
        if (LButton == LOW && currDirection != lastDirection) {
          
          lastDirection = currDirection;
          Serial.println("2");
        }
        LButtonState = LButton;
    }
    if ((millis() - lastRDebounceTime) > debounceDelay) {
        if (RButton == LOW && currDirection != lastDirection) {
          
          lastDirection = currDirection;
          Serial.println("1");

        }
        RButtonState = RButton;
    }

  } else if (autoSelectMode == MANUAL_SELECT && !masterTriggered) {
    OButton = digitalRead(O_SWITCH);
    if (lastOButtonState != OButton && currDirection != lastDirection){
      lastODebounceTime = millis();
      currDirection = OFF;

    }
    if ((millis() - lastODebounceTime) > debounceDelay) {
        if (OButton == LOW) {
          Serial.println("0");
          digitalWrite(EN_PIN, HIGH);
          lastDirection = currDirection;
          lastRButtonState = 0; //since Right is already ON even when master is at off position
        }
        OButtonState = OButton; 
    }
  }

    // Check if a message has been completed
  if (stringComplete) {
    if (inputString == "R\n"){
      hasSentAPOff = false;
      hasHomed = false;
      autoSelectMode = AUTO_SELECT;
      if (masterTriggered) {
        digitalWrite(EN_PIN, LOW);
        homeStepper();
        currSelectorPosition = 1.0; //RESET TO right;
        prevSelectorPosition = 1.0;
      }

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

      if (autoSelectMode == AUTO_SELECT && currSelectorPosition  <= 2.0 && currSelectorPosition >= 1.0){
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
      } else if (autoSelectMode == MANUAL_SELECT && currSelectorPosition  <= 2.0 && currSelectorPosition >= 1.0) {
        long steps = mapInputToSteps(currSelectorPosition);
        stepper.setCurrentPosition(steps*-1); // no physical movements, just setting the position internally
      }
    prevSelectorPosition = currSelectorPosition;
  }

  lastLButtonState = LButton;
  lastRButtonState = RButton;
  lastOButtonState = OButton;
  lastMButtonState = MButton;
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
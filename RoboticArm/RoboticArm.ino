#include <Servo.h>

/*
 * To use this program use the two buttons for control, the initial state is "Step".
 * 
 * Button4
 *  First click "Run" run testMovement or Program
 *  Second click "Step" start program stepping mode (each subsequent click advances program one line)
 *  
 * Button7
 *  First click "Stop" stop movement/programming/stepping
 *  Second click "Driving" - driving mode - arms move according to pot values (each subsequent click adds a line to the program)
 *  
 *  To see output connect Tools -> Serial Monitor 9600 Baud
 *    In stepping mode each step outputs code that can be used to initialise
 *    the program, look for "// Start of Program" and "// End of Program"
 */

// These control the start angle and the degree of movement to each side when
// running without a program.
// Moves from testMoveStart - testMoveRange/2 to testMoveStart + testMoveRange/2
const int testMoveStart = 90;
const int testMoveRange = 90;

// uArm Pin settings
const int button4Pin = 4;
const int button7Pin = 7;
const int servo0Pin = 8; // increments pins for each servo so servo3 is pin 10
const int pot0Pin = A5; // decrements pins for each pot so pot3 for servo3 is A3

// The PWM settings for each servo, defaults are 544 to 2400
// the third setting is a speed multiplier as a percentage eg 100 for 100% speed (unused)
const int servoLimits[6][3] = {
  {800, 2200, 100}, // 6 (fingers)
  {750, 2000, 100}, // 4 (white rotation)
  {900, 1800, 20}, // 3? (point down/up)
  {750, 1900, 20}, // 2? (forwards backwards)
  {544, 2400, 100},
  {544, 2400, 100}
};

// These variables are set to true when a button press is detected and set to false when
// the button action has been handled.
bool button4Pressed = false;
bool button7Pressed = false;

// Allow a certain amount of movement to happen without being detected as "different"
// this accounts for small changes to the pot values that were unintentional.
// If the calculated angle is within jitter degrees of the old value it isn't counted
// as having changed. So you can press record new line multiple times but only one value
// is stored.
// This is also used in driving mode to not move the arm until the new value is at least
// jitter degrees that the current value (makes movement less accurate but less jerky).
const int jitter = 5;

// How many servos/pots are connected - starting at servo0Pin and pot0Pin
const int servoCount = 4;

// Control objects for each servo
Servo servos[servoCount];

// Values for each pot (used in driving mode), 0 to 1
float pots[servoCount];

// This is where the program is stored. The maximum size is programMaximumSize. Each
// line in the array contains the angles of each servo the extra item being the number
// steps for the movement currently uses defaultMoveDuration.
const int programMaximumSize = 100;
byte program[programMaximumSize][servoCount+1];
int programLength = 0;
int defaultMoveDuration = 50;

/*
 * From https://github.com/tobiastoft/ArduinoEasing/blob/master/Easing.cpp
 * quadratic easing in/out - acceleration until halfway, then deceleration
 */
float easeInOutQuad (float t, float b, float c, float d)
{
  if ((t/=d/2) < 1) return c/2*t*t + b;
  return -c/2 * ((--t)*(t-2) - 1) + b;
}

/*
 * Initialize each Servo object in order using the values in servoLimits
 */
void initServos()
{
  for (int servo = 0; servo < servoCount; servo++)
  {
    servos[servo].attach(servo0Pin + servo, servoLimits[servo][0], servoLimits[servo][1]);
    if (programLength > 0)
    {
      // If there is a program move the each servo to the starting position
      servos[servo].write(program[0][servo]);
    }
    else
    {
      // If there is no program move to the middle of test movement
      servos[servo].write(testMoveStart); 
    }
  }
}

/*
 * Initialize the pot inputs
 */
void initPots()
{
  for (int i = 0; i < servoCount; i++)
  {
    pinMode(pot0Pin - i, INPUT);
  }
}

/*
 * Initialize the button inputs - these are default HIGH so a press is LOW
 */
void initButtons()
{
  pinMode(button4Pin, INPUT);
  pinMode(button7Pin, INPUT);
}

/*
 * Start point of whole program
 *  Loads the initial program (if required)
 *  Initializes inputs and servos
 *  Starts the serial communication for debugging and program recording
 */
void setup()
{
  // Load the program, optional, can be removed or commented out when not required
  loadInitialProgram();
  
  // Initialize the inputs and outputs
  initServos();
  initPots();
  initButtons();

  Serial.begin(9600);
  Serial.println("Robotic arm is ready, use button4 (start/step) and button7 (stop/drive/program).");
}

/*
 * You can paste the information returned in serial mode to enter a default program here.
 * Without calling this when in "Run" mode a sequence of test movements is performed instead.
 */
void loadInitialProgram()
{
  // Start of program
  program[0][0] = 1;
  program[0][1] = 177;
  program[0][2] = 0;
  program[0][3] = 15;
  program[0][4] = 50;
  program[1][0] = 56;
  program[1][1] = 155;
  program[1][2] = 51;
  program[1][3] = 65;
  program[1][4] = 50;
  program[2][0] = 175;
  program[2][1] = 0;
  program[2][2] = 3;
  program[2][3] = 64;
  program[2][4] = 50;
  programLength = 3;
  // End of program
}

// Stores the current "mode" of the program, these values are changed due to button presses.
bool driving = false;
bool running = false;
bool stepping = false;
bool nextStep = false;

/*
 * Main running loop of the program
 */
void loop() {
  if (running || (stepping && nextStep))
  {
    if (programLength == 0)
    {
      performTestMoves();
      delayButtons(200);
    }
    else
    {
      performProgram();
      nextStep = false;
      delayButtons(10);
    }
  }
  else if (driving) {
    readPots();
    driveServos();
    delayButtons(10);
  }
  else
  {
    // Not running or driving
    delayButtons(200);
  }

  // If the flag has been set indicating a button press, react to button and clear flag
  if (button4Pressed || button7Pressed)
  {
    if (button4Pressed)
    {
      if (driving)
      {
        Serial.println("Add line");
        addProgramLine();
      }
      else if (!running && !stepping)
      {
        driving = true;
        Serial.println("Driving");
      }
      else
      {
        running = false;
        stepping = false;
        Serial.println("Stopping");
      }
      button4Pressed = false;
    }
    if (button7Pressed)
    {
      driving = false;
      if (stepping)
      {
        nextStep = true;
        // in performServoNextProgramLine() the program is written out for you
      }
      else if (running && programLength > 0)
      {
        running = false;
        stepping = true;
        Serial.println("Stepping");
      }
      else
      {
        running = true;
        Serial.println("Running");
      }
      button7Pressed = false;
    }
    // Delay to help prevent double button presses, should debounce properly
    delay(100);
  }
}

/*
 * While waiting we go into a loop looking for button presses every 20ms
 */
void delayButtons(int duration)
{
  while (duration > 20)
  {
    delay(20);
    readButtons();
    duration -= 20;
  }
  delay(duration);
  readButtons();
}

/*
 * Read the button state only if the current state has not been cleared yet
 */
void readButtons()
{
  if (!button4Pressed)
  {
    button4Pressed = !digitalRead(button4Pin);
  }
  if (!button7Pressed)
  {
    button7Pressed = !digitalRead(button7Pin);
  }
}

/*
 * Read the pot values, in a loop to try to smooth out the values as they were very noisy.
 *  Uncommenting the Serial lines can be used for debugging issues with the values.
 */
void readPots()
{
  //Serial.println("Reading pots");
  for (int servo = 0; servo < servoCount; servo++)
  {
    // Smooth this out by taking a lot of readings an using an average, still pretty noisy though
    int sum = 0;
    int readings = 20;
    for (int j = 0; j < readings; j++)
    {
      sum = sum + analogRead(pot0Pin - servo);
      delay(2);
    }
    pots[servo] = (sum / 1024.0) / readings;
    //Serial.println(pots[i]);
  }
}

/*
 * Move the servos according to the pot values
 */
void driveServos()
{
  for (int servo = 0; servo < servoCount; servo++)
  {
    int target = 180 * pots[servo];
    if (abs(servos[servo].read() - target) > jitter)
    {
      moveServo(servos[servo], target, 10);
    }
  }
}

/*
 * Add a line to the program only when the angle values have changed by more than the jitter value
 */
void addProgramLine()
{
  if (programLength >= programMaximumSize)
  {
    Serial.println("Program too big!");
  }
  
  bool anyDifferent = false;
  for (int servo = 0; servo < servoCount; servo++)
  {
    // Put each value in the next program line, this will be unused if anyDifferent remains false
    program[programLength][servo] = servos[servo].read();

    // If there are no lines or the value is different enough from the previous value then anyDifferent is true
    if (programLength == 0 || abs(program[programLength - 1][servo] - program[programLength][servo]) > jitter)
    {
      anyDifferent = true;
    }
  }

  // Set the move duration to default - this should be calculated based on the servo speed factor
  program[programLength][servoCount] = defaultMoveDuration;
  
  if (anyDifferent)
  {
    Serial.print(programLength);
    Serial.print(":");
    // servoCount + 1 to include the duration
    for (int i = 0; i < servoCount + 1; i++)
    {
      Serial.print(" ");
      Serial.print(program[programLength][i]);
    }
    Serial.println("");
    programLength++;
  }
  else
  {
    Serial.println("No changes to pots, line ignored.");
  }
}

/*
 * Move servo to target with easing in duration number of steps
 */
void moveServo(Servo servo, int target, int duration)
{
  int start = servo.read();
  for (int pos = 0; pos < duration; pos++)
  {
    //move servo from start to angle degrees forward
    servo.write(easeInOutQuad(pos, start, target - start, duration));
    delayButtons(15);
  } 
}

/*
 * Move left, right, back to center each servo in turn
 */
int currentTestServo = 0;
void performTestMoves()
{
  int deflection = testMoveRange / 2;
  // move negative
  moveServo(servos[currentTestServo], testMoveStart - deflection, 25);
  // move positive
  moveServo(servos[currentTestServo], testMoveStart + deflection, 50);
  // move back to center
  moveServo(servos[currentTestServo], testMoveStart, 25);

  // Count eg 0,1,2,0,1,2 (when servoCount is 3)
  currentTestServo++;
  currentTestServo = currentTestServo % servoCount;
}

/*
 * Runs next line of the program, if the last line is reached it starts again from 0
 */
int currentProgramLine = 0;
void performProgram() {
  // Start of program
  if (stepping && currentProgramLine == 0){
    Serial.println("  // Start of program");
  }

  // We move all servos at once so record the starting position
  int start[servoCount];
  for (int servo = 0; servo < servoCount; servo++)
  {
    start[servo] = servos[servo].read();
    // Servo position for program output
    if (stepping) {
      char line[30];
      sprintf(line, "  program[%d][%d] = %d;", currentProgramLine, servo, program[currentProgramLine][servo]);
      Serial.println(line);
    }
  }

  if (stepping)
  {
    // Duration for program output
    char line[30];
    sprintf(line, "  program[%d][%d] = %d;", currentProgramLine, servoCount, program[currentProgramLine][servoCount]);
    Serial.println(line);

    // End of program
    if (currentProgramLine == programLength - 1)
    {
      char line[30];
      sprintf(line, "  programLength = %d;", programLength);
      Serial.println(line);
      Serial.println("  // End of program");
    }
  }

  // Ask all servos to move a small distance, pause, repeat, until all movement has hit target
  int duration = program[currentProgramLine][servoCount];
  for (int pos = 0; pos < duration; pos++)
  {
    for (int servo = 0; servo < servoCount; servo++)
    {
      servos[servo].write(easeInOutQuad(pos, start[servo], program[currentProgramLine][servo] - start[servo], duration));
    }
    delayButtons(15);
  }

  // Count the program lines eg 0,1,2,4 (where programLength is 4)
  currentProgramLine++;
  currentProgramLine = currentProgramLine % programLength;
}


/*
 * Project 1
 * Team G12
 * 
 * This file includes source code for Functionality 1 - obstacle avoidance,
 * Functionality 2 - line following, 
 * Functionality 3 - additional clap control functionality,
 * as well as shared helper functions and mode select functions.
 *
 */

 #include<Servo.h>              // include the servo library
 #include<LiquidCrystal.h>      // include LCD library

// GENERAL functionality Variables and Constatnts
// ------------------------------------------------------------------------- 

// MODE SELECT
#define FUNC1 1
#define FUNC2 2
#define FUNC3 3
#define FUNC4 4
#define BUTTONPIN 2  //hall effect sensor for wheel rotation menu scrolling

#define NINETY 275
#define ONE_EIGHTY 550

volatile int LCDcounter = 0;             // Counter to update the LCD every X loops where x is defined in the function
volatile int buttonState;                // the current reading from the input pin
volatile int lastButtonState = LOW;      // the previous reading from the input pin
int mode = 0;

// debounce variables used for mode select
// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


// FUNCTION 1 Variables and Constants
//------------------------------------------------------------------------------
#define SPEED_R 5               // Speed pin for motor1
#define SPEED_L 6               // Speed pin for motor2
#define DIRECTION_R 4           // Direction pin motor 1
#define DIRECTION_L 7           // Direction pin motor 2

#define HES_R 2                 // Hall effect sensor pin left
#define HES_L 3                 // Hall effect sensor pin right

#define ECHO 1                  // Ultrasonic sensor echo pin
#define TRIG A5                 // Ultrasonic sensor trigger pin
#define SERVO1 0                // Servo for ultrasonic sensor
#define THERM A0                // LM35 temperature sensor pin for accurate distance measurements with sonar
#define TOLERANCE 5

#define LCD_RS 8                // LCD pin assignments
#define LCD_E 9
#define LCD_DB4 10
#define LCD_DB5 11
#define LCD_DB6 12
#define LCD_DB7 13
                                //constants for direction correcting 
#define MAX_RANGE 1023          // Maximum range for the analogWrite() value for the motors
#define MIN_RANGE 980           // Minimum range for the analogWrite() value for the motors
#define VALUE_R 1023            // Maximum Speed
#define VALUE_L 1023            // Maximum Speed

Servo servo;                    // Servo Motor Object

float distance = 0;             // Global var to be displayed on LCD

volatile int rotationsR = 0;             // Rotation counter for the right wheel
volatile int rotationsL = 0;             // Rotation counter for the left wheel
volatile int magnetR    = false;         // Flag for right wheel
volatile int magnetL    = false;         // Flag for left wheel

volatile int driveCounter = 0;           // When distance > 40 will only check distance again every X loops where X is defined in THE FUNCTION
volatile int valueR       = 1023;        // Initially give right motor more power than left
volatile int valueL       = 1023;        // to compensate for left motor being faster.
volatile int lastR;
volatile int lastL;
              
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7); //initializes the lcd object 



// FUNCTION 2 Variables and Constants
//------------------------------------------------------------------------------
#define BLPIN A4                // back  left  sensor pin
#define FLPIN A3                // front left  sensor pin
#define BRPIN A1                // back  right sensor pin
#define FRPIN A2                // front right sensor pin
#define NUM_PINS 4              // number of sensor pins
#define DELAY 2                 // Minumum delay for each direction change
#define SENSORTHRESHOLD 700     // sensor reading threshold (what is considered light vs dark)
#define VALUE_L_2 950           // default speed of the left wheel for function 2
#define VALUE_R_2 965           // default speed if the right wheel for function 2

//The Order of the States are Back Left _ Front Left _ Front Right _ Back Right
int sensorPins[4] = {BLPIN, FLPIN, FRPIN, BRPIN};   // sensor pins array (to keep indexes between data & sensors constant)
int sensorData[4] = {0,0,0,0};                      // readings of the sensor data (1 if on black, 0 if on white)
int stateEncoding = 0;                              // encoding based on the current state (FSM)
int lastState = 0;                                  // encoding based on the last state (FSM)

// Combinational Logic for Function 2

// Function Wrappers for function pointers
void Straight() {moveStraight(DELAY, false);}
void Right() {rotateRight(DELAY, false);}
void HardRight() {rotateRight(DELAY, true);}
void Left() {rotateLeft(DELAY, false);}
void HardLeft() {rotateLeft(DELAY, true);}
void doNothing() {}

void (* StraightPTR) () = Straight; // Moves Straight
void (* RightPTR) () = Right; // Turns right
void (* HardRightPTR) () = HardRight; // Turns right on the spot
void (* LeftPTR) () = Left; // Turns Left
void (* HardLeftPTR) () = HardLeft; // Turns left on the spot
void (* NothingPTR)() = doNothing; // Keep doing last acition
void (*FSMLibrary[16])() = {NothingPTR, HardRightPTR, RightPTR, RightPTR, // State 00XX // XX are in order of 00, 01, 10, 11
                            LeftPTR, LeftPTR, StraightPTR, HardRightPTR,  // State 01XX
                            HardLeftPTR, NothingPTR, RightPTR, RightPTR,  // State 10XXk
                            LeftPTR, LeftPTR, HardLeftPTR, StraightPTR};  // State 11XX\
// The states directly translate from a binary encoding to a integer index in the function array
/*
 * Any symmetrical state is either remember last state or move straight
 * States 0000, 1001, 0110, 1111
 * Right sensors active indicates turning right, more active sensors means a hard right is required
 * States , 0001, 0010, 0011, 0111
 * The same is mirrored for the left
 * States , 1000, 0100, 1100, 1110
 * Fringe cases are imbalenced states with both sides active, these indicate intersections/imperfect paths
 * States, 0101, 1010, 1011, 1101
 */

/** 
 *  Strings to be displayed on LCD during functionality 2
 */
String stateArray[] = {
  "Thinking...", "Rotate Right",  "Rotate Right", "Rotate Right",
  "Rotate Left", "Rotate Left",   "Straight",     "Rotate Right",
  "Rotate Left", "Thinking...",   "Rotate Right", "Rotate Right",
  "Rotate Left", "Rotate Left",   "Rotate Left",  "Straight"
};



// FUNCTION 3 Variables and Constants
//------------------------------------------------------------------------------
#define MICROPHONE A5          // Microphone pin (sharing with trig which is OK)
#define CLAP_TIME 800          // Amount of milliseconds claps must occur within

int volatile state = 0;        // for microphone state
int volatile claps = 0;                 // To display number of claps on LCD


// FUNCTION 4 Variables and Constants
//------------------------------------------------------------------------------
#define XAXIS 6 
#define YAXIS 7 
#define SBUTTON 1

#define WEST 0
#define EAST 1
#define NORTH 2
#define SOUTH 3

#define J_DOWN_XY 0//00
#define J_UP_XY 3//11
#define J_LEFT_XY 1//01
#define J_RIGHT_XY 2//10



// GENERAL shared functions
// -----------------------------------------------------------------------------------

/**
 * This setup function allows the user to input the mode to run the robot on
 * When the mode is selected, the corresponding functions will be called
 */
void setup() {
    lcd.begin(16, 2);          // Initialize the LCD with 2 rows and 16 columns
    lcd.clear();               // clear the LCD
    mode = modeSelect();       //promts the user for a mode in the scroll menu 

    if (mode == FUNC1) { 
        pinMode(SPEED_R, OUTPUT);          // Speed pin for motor1
        pinMode(SPEED_L, OUTPUT);          // Speed pin for motor2
        pinMode(DIRECTION_R, OUTPUT);      // Direction pin motor 1
        pinMode(DIRECTION_L, OUTPUT);      // Direction pin motor 2
        pinMode(HES_R, INPUT);             // Hall effect sensor pin left
        pinMode(HES_L, INPUT);             // Hall effect sensor pin right
        pinMode(ECHO, INPUT);              // Ultrasonic sensor echo pin
        pinMode(TRIG, OUTPUT);             // Ultrasonic sensor trigger pin
        pinMode(SERVO1, OUTPUT);           // Servo pin
        pinMode(THERM, INPUT);             // LM35 temperature sensor pin
        servo.attach(SERVO1);              // attaches digital IO pin 6 to control the servo
        servo.write(90);                   // Orient sonar to point forward
        navigate();                        // Infinite loop for functionality 1
    }
    else if (mode == FUNC2) {              
        // initializes all pins to be inputs
        for (int i = 0; i < NUM_PINS; i++) {
            pinMode(sensorPins[i], INPUT);
        }
        followLine();                   
    }
    else if (mode == FUNC3) {
        pinMode(MICROPHONE, INPUT);
       functionalityThree();
    }
    else if (mode == FUNC4) {
        pinMode(SPEED_R, OUTPUT);          // Speed pin for motor1
        pinMode(SPEED_L, OUTPUT);          // Speed pin for motor2
        pinMode(DIRECTION_R, OUTPUT);      // Direction pin motor 1
        pinMode(DIRECTION_L, OUTPUT);  
        pinMode(XAXIS, INPUT);
        pinMode(YAXIS, INPUT);
        pinMode(SBUTTON, INPUT);
        pinMode(HES_R, INPUT);             // Hall effect sensor pin left
        pinMode(HES_L, INPUT);
        pinMode(1, INPUT);
         setDestination();
    }
    else {
        // Do nothing
    }
}

/** 
 *  modeSelect reads the button presses and prints the selection menu
 *  which changes based on how many times the button is pressed.
 *  @param: none
 *  @returns: integer - the mode selected by the user
 *  This function will wait until a valid state is selected (1-4)
 *  Once a state is selected, it will wait up to 3 seconds 
 *  (in case of adjustments)
 */
int modeSelect() {
    lcd.setCursor(0,0);     // set cursor to top left
    lcd.print("Mode select:");
    lcd.setCursor(0,1);     // Set cursor to bottom row
    lcd.print("1  2  3  4  ");

    int cycleTimer = 0;     // initializing the counter timer for mode select
    bool modeSelected = false;
    int selectedMode = 0;

    while(!modeSelected) {
        // read the input pin
        int reading = digitalRead(BUTTONPIN);
        // if the reading is different from last time, start counting how long the
        // button is pressed
        if(reading != lastButtonState) lastDebounceTime = millis();

        // if the time difference from the last switch is larger than the delay
        // (not noise)
        if((millis() - lastDebounceTime) > debounceDelay) {
            if(reading != buttonState) {
                buttonState = reading;
                if(reading == HIGH) {
                    // increment counter and reset to 0 if it is at 'mode 5'
                    selectedMode = ++selectedMode % 5; }
                    cycleTimer = 0;
            }
        }

        // For mode select
        // mode that the pointer is on will
        // be selected after 300ms
        if(cycleTimer == 300) {
            if(selectedMode == 0) {
                cycleTimer = 0;
            } else {
                modeSelected = true;
                break;
            }
        }

        // update display
        String displayValue = "1  2  3  4  ";

        if(selectedMode != 0) { 
            //printing the MENU
            displayValue = displayValue.substring(0, ((selectedMode - 1) * 3) + 1) + '<' 
            + displayValue.substring(((selectedMode-1) * 3) + 2, displayValue.length());   
        } else {
          displayValue = "1  2  3  4  ";
        }

        lcd.setCursor(0,1);
        lcd.print(displayValue);
        cycleTimer++;
        lastButtonState = reading;
        delay(1);
    }
    return selectedMode;
}

/**
 *  moveStraight() moves the robot straight based on the global speed values,
 *  checks the rotations of the wheels, and adjusts the speed of the wheels
 *  to keep the robot moving straight
 *  @param void
 *  @return void
 */
void moveStraight(int delayTime, bool control){
    digitalWrite(DIRECTION_R, LOW);     // Since the left and right motors are flipped in or,
    digitalWrite(DIRECTION_L, HIGH);    // directions will be opposite to move forwards or backwards
    analogWrite(SPEED_R, valueR);
    analogWrite(SPEED_L, valueL);

    // Wheel rotations are only controlled in 
    // functionalities 1 and 3
    if (control) {                      
        controlRotations();              //Adjusts speeds of wheels depending on hall sensor readings (always happening)
        if(mode == FUNC1){              
            if (distance>40){            //if we are over 40cm in distance we are not too concerned about hitting an object so we read distances less often to focus the processor power on going straight
                driveCounter++;          //increase the driveCounter
                if (driveCounter == 5){  //whenever the whole program has looped IGNORETIMES times 
                    drive();             //check the distance 
                    driveCounter = 0;    //reset the driveCounter
                }
            }
            else if (distance <= 40){   //otherwise if we are close to obstacles, we want to check every single time and not worry too much about moving straight
                drive();                //Move forward and check for obstacles every time program branches here
            }
            else{
              //do nothing
            }
        }
    }
    delay(delayTime);                  //we have a short delay
}

/*
 *   drive scans if there is an object if front of the robot
 *   if there is an object, stop the robot and scan left and right
 *   then rotate left or righ, determined by the side with most space
 *   @Param: void
 *   @Return: void 
*/
void drive(){
  
    distance = getDistance();                                                 //gets a measure of distance (this variable is global and stored outside)
    
    float distance2 = getDistance();                                          //gets a second measure of distance to compute a difference

    analogWrite(SPEED_R, lastR);
    analogWrite(SPEED_L, lastL);
    
    float difference = fabs(distance - distance2);                            //compute the absolute value of the difference in readings                              
  
    if ((distance2 <= 40) && (difference < TOLERANCE)){                       //if the most recent distance is less than 40cm and the difference in readings is within the tolerance
      
        if (distance2 > 7){                                                   //furthermore if the most recent distance is still greater than 7, make the speed a function of that distance
            valueR = 822 + distance2*4.023;                                   //our deduced formula is thoroughly explained in the report
            valueL = 810 + distance2*4.023;   
        
            analogWrite(SPEED_R, valueR); // PWM Speed Control                //write the new speeds to the wheels
            analogWrite(SPEED_L, valueL); // PWM Speed Control
        } 
        else {
            valueR = 0;                                                       //if the distance is smaller or equal to 7, STOP
            valueL = 0;
            analogWrite(SPEED_R, valueR); // PWM Speed Control              
            analogWrite(SPEED_L, valueL); // PWM Speed Control
      
            servo.write(180);                                                 //rotate the servo left  
            delay(500);                                                       //allowing enough time to be read properly without servo/movement noise     
            float d_left = getDistance();                                     //and read the left direction 
            //delay(1000);                                                      

            servo.write(0);                                                   // rotate the servo right
            delay(500);                                                       //allowing enough time to be read properly without servo/movement noise
            float d_right = getDistance();                                    //and read the left direction
            //delay(1000);

            //restoring starting movement values
            servo.write(90);       // rotate the servo to point back to the front
            valueL = MAX_RANGE;    // Restore motors to max speed 
            valueR = MAX_RANGE;    // Restore motors to max speed
            rotationsR = 0;        // Reset hall effect counters
            rotationsL = 0;        // Reset hall effect counters

            // based on the readings, decide which direction to move in
            if (d_left > d_right){
                // if the left side has more space, turn left
                rotateLeft(NINETY,true);
            } else {
                // if the right side has more space, turn right
                rotateRight(NINETY,true);
            }

            
            analogWrite(SPEED_R, 0);  // Stop robot after completing rotation
            analogWrite(SPEED_L, 0);  // Stop robot after completing rotation 
            delay(150);

          analogWrite(SPEED_R, valueR);
          analogWrite(SPEED_L, valueL);
          controlRotations();
          
        }
    }
}


/**
 *  controlRotations utilizes the rotation readings and adjusts the speed of the wheels
 *  to compensate for any adjustments required in case if one wheel is moving faster than the other
 */
void controlRotations(){
    checkRotationsRight();   // Functions to monitor hall right effect sensor
    checkRotationsLeft();    // Functions to monitor hall left effect sensor

    if(rotationsR == rotationsL){
      lastL = valueL;
      lastR = valueR;
    }
    if((valueL <= MIN_RANGE)||(valueR < MIN_RANGE)){
        valueR = 1023; // If rotations of left and right wheel are the same
        valueL = 1023; // do not change the rotations 
    }
    else if(rotationsR > rotationsL){
        valueL++;  // Speed up left wheel

        // Make sure voltages stay within valid range or else motor will stop
        if(valueL >= MAX_RANGE){
            valueL = MAX_RANGE;
        }
    }
    else if(rotationsR < rotationsL){
        valueL--;  // Slow down left wheel

        if(valueL < MIN_RANGE){
            valueL = MIN_RANGE;
        }
    }
}

/**
 *  rotateRight() rotates the robot clockwise in a spot
 *  by moving the left wheel forwards and right wheel backwards
 *  at the same speed
 *  @param time - the time in miliseconds to turn right for
 *  @return void
 */
void rotateRight(int ms, bool onSpot) {
    digitalWrite(DIRECTION_L, LOW);  // Rotate robot to left
    digitalWrite(DIRECTION_R, LOW);  
    analogWrite(SPEED_R, valueR);

    if (onSpot) {
       analogWrite(SPEED_L, valueL);  // Rotate both wheels (on the spot)
    }
    else {
        analogWrite(SPEED_L, 0);      // Rotate only one wheel
    }
    delay(ms);
}

/**
 *  rotateLeft() rotates the robot 90 degrees counterclockwise in a spot
 *  by moving the right wheel forwards and left wheel backwards
 *  at the same speed.
 *  @param time - the time in miliseconds to turn left for
 *  @return void
 */
void rotateLeft(int ms, bool onSpot) {
    digitalWrite(DIRECTION_L, HIGH);
    digitalWrite(DIRECTION_R, HIGH);
    analogWrite(SPEED_L, valueL);       //PWM Speed Control

    if (onSpot) {
        analogWrite(SPEED_R, valueR);   // Rotate both wheels
    }
    else {
        analogWrite(SPEED_R, 0);        // Rotate only one wheel
    }
    delay(ms);
}

/*
 *  checkRotationsRight() reads the Hall Effect sensor and
 *  increments the counter when the sensor reads a magnet
 */
void checkRotationsRight(){
    // If hall effect transitions from low to high, increment magnet count (rotationsR)
    if ((magnetR == true) && (digitalRead(HES_R) == HIGH)){
       magnetR = false; // Flag for debouncing purposes
       rotationsR++;    // Increment right wheel rotation counter to help callibrate both wheels
    }

    // If hall effect does not detect magnet, prepare to read next magnet
    if (digitalRead(HES_R) == LOW){
       magnetR = true;      // Flag for debouncing purposes
    }
}

/*
 *  checkRotationsLeft reads the Hall Effect sensor and
 *  increments the counter when the sensor reads a magnet
 *  This function is used to observe the differences in the right wheel speed
 */ 
void checkRotationsLeft(){
    if ((magnetL == true) && (digitalRead(HES_L) == HIGH)){
        magnetL = false;
        rotationsL++;   // Increment left wheel rotation counter to help callibrate both wheels
    }
    if (digitalRead(HES_L) == LOW){
        magnetL = true; // Flag for debouncing purposes
    }

}


/*
* Unused main loop function
*/
void loop() {}





// Functionality1 (Obstascle avoidance) functions 
// ----------------------------------------------------------------------------------------

/**
*   Function 1 - Obstacle avoidance loop
*   @param: void
*   @return: void
*/
void navigate() {
    while(true) {
        moveStraight(0, true);      // Make the robot move in a straight line
        LCDcounter++;               // Only display information every 
        if (LCDcounter % 5 == 0){   // five cycles to make LCD more readable (less clearing)
           printFunction1Status();  // Display info on LCD
        }
    }
}

/*
 *  getDistance() scans the distance based on the sonar sensor and the ambient temperature
 *  @return float -- the distance (in centimeters) in the sonar diretion
 */ 
float getDistance(){
    //gets the speed of sound given the temperature
    float localdistance;    
    float speedOfSound = 331.5 + (0.6*getTemperature());
    
    analogWrite(TRIG, 1023);                    // writes on the trigger
    delayMicroseconds(10);                      // waits 10 microseconds
    analogWrite(TRIG, 0);                       // turns of trigger
    float pulseWidth = pulseIn(ECHO, HIGH);     // reads the arriving pulse

    // computes the values of the most current distances given the
    // width of the pulses sent, the result is the distance in centimeters
    localdistance = (pulseWidth/2)*((speedOfSound*100)/1000000); // cm = microseconds * cm/microsecond
    return localdistance;
}

/*
 *  getTemperature() returns the thermal reading from the analogue temperature sensor
 *  @return int -- the current temperature reading
 */
int getTemperature(){
    // 10 degrees Celcius per milivolt as in datasheet,
    // analogRead return is mapped from 0-1024 and voltage is between 0-5000 mV
    return (analogRead(THERM)/1024.0*5000)/10.0;
}

/**
 * printFunction1Status displays on the LCD: 
 * the mode, power delivered to each wheel
 * as well as distance measured by the ultrasonic sensor
 */
void printFunction1Status() {
    lcd.clear();
    lcd.setCursor(0,0);     // set cursor to top left
    String topString = "M:1 L:";
    topString += (valueL / 4);
    topString += (" R:");
    topString += (valueR / 4);
    String botString = "Dist: ";
    botString += (distance);
    lcd.print(topString);
    lcd.setCursor(0,1);
    lcd.print(botString);
}




// Functionality2 (Line following) functions 
// ----------------------------------------------------------------------------------------

/**
*   Function 2 - Line following loop
*   @param: void
*   @return: void
*/
void followLine() {
    valueL = VALUE_L_2;            // Initialize speeds of wheels
    valueR = VALUE_R_2;
    while(true) {
        readSensors();             // Read and store sensor data
        decideLogic();             // change state based on the current sensor.
        printFunction2Status();    // Display info on LCD
    }
}

/**
*   decideLogic sets the speeds of the motors based on the current and previous sensor readings
*   @param: void
*   @return: void
*/
void decideLogic() {
    lastState = stateEncoding; // store the last state for logic decisions

    // Convert the 4bit data into an integer
    int stateEncoding = 8*sensorData[0] + 4*sensorData[1] + 2*sensorData[2] + sensorData[3];
    (*FSMLibrary[stateEncoding])(); // Pull a function from the array, and excecute it
}

/**
*   Updates the sensorData array based on the current readings
*   @Param: void
*   @Return: void
*/
void readSensors() {
    for(int i = 0; i < NUM_PINS; i++) {
        if(analogRead(sensorPins[i]) > SENSORTHRESHOLD) {
            sensorData[i] = 1;  //Set sensor flags for functionality 2
        } else {
            sensorData[i] = 0;
        }
    }
}

/**
 * printFunction2Status displays the state of the robot's turning
 * on the LCD during functionality 2
 */
void printFunction2Status() {
    lcd.clear();
    lcd.setCursor(0,0);
    String topString = "Mode:2";
    lcd.print(topString);
    String botString = stateArray[lastState];
    lcd.setCursor(0,1);
    lcd.print(botString);
}


// Functionality3 (Clap commands) functions
// ----------------------------------------------------------------------------------------

/**
 *  Function 3 - Clap Control loop
 *  @param: void
 *  @return: void
 */
void functionalityThree(){
   while(true){
            clapControl();                 // Control robot based on number of claps
            delay(250);                    // for robot to spin freely with enough time for it not to trigger the microphone
            printClaps();                  // Display number of claps on LCD
            getNextAction(getClaps());     // Poll for more claps
            claps = 0;                     // Reset claps to zero
        } 
}

/**  
 *  getClaps returns the number of claps done consecutively by the human
 *  @param: none
 *  @returns: claps - where each clap was performed not more than one second after the previous
 *  The function will return 4 immidiately after 4 consecutive claps (where claps are less than one second apart)
 *  Otherwise it will return the number of claps once no claps are detected in 1 second
 */
int getClaps(){
    //these are not the initial values but rather the values after the first clap TODO: Is this comment correct?
    int ms = 0;      

    // a "polling" while loop
    while(digitalRead(MICROPHONE) != LOW){              
        //wait for first clap
    }

    claps++;
    delay(50);         //de-bouncing delay
    ms = 50;
    printClaps();
    while ((claps < 3) && (ms < CLAP_TIME)){
        //repeatedly increments the time counter and checks the condition while no claps are heard
        while(digitalRead(MICROPHONE) != LOW){           
            delay(1);        
            if (ms >= CLAP_TIME){ 
                //if a second passed and no claps are detected, return 1 clap
                return claps;                               
            }
            else{
                ms++;
            }
       }
       claps++;
       printClaps();    // Display number of claps on LCD
       delay(50);
       ms = 50;
    }
    return claps;
}

/**
 *  getNextAction(int) takes the number of claps and decides the logic
 *  of the next action
 *  @param: int - claps: the number of claps that were detected
 *  @returns: void
 *  Sets the state and the wheel speeds to the corresponding values
 */
void getNextAction(int claps){
    valueL = MAX_RANGE; // initialize speed of wheels to max  
    valueR = MAX_RANGE; // initialize speed of wheels to max

    switch(claps){
        // Start or stop
        case 1: state = !state; break; 

        // Turn right
        case 2: rotateRight(NINETY, true); break;

        // Turn left
        case 3: rotateLeft(NINETY, true); break;
    }
}

/**
 *  clapControl() controls the robot and excecutes the commands
 *  of the next action
 *  @param: void
 *  @returns: void
 *  Utilizes the current state and excecutes the command
 */
void clapControl(){
    if (state == 1){
        moveStraight(0, true);
    }
    else{
        digitalWrite(DIRECTION_R, LOW);   // Since the left and right motors are flipped in or,
        digitalWrite(DIRECTION_L, HIGH);  // directions will be opposite to move forwards or backwards
        analogWrite(SPEED_R, 0);  // Set wheel speed to 0 before next step   
        analogWrite(SPEED_L, 0);  // Set wheel speed to 0 before next step
    }
}

/**
 * printClaps is a helper function to display 
 * the number of claps detected during functionality 3
 */
void printClaps(){
    lcd.clear();
    printFunction3Status();
    lcd.setCursor(0,1);
    lcd.print("Claps: ");
    lcd.print(claps);
}

/** 
 * printSecondRow moves the cursor to the second row 
 */
void printSecondRow(String str) {
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print(str);
}


/**
 * printFunction3Status displays the number of claps detected
 * on the LCD during functionality 3
 */
void printFunction3Status() {
    lcd.setCursor(0,0);
    String topString = "Mode:3";
    lcd.print(topString);
}




// Functionality4 (Path programming) functions 
// ----------------------------------------------------------------------------------------

/**
 * setDestination allows users to program a path onto the robot by providing compass coordinates
 * and distances to move in the given directions. Users are able to scroll through N,E,S,W and durations from 0-9 seconds
 * by spinning the wheels. Directions and durations are entered and stored in memory using a pushbutton.
 * If the user enters two identical directions consecutively, then the programming stage will exit 
 * and the robot will prepare to execute the path programmed by the user. 
 *
 */
void setDestination(){
  while (true){
    //prompt user with instrutions
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Use Wheels for");
      lcd.setCursor(0,1);
      lcd.print("Dir/Time");
      delay(2000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Button to");
      lcd.setCursor(0,1);
      lcd.print("confirm");
      delay(2000);
       
      int moves[16] = {0};          // Array for storing directions
      int distances[16] = {0};      // Array for storing distances
      int i = 0;                    // Counter for iterating through arrays 
      int distance = 0;             
      int buttonState = 0;          // State for enter button
      int orientation = NORTH;      // Orientation of robot to ensure moving in correct compass directions
      int state = 0;

      // State storage for functionality 4
      int previousRight = 0;
      int previousLeft = 0;
      int currentRight = 0;
      int currentLeft = 0;
      int direction = 0; 
      int travelDist = 0;

      bool Set = false; // flag used to enter directions and distances
      bool End = false; // flag for end of programming stage

      // Clear the lcd after displaying the menu
      lcd.clear();
      //While we have stored less than 17 elements since LCD can only display 16 characters
      while(i < 16) {
        Set = false; //reset set flag to take in next direction and distance
        while (!Set) {
          // Storing previous states in order to detect repeated entries
          currentRight = digitalRead(HES_R);
          currentLeft = digitalRead(HES_L);

          // Scroll through options (N,E,S,W)
          if (currentRight != previousRight && currentRight == 1) {
            direction++;
            direction = direction % 4;
          }

          // Scroll through distance options 0-9
          if (currentLeft != previousLeft && currentLeft == 1) {
            travelDist++;
            travelDist = travelDist % 10;
          }

          // When the button is pressed, lock in directions and durations and move cursor
          while (digitalRead(SBUTTON) == HIGH) {
            Set = true;
            delay(500);
            if (i > 0 && i < 16 && direction == moves[i-1]) {
              End = true;
            }
          }

          lcd.setCursor(i, 0); // Move cursor one spot to the right to display directions on top row
          switch(direction){
          case 0: lcd.print("W");break;
          case 1: lcd.print("E");break;
          case 2: lcd.print("N");break;
          case 3: lcd.print("S");break;
          default: lcd.print("*"); break;
          }

          // Display travel duration/distance on bottom row
          lcd.setCursor(i, 1);
          lcd.print(travelDist);

          // Set current values as previous values for later comparison
          previousRight = currentRight;
          previousLeft = currentLeft;
        }

        // Store direction and travel duration in array to be read later
        moves[i] = direction;
        distances[i] = travelDist * 1000;
        i++;

        // End of programming stage, move onto execution stage
        if (End) {
          break;
        }
    }

    // Give time to allow user to place robot on the floor
    delay(500);

    // Executing the program. The robot moves based on the compass coordinate directions relative to the user
    // NOT relative to the robot, this creates a more user friendly interface since the user can program the path
    // based on their own stationary perspective of NEWS
    for (int i = 0; i < 16; i++) {
      // Moving west
      if (moves[i] == WEST) {  
        if (orientation == NORTH) {         // If robot is facing north
          rotateLeft(NINETY, true);         // rotate 90 degress counter clockwise (ccw) to face west
          moveTime(distances[i]);           // move forwards by the amount of time entered by user
        }   
        else if (orientation == SOUTH) {    // if robot is facing south
          rotateRight(NINETY, true);        // rotate clockwise (cw) to face west
          moveTime(distances[i]);           // move forwards
        }
        else if (orientation == EAST) {     // if robot is facing east
          rotateRight(ONE_EIGHTY, true);    // turn robot around to face west
          moveTime(distances[i]);           // move forwards
        }
        else 
        {                                   // if robot is facing west
          moveTime(distances[i]);           // does not need to rotate, just move forwards
        }
        orientation = WEST;                 // Robot is now facing west
      }

      // Moving east
      else if (moves[i] == EAST) {           
        if (orientation == NORTH) {         // If robot is facing north,
          rotateRight(NINETY, true);        // rotate 90 degrees cw to face east
          moveTime(distances[i]);           // move forwards
        } 
        else if (orientation == SOUTH) {    // If robot is facing south
          rotateLeft(NINETY, true);         // rotate 90 degrees ccw to face east
          moveTime(distances[i]);           // move forwards
        }
        else if (orientation == WEST) {     // If robot is facing west
          rotateRight(ONE_EIGHTY, true);    // turn around to face east
          moveTime(distances[i]);           // move forwards
        }
        else 
        {                                   // if robot is already facing east
          moveTime(distances[i]);           // move forwards
        }
        orientation = EAST;                 // Robot is now facing east
      }

      // Moving north
      else if (moves[i] == NORTH) {
        if (orientation == EAST) {          // If robot is facing east
          rotateLeft(NINETY, true);         // rotate 90 degrees ccw to face north
          moveTime(distances[i]);           // move forwards
        } 
        else if (orientation == SOUTH) {    // if robot is facing south
          rotateLeft(ONE_EIGHTY, true);     // turn around to face north
          moveTime(distances[i]);           // move forwards
        }
        else if (orientation == WEST) {     // if robot is facing west
          rotateRight(NINETY, true);        // rotate 90 degress cw to face north
          moveTime(distances[i]);           // move forwards
        }
        else 
        {                                   // if robot is facing north already
          moveTime(distances[i]);           // move forwards
        }
        orientation = NORTH;                // robot is now moving forwards 
      }

      // Moving south
      else if (moves[i] == SOUTH) {
        if (orientation == NORTH) {         // if robot is facing north
          rotateRight(ONE_EIGHTY, true);    // turn around to face south
          moveTime(distances[i]);           // move forwards
        } 
        else if (orientation == WEST) {     // if robot is facing west
          rotateLeft(NINETY, true);         // rotate 90 degrees ccw to face south
          moveTime(distances[i]);           // move forwards
        }
        else if (orientation == EAST) {     // if robot is facing east
          rotateRight(NINETY, true);        // rotate 90 degrees cw to face south
          moveTime(distances[i]);           // move forwards
        }
        else 
        {                                   // if robot is facing south already
          moveTime(distances[i]);           // move forwards
        }
        orientation = SOUTH;
      }
      else {
        // Do nothing
      }
      // Stop
      analogWrite(SPEED_R, 0);
      analogWrite(SPEED_L, 0);
    }
  }
}

/**
 * moveTime -- Keeps calling move straight for the amount of time being passed to it
 * @param ms amount of time in milliseconds to moveStraight
 */
void moveTime(int ms) {
  unsigned long currentTime = millis();
  while (millis() - currentTime < ms) {
    moveStraight(0, true);
  }
}

/**
 * getTime displays the amount of time the robot will move in a certain direction
 * @param LcdIndex where to move the cursor to display on the LCD
 */
int getTime(int lcdIndex){
    int time = 0;
    while (true){
      lcd.setCursor(lcdIndex,1);      //we want to be overwriting the current index
      lcd.print(time);
    }
}


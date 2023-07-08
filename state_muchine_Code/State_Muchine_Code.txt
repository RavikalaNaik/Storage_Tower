#include <Adafruit_NeoPixel.h>
#include <Keypad.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define LED_PIN 23 //GPIO 15 ---Pin number 23 on the ESP32 Dev Kit -prs -30pin IC
#define LED_STRIP_PIN 19 //GPIO 19 ---Pin number 31 on the ESP32 Dev Kit
#define NUM_LED 60
#define CABIN_LED_OUT 23 //GPIO 23 ---Pin Number 37 
#define BUZZER_OUT 5 //GPIO 5 -->Pin number 29
#define DOOR_SWITCH_IN 18 //GPIO 18 --> Pin number 30
#define DOOR_SOLENOID_LATCH_OUT 15 // GPIO 21 --> Pin number 33
#define KEYPAD_C1_OUT 27 //GPIO32 -->Pin number 7
#define KEYPAD_C2_OUT 14 //GPIO33 -->Pin number 8
#define KEYPAD_C3_OUT 13 //GPIO25 -->Pin number 9
#define KEYPAD_R1_IN  32 //GPIO26 -->Pin number 10
#define KEYPAD_R2_IN  33 //GPIO27 -->Pin number 11
#define KEYPAD_R3_IN  25 //GPIO14 -->Pin number 12
#define KEYPAD_R4_IN  26 //GPIO13 -->Pin number 14

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LED, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);
 
const byte ROWS = 4; // four rows
const byte COLS = 4; // three columns

char hexaKeys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[ROWS] = {KEYPAD_R1_IN, KEYPAD_R2_IN, KEYPAD_R3_IN, KEYPAD_R4_IN}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {KEYPAD_C1_OUT, KEYPAD_C2_OUT, KEYPAD_C3_OUT};    // connect to the column pinouts of the keypad

Keypad Stkeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS );


//Different states of state machine
enum State {IDLE_1, ACTIVE, STORE_RETRIVE,WARNING};
State currentState = IDLE_1;

// Set the timer interval in microseconds
const unsigned long TIMER_INTERVAL = 1000000;

// Variable to count the number of timer interrupts
volatile unsigned long timerCounter = 0;

// Timer interrupt service routine
void IRAM_ATTR onTimer() {
  // Increment the timer counter
  timerCounter++;
}

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(CABIN_LED_OUT, OUTPUT);
  pinMode(BUZZER_OUT, OUTPUT);
  pinMode(DOOR_SOLENOID_LATCH_OUT, OUTPUT);  
  pinMode(DOOR_SWITCH_IN, INPUT);  
  pinMode(DOOR_SWITCH_IN, INPUT_PULLDOWN);

    // Initialize serial communication
  Serial.begin(115200);

  // Create an instance of hw_timer_t
  hw_timer_t *timer = NULL;

  // Allocate the timer
  timer = timerBegin(0, 80, true);

  // Set the timer period
  timerAlarmWrite(timer, TIMER_INTERVAL, true);

  // Attach the ISR function to the timer
  timerAttachInterrupt(timer, &onTimer, true);

  // Enable the timer interrupt
  timerAlarmEnable(timer);
}
  
  char password[] = "1234";
  char enteredPassword[5];
  int string_matched=0;

void loop()
 {
   char key = Stkeypad.getKey();
   int reading = digitalRead(DOOR_SWITCH_IN);
  switch(currentState) 
  {   // Use a switch statement to handle the different states of the state machine
    case IDLE_1:
      // Wait for messages received in com port and Read com port
       //if message received then go to active mode
      
      if (key != '*') 
      {
        Serial.println(key);
        currentState = ACTIVE;
      }
     
      break;
    case ACTIVE:   
      //If OTP matches then go to STORE_RETRIVE mode otherwise go to WARNIG mode
      if (key != '\0') 
       {
          Serial.println(key);
          int password_length=strlen(enteredPassword);
          enteredPassword[password_length]=key;
          enteredPassword[password_length+1]='\0';

          if(strcmp(enteredPassword,password)==0)
          {
          string_matched=1;
          currentState = STORE_RETRIVE;
          }
          else
          {
            currentState = WARNING;
          }
         
        }
      //If No actions go to IDLE after one min
      else
      {
        delay(3000);
        currentState = IDLE_1;
      }

      break;
    case STORE_RETRIVE: 
      //If new customer then Store, If Existing customer then Retrive 
      //when door closed go to IDLE State, If Door not Closed within 3min then go to Warning
      reading = digitalRead(DOOR_SWITCH_IN);
      if (reading == 1)
      {
       digitalWrite(CABIN_LED_OUT, HIGH);
       digitalWrite(DOOR_SOLENOID_LATCH_OUT, HIGH);
       digitalWrite(BUZZER_OUT, HIGH);
      }
      reading = digitalRead(DOOR_SWITCH_IN);
      if (reading == 0) 
      {
        digitalWrite(CABIN_LED_OUT, LOW);
        digitalWrite(DOOR_SOLENOID_LATCH_OUT, LOW);
        digitalWrite(BUZZER_OUT, LOW); 
        currentState = IDLE_1;
      }
      else
      {
        if (timerCounter >= 1000000) 
        {
           Serial.println(timerCounter);
           currentState = WARNING;
           timerCounter = 0;
        }

      }
      
      break;
      case WARNING:
        // Display Warnings: OTP not matches, Door open TIME OUT
        Serial.println("Something Went wrong please check");
        currentState = IDLE_1;
    default:
      //Unknown State go to IDLE
      currentState = IDLE_1;
      break;
  }
}





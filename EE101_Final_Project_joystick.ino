#include <SoftwareSerial.h> //BLE library
SoftwareSerial mySerial(6,7);  //BLE TX is on pin2 and RX on pin 
const int BuzzPin = 0; //buzzer pin
const int VRx = 0; //analogpins for joystick
const int VRy = 1; //^
const int sw = 8;  //^^
const int buttonPin = 12;  // the pin that the pushbutton is attached to
const int inputPin = 13;               // choose the input pin (for PIR sensor)
const int solenoid = 10; //solenoid pin
const int servopin=11;  //servopin
// Variables will change:
int servotimer = 50;        //time to move servo to new position
int buttonState = 0;        // current state of the button
int lastButtonState = 0;        // last state of the button
int switchState = 0;           //current state of joystick switch
int lastSwitchState = 0;      //last state of joystick switch
int buzztimer = 1000;       //amount of time the buzzer will be on for in ms
int val = 0;                    // variable for reading the motion sensor status
int pirState = LOW;             // we start, assuming no motion detected
int analogJoy = 0;           //analog joystick input
bool joyflag = false;        //flag to open/close programming loop

//***************4x4*******************************************************************************************************
const int latchPin1 = 4;	// Latch pin of 74HC595 is connected to Digital pin 5
const int clockPin1 = 5;	// Clock pin of 74HC595 is connected to Digital pin 6
const int dataPin1 = 3;	// Data pin of 74HC595 is connected to Digital pin 4

const int latchPin2 = 1;	// Latch pin of 74HC595 is connected to Digital pin 1
const int clockPin2 = 2;	// Clock pin of 74HC595 is connected to Digital pin 2
const int dataPin2 = 0;	// Data pin of 74HC595 is connected to Digital pin 0

byte ledcol = 0;		// Variable to hold the pattern of which LEDs are currently turned on or off
byte ledrow = 0;
byte ledcol1 = 0b00001000; //bytes that determine what leds in each column are turned on - only the last 4 digits are connected
byte ledcol2 = 0b00000100;
byte ledcol3 = 0b00000010;
byte ledcol4 = 0b00000001;

/*
 * setup() - this function runs once when you turn your Arduino on
 * We initialize the serial connection with the computer
 */
void setup() 
{
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  //initialize motion sensor pin as input
  pinMode(inputPin, INPUT);
  //initialise solenoid pin as output
  pinMode(solenoid, OUTPUT);
  //initialize servo pin as output
  pinMode(servopin, OUTPUT);
  // Set all the pins of 74HC595 as OUTPUT
  pinMode(latchPin1, OUTPUT);
  pinMode(dataPin1, OUTPUT);  
  pinMode(clockPin1, OUTPUT);
  pinMode(latchPin2, OUTPUT);
  pinMode(dataPin2, OUTPUT);  
  pinMode(clockPin2, OUTPUT);
  
  val = digitalRead(inputPin);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
    if (pirState == LOW) {
      // we have just turned on
      mySerial.begin(9600);   
      delay(100);
      mySerial.write("AT\r\n");  //put HM10 in AT command mode
      delay(100);
      mySerial.write("AT+NAME ERBLE\r\n");  //Name our HM10 something so as to not interfere with others
      delay(100);
      mySerial.write("AT+NAME\r\n");  //Verify new name       delay(100);
      mySerial.write("AT+RESET\r\n");  //reset HM10 so new name will take effect
      mySerial.println("Motion detected!");
      mySerial.println("Loading the last route that was entered");
      //EEPROM retrieval - load the route for 5 seconds
      mySerial.println("Would you like to program a new route, load a route remotely or use the current route?");
      mySerial.println("Enter 'p' to program a new route, 'l' to load a route, or 'c' to keep the current route");
      char input = cGetCharFromPhone();
      switch (input){
        case 'p':
            bool joyflag = true;
            mySerial.println("Entering Programming mode")
            mySerial.println("Use the joystick to navigate the board - press in to select a hold");
            mySerial.println("Press in twice when you are done selecting holds");
          break;
        
        case 'l':
          
          break;
        
        case 'c':
          mySerial.println("Keeping the same route");
          break;
        
        default:
          mySerial.println("The input entered is not recognized, please reset the program and try again");
          break;
        
      }
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } 
  else {
    if (pirState == HIGH){
      // we have just turned of
      Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }
}





/*
 * loop() - this function runs over and over again
 */
void loop() 
{
  if (joyflag == true){
    analogJoy = analogRead(VRx);	// read X axis value [0..1023]
    mySerial.print("X:");
    mySerial.println(analogJoy, DEC);
    analogJoy = analogRead(VRy);	// read Y axis value [0..1023]
    mySerial.print("Y:");
    mySerial.println(analogJoy, DEC);
    delay(100);
    switchState = digitalRead(sw);
  }


    
  //   // compare the switchState to its previous state
  //   if (switchState != lastSwitchState) {
  //     // if the state has changed, increment the counter
  //     if (switchState == HIGH) {
  //       // if the current state is HIGH then the button went from off to on:
  //       } 
  //     else {
  //       // if the current state is LOW then the button went from on to off: Turn off buzzer
  //       digitalWrite(BuzzPin, LOW);
  //     }
  //   }
  //   // save the current state as the last state, for next time through the loop
  //   lastButtonState = buttonState; 
  // }


//This will turn on the correct LEDs
  ledcol = 0;
  ledrow = 0;	// Initially turns all the LEDs off, by giving the variable 'leds' the value 0
    for (int i = 0; i < 4; i++)	// Turn all the LEDs ON one by one.
  {
    ledcol = 0;
    ledrow = 0;
    bitSet(ledrow, i);		// Set the bit that controls that LED in the variable 'leds'
    updateShiftRegister();

    switch(i){
      case 0:
        ledcol = ledcol1;
      case 1:
        ledcol = ledcol2;
      case 2:
        ledcol=ledcol3;
      case 3:
        ledcol=ledcol4;
    }

    updateShiftRegister();
    delay(1);
  }
  
//   // compare the buttonState to its previous state
//   if (buttonState != lastButtonState) {
//     // if the state has changed, increment the counter
//     if (buttonState == HIGH) {
//       mySerial.println("Congratulations on finishing the climb!");
//       // if the current state is HIGH then the button went from off to on:
//       for (int i; i <= buzztimer; i++){
//         digitalWrite(BuzzPin, HIGH);
//         delay(1);
//         digitalWrite(BuzzPin, LOW);
//       }
//     } 
//     else {
//       // if the current state is LOW then the button went from on to off: Turn off buzzer
//       digitalWrite(BuzzPin, LOW);
//     }
//   }
//   // save the current state as the last state, for next time through the loop
//   lastButtonState = buttonState;
   }

void updateShiftRegister(){

   digitalWrite(latchPin2, LOW);
   shiftOut(dataPin2, clockPin2, LSBFIRST, ledcol);
   digitalWrite(latchPin2, HIGH);

   digitalWrite(latchPin1, LOW);
   shiftOut(dataPin1, clockPin1, LSBFIRST, ledrow);
   digitalWrite(latchPin1, HIGH);  
}

//function cgetcharfromphone - inputs:none returns:char
char cGetCharFromPhone(){
  int IInByte;                            //declare local variable
  while (mySerial.available() == 0) {}     //wait for data available
  IInByte = mySerial.read();               //read character
  return(char(IInByte));
}
//function wallangle - inputs:none returns:none
void wallAngle(){
  mySerial.println("Would you like the wall at an angle? enter 'y' for yes and 'n' for no");
  char angle = cGetCharFromPhone();
  switch (angle){
    case 'y':
      //fire solenoid
      digitalWrite(solenoid, HIGH);
      //change servo angle
      for (int i=0; i <= servotimer; i++){
        digitalWrite(servopin,HIGH);
        delay(2);
        digitalWrite(servopin,LOW);
        delay(18);  
      }
      break;
    
    case 'n':
      //turn off solenoid
      digitalWrite(solenoid, HIGH);
      //reset servo angle
      for (int i=0; i <= servotimer; i++){
        digitalWrite(servopin,HIGH);
        delay(1.5);
        digitalWrite(servopin,LOW);
        delay(18.5);  
      }
      break;
    
    default:
      mySerial.println("The input entered is not recognized, please reset the program and try again");
      break;
    
  }
}

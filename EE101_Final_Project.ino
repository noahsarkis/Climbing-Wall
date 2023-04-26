//***************4x4*******************************************************************************************************
int latchPin1 = 4;	// Latch pin of 74HC595 is connected to Digital pin 5
int clockPin1 = 5;	// Clock pin of 74HC595 is connected to Digital pin 6
int dataPin1 = 3;	// Data pin of 74HC595 is connected to Digital pin 4

int latchPin2 = 1;	// Latch pin of 74HC595 is connected to Digital pin 1
int clockPin2 = 2;	// Clock pin of 74HC595 is connected to Digital pin 2
int dataPin2 = 0;	// Data pin of 74HC595 is connected to Digital pin 0

byte ledcol = 0;		// Variable to hold the pattern of which LEDs are currently turned on or off
byte ledrow = 0;
byte ledcol1 = 0b00001010;
byte ledcol2 = 0b00001001;
byte ledcol3 = 0b00000100;
byte ledcol4 = 0b00000011;

/*
 * setup() - this function runs once when you turn your Arduino on
 * We initialize the serial connection with the computer
 */
void setup() 
{
  

  // Set all the pins of 74HC595 as OUTPUT
  pinMode(latchPin1, OUTPUT);
  pinMode(dataPin1, OUTPUT);  
  pinMode(clockPin1, OUTPUT);
  pinMode(latchPin2, OUTPUT);
  pinMode(dataPin2, OUTPUT);  
  pinMode(clockPin2, OUTPUT);
}



/*
 * loop() - this function runs over and over again
 */
void loop() 
{
  ledcol = 0;
  ledrow = 0;	// Initially turns all the LEDs off, by giving the variable 'leds' the value 0

  updateShiftRegister();
    for (int i = 0; i < 4; i++)	// Turn all the LEDs ON one by one.
  {
    bitSet(ledrow, i);		// Set the bit that controls that LED in the variable 'leds'
    updateShiftRegister();
    if(i==0) {
      ledcol=0;
      bitSet(ledcol, 0) ;  
      updateShiftRegister();
      delay(1);
    }else if (i==1) {
      ledcol=0;
      bitSet(ledcol, 1) ;
      updateShiftRegister();
      delay(1);
    }else if (i==2) {
      ledcol=0;
      bitSet(ledcol, 2) ;
      updateShiftRegister();
      delay(1);
    }else {
      ledcol=0;
      bitSet(ledcol, 3) ;
      updateShiftRegister();
      delay(1);
    }
    delay(1);
    ledrow = 0;
  }
  
  // bitSet(ledcol,0);
  // updateShiftRegister();
  // delay(1);
  // ledrow = ledrow1;
  // updateShiftRegister();
  // ledcol = 0;
  // delay(1);
  
  // bitSet(ledcol,1);
  // updateShiftRegister();
  // delay(1);
  // ledrow = ledrow2;
  // updateShiftRegister();
  // ledcol = 0;
  // delay(1);

  // bitSet(ledcol,2);
  // updateShiftRegister();
  // delay(1);
  // ledrow = ledrow3;
  // updateShiftRegister();
  // ledcol = 0;
  // delay(1);

  // bitSet(ledcol,3);
  // updateShiftRegister();
  // delay(1);
  // ledrow = ledrow4;
  // updateShiftRegister();
  // ledcol = 0;
  // delay(1);

}
/*
 * updateShiftRegister() - This function sets the latchPin to low, then calls the Arduino function 'shiftOut' to shift out contents of variable 'leds' in the shift register before putting the 'latchPin' high again.
 */
void updateShiftRegister()
{

   digitalWrite(latchPin2, LOW);
   shiftOut(dataPin2, clockPin2, LSBFIRST, ledcol);
   digitalWrite(latchPin2, HIGH);

   digitalWrite(latchPin1, LOW);
   shiftOut(dataPin1, clockPin1, LSBFIRST, ledrow);
   digitalWrite(latchPin1, HIGH);  
}
//*********BLE+Joystick+Buzzer****************************************************************************************************************************
//modified from example code found on KY-023 DUAL AXIS JOYSTICK MODULE website - https://arduinomodules.info/ky-023-joystick-dual-axis-module/
#include <SoftwareSerial.h>
SoftwareSerial mySerial(6,7);  //TX is on pin2 and RX on pin 
//buzzer pin
const int BuzzPin = 0;
//analogpins for joystick
const int VRx = 0;
const int VRy = 1;
const int sw = 8;
int direction; //initialize joystick input
void setup(){
  mySerial.begin(9600);   
  Serial.begin(9600);   
  delay(100);

  mySerial.write("AT\r\n");  //put HM10 in AT command mode
  delay(100);
  mySerial.write("AT+NAME ERBLE\r\n");  //Name our HM10 something so as to not interfere with others
  delay(100);
  mySerial.write("AT+NAME\r\n");  //Verify new name
  delay(100);
  mySerial.write("AT+RESET\r\n");  //reset HM10 so new name will take effect

}

void loop(){

  direction = analogRead(VRx);	// read X axis value [0..1023]
  mySerial.print("X:");
  mySerial.println(direction, DEC);
  direction = analogRead(VRy);	// read Y axis value [0..1023]
  mySerial.print("Y:");
  mySerial.println(direction, DEC);
  delay(250);
    //switch statement to control buzzer mode
  switch (input){
    case '+':{
      //pwm to control buzzer tone
      digitalWrite(BuzzPin, HIGH);
      delay(1);
      digitalWrite(BuzzPin, LOW);
      break;
    }
    case '-':{
      digitalWrite(BuzzPin, HIGH);
      delay(2);
      digitalWrite(BuzzPin, LOW);
      break;
    }
    case '0':{
      //turn buzzer off
      digitalWrite(BuzzPin, LOW);
      break;      
    }
    default:{
      //input validation
      mySerial.println("The character you entered is not recognized, please reset the program and try again");
      break;
    }
  }
}

//function cgetcharfromphone - inputs:none returns:char
char cGetCharFromPhone(){
  int IInByte;                            //declare local variable
  mySerial.println("Enter '+' to set the buzzer to high frequency, '-' to set it to low frequency, and '0' to turn it off");
  mySerial.println("reset the program to change input");
  while (mySerial.available() == 0) {}     //wait for data available
  IInByte = mySerial.read();               //read character
  return(char(IInByte));
}
//**********************solenoid*************************************
const int solenoid = 10;
void setup() {


void loop() {


    Serial.println("Solenoid firing");
    digitalWrite(solenoid, HIGH);
    delay(200);
    digitalWrite(solenoid, LOW);
//********************servo*************************************
//set digital I/O pin
const int pulsepin=11;
//set timer value
int timer = 150;

void setup() {

  //initialize pulsepin as output
  pinMode(pulsepin,OUTPUT);
}

void loop() {
  //get angle from user via cgetcharfromkeyboard function
  char angle=cGetCharFromKeyboard();
//switch statement to choose angle
  switch (angle){
    case '+':{
      //for loop to send pulse a number of times proportional to timer value
      for (int i=0; i <= timer; i++){
        digitalWrite(pulsepin,HIGH);
        delay(1);
        digitalWrite(pulsepin,LOW);
        delay(19);
      }
      break;
    }
    case '-':{
      for (int i=0; i <= timer; i++){
        digitalWrite(pulsepin,HIGH);
        delay(2);
        digitalWrite(pulsepin,LOW);
        delay(18);
      }
      break;
    }
    case '0':{
      for (int i=0; i <= timer; i++){
        digitalWrite(pulsepin,HIGH);
        delay(1.5);
        digitalWrite(pulsepin,LOW);
        delay(18.5);
      }
      break;      
    }
    default:{
      //input validation
      Serial.println("The character you entered is not recognized, please try again");
      delay(200);
      break;
    }
  }
}
//*****************Push Button*************************************************************************
/*
  State change detection (edge detection)

  Often, you don't need to know the state of a digital input all the time, but
  you just need to know when the input changes from one state to another.
  For example, you want to know when a button goes from OFF to ON. This is called
  state change detection, or edge detection.

  This example shows how to detect when a button or button changes from off to on
  and on to off.

  The circuit:
  - pushbutton attached to pin 5 from +5V
  - 10 kilohm resistor attached to pin 2 from ground

  created  27 Sep 2005
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/StateChangeDetection
*/

// this constant won't change:
const int buttonPin = 12;  // the pin that the pushbutton is attached to

// Variables will change:
int buttonState = 0;        // current state of the button
int lastButtonState = 0;        // last state of the button

void setup() {
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  // initialize serial communication:
  Serial.begin(9600);
}


void loop() {
  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      Serial.println("on");
    } 
    else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("off");
    }
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
}
//**********Motion Sensor***************************
/*
 * PIR sensor tester - modified from Adafruit.com
 */
int inputPin = 13;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status
 
void setup() {
  pinMode(inputPin, INPUT);     // declare sensor as input
 
  Serial.begin(9600);
}
 
void loop(){
  val = digitalRead(inputPin);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
    if (pirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
    if (pirState == HIGH){
      // we have just turned of
      Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }
  delay(500);
}



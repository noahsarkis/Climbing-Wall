#include <SoftwareSerial.h> //BLE library
SoftwareSerial mySerial(6,7);  //BLE TX is on pin2 and RX on pin 
const int BuzzPin = 0; //buzzer pin
const int VRx = 0; //analogpins for joystick
const int VRy = 1; //^
const int sw = 8;  //^^
const int buttonPin = 12;  // the pin that the pushbutton is attached to
const int inputPin = 13;               // choose the input pin (for PIR sensor)
const int pirState = LOW;             // we start, assuming no motion detected
const int solenoid = 10; //solenoid pin
const int servopin=11;  //servopin
// Variables will change:
int servotimer = 50;        //time to move servo to new position
int buttonState = 0;        // current state of the button
int lastButtonState = 0;        // last state of the button
int buzztimer = 1000;       //amount of time the buzzer will be on for in ms
int val = 0;                    // variable for reading the motion sensor status
//***************4x4*******************************************************************************************************
const int latchPin1 = 4;	// Latch pin of 74HC595 is connected to Digital pin 5
const int clockPin1 = 5;	// Clock pin of 74HC595 is connected to Digital pin 6
const int dataPin1 = 3;	// Data pin of 74HC595 is connected to Digital pin 4

const int latchPin2 = 1;	// Latch pin of 74HC595 is connected to Digital pin 1
const int clockPin2 = 2;	// Clock pin of 74HC595 is connected to Digital pin 2
const int dataPin2 = 0;	// Data pin of 74HC595 is connected to Digital pin 0

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
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  //initialize motion sensor pin as input
  pinMode(inputPin, INPUT);
  //initialise solenoid pin as output
  pinMode(solenoidpin, OUTPUT);
  //initialize servo pin as output
  pinMode(servopin, OUTPUT);
  // Set all the pins of 74HC595 as OUTPUT
  pinMode(latchPin1, OUTPUT);
  pinMode(dataPin1, OUTPUT);  
  pinMode(clockPin1, OUTPUT);
  pinMode(latchPin2, OUTPUT);
  pinMode(dataPin2, OUTPUT);  
  pinMode(clockPin2, OUTPUT);
  // initialize serial communication:
  Serial.begin(9600);
  
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
      mySerial.println("Would you like the wall at an angle? enter y for yes and n for no");
      char angle = cGetCharFromPhone();
      switch (input){
        case 'y':{
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
        }
        case 'n':{
          //turn off solenoid
          digitalWrite(solenoid, HIGH);
          //reset servo angle
          for (int i=0; i <= servotimer; i++){
            digitalWrite(servopin,HIGH);
            delay(1.5);
            digitalWrite(servopin,LOW);
            delay(18.5);  
          break;
        }
        default{
          mySerial.println("The input entered is not recognized, please reset the program and try again");
          break;
        }
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
  //***********Buzzer triggered by Button**********************************************************
  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      for (int i; i <= buzztimer; i++){
        digitalWrite(BuzzPin, HIGH);
        delay(1);
        digitalWrite(BuzzPin, LOW);
      }
    } 
    else {
      // if the current state is LOW then the button went from on to off: Turn off buzzer
      digitalWrite(BuzzPin, LOW);
    }
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
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
  while (mySerial.available() == 0) {}     //wait for data available
  IInByte = mySerial.read();               //read character
  return(char(IInByte));
}

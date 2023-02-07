int latchPin1 = 5;	// Latch pin of 74HC595 is connected to Digital pin 5
int clockPin1 = 6;	// Clock pin of 74HC595 is connected to Digital pin 6
int dataPin1 = 4;	// Data pin of 74HC595 is connected to Digital pin 4

int latchPin2 = 1;	// Latch pin of 74HC595 is connected to Digital pin 1
int clockPin2 = 2;	// Clock pin of 74HC595 is connected to Digital pin 2
int dataPin2 = 0;	// Data pin of 74HC595 is connected to Digital pin 0

byte ledcol = 0;		// Variable to hold the pattern of which LEDs are currently turned on or off
byte ledrow = 0;
byte ledrow1 = 0b00001010;
byte ledrow2 = 0b00001001;
byte ledrow3 = 0b00000100;
byte ledrow4 = 0b00000011;

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

  bitSet(ledcol,0);
  updateShiftRegister();
  delay(1);
  ledrow = ledrow1;
  updateShiftRegister();
  ledcol = 0;
  delay(1);
  
  bitSet(ledcol,1);
  updateShiftRegister();
  delay(1);
  ledrow = ledrow2;
  updateShiftRegister();
  ledcol = 0;
  delay(1);

  bitSet(ledcol,2);
  updateShiftRegister();
  delay(1);
  ledrow = ledrow3;
  updateShiftRegister();
  ledcol = 0;
  delay(1);

  bitSet(ledcol,3);
  updateShiftRegister();
  delay(1);
  ledrow = ledrow4;
  updateShiftRegister();
  ledcol = 0;
  delay(1);

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
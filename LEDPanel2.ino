/******************************************************************************
This is an Arduino sketch to drive 4 LED panels based on MBI5034 LED drivers.

Written by Oliver Dewdney and Jon Russell

It works specifically on a Arduino Micro (ATMega32U4), as it accesses the 
registers directly. However, with a small amount of editing, it should work on 
most Arduinos.

Basic Operation:

The objective was to be able to drive four panels with a single Arduino. 
Each panel has two data lines, so four panels require 8 data lines, i.e. a 
single byte.

An Arduino Micro was selected, as it has a native USB port and slightly more
RAM than an Uno. A Mega would probably work well too.

The code has a frame buffer in RAM with 4 sets of 384 bits 
(1 bank = 64 LEDs x 2 Rows x 3 bits (RGB) = 384) for each data line. 
Four panels, two data lines each, means all four panels can be driven by a byte 
wide frame buffer, assuming 3 bit colour. This means the update can be very 
efficient. The frame buffer is about 1.5KB so fits happily in to the ATMega32U4
with some room for local variables.

The UpdateFrame loop iterates over the line of 384 bits of serial data from the
frame buffer and clocks them out quickly.

Ideally, we needed a contiguous port on the Microcontroller to be attached to 
8 external data lines for the four panels. But most Arduinos don’t have this. 
On the Arduino Micro, PortB has the RX LED on bit 0 and PortD has the TX LED on
bit 5. So, I have connected 4 data lines to the high nibble on PortB and 4 data
lines to the low nibble on PortD.

If we had a contiguous port free (If we used a Mega) we could use only one port
and the UpdateFrame loop would be 1 instruction faster ! :-) But currently I 
copy the data to both ports, ignoring the high and low nibbles I don’t need.

UpdateFrame is called by an interrupt 390 times a second 
(OCR1A = 160; // compare match register 16MHz/256/390Hz)). 
UpdateFrame needs to be called 4 times to update the entire panel, as the panel
is split in to 4 rows of 128 LEDs (as 2 rows of 64).

For each half a panel (one data line) there are 8 rows of 64 LEDs, addresses in 
banks. Bank 0x00 is row 0&4,  Bank 0x01 is row 1&5, Bank 0x02 is row 2&6, Bank 
0x03 is row 3&7.

Each call updates one bank and leaves it lit until the next interrupt.

This method updates the entire frame (1024 RGB LEDs) about 100 times a second.

Port map for Arduino Micro (ATMega32U4)
    7     6     5     4     3     2     1     0
PB  D11   D10   D9    D8    MISO  MOSI  SCK   RX/SS
PC  D13   D5    X     X     X     X     X     X
PD  D6    D12   TX    D4    D1    D0    D2    D3
PE  X     D7    X     X     X     HWB   X     X
PF  A0    A1    A2    A3    X     X     A4    A5
******************************************************************************/

#include "digitalWriteFast.h"
#include "font.h"

#define PIN_D1    3   //PD0 - D1 on Panel 1
#define PIN_D2    2   //PD1 - D2 on Panel 1
#define PIN_D3    0   //PD2 - D1 on Panel 2
#define PIN_D4    1   //PD3 - D2 on Panel 2

#define PIN_D5    8   //PB4 - D1 on Panel 3
#define PIN_D6    9   //PB5 - D2 on Panel 3
#define PIN_D7    10  //PB6 - D1 on Panel 4
#define PIN_D8    11  //PB7 - D2 on Panel 4

#define PIN_A0    A4  //PF1 - A0 on all Panels
#define PIN_A1    A1  //PF6 - A1 on all Panels
#define PIN_CLK   A3  //PF4 - CLK on all Panels
#define PIN_LAT   A2  //PF5 - LAT on all Panels
#define PIN_OE    A5  //PF0 - OE on all Panels

byte frame[4][384];

void FillBuffer(byte b){
  for(uint8_t x=0; x<4; x++){
    for(uint16_t y=0; y<384; y++){
      frame[x][y] = b;
    }
  }
}

// bb describes which data lines drive which of the 4 panels.
// By adjusting the order of the bits in the array, you can change the panel order visually.
byte bb[8] = { 0x40, 0x80, 0x10, 0x20, 0x04, 0x08, 0x01, 0x02 };

// Set a pixel to a specific 3 bit colour (8 colours)
// 0b00000000 = black (off), 0b00000001 = Red, 0b00000010 = Green, 0b00000100 = Blue, 0b00000111 = White, etc.
void setpixel(byte x, byte y, byte col) {
  int16_t off = (x&7) + (x & 0xf8)*6 + ((y & 4)*2);
//  int16_t off = (x&7) + (x >> 3)*48 + ((y & 4)*2);
  byte row = y & 3;
  byte b = bb[(y&0x3f) >> 3];
  byte *p = & frame[row][off];
  for(byte c = 0; c < 3;c++) {
    if ( col & 1 ) {
      *p |= b;
    } else {
      *p &= ~b;
    }
    col >>= 1;
    p += 16;
  }
}

void drawText(byte x, byte y, byte col, const char* msg) {
  while(*msg) {
    const char c = *msg;
    if (c==32) col++;
    const unsigned char* f  = &font[((uint8_t)c)*5];
    for( byte fx = 0; fx < 5; fx++) {
      unsigned char fc = pgm_read_byte_near(f++);
      for(byte fy = 0; fy < 7; fy++) {
        if (fc & (1<<fy)){
          setpixel(x+fx, y+fy, col);
        }
      }
    }
    x += 6;
    if (x>=(63-5)){
      x = 0;
      y += 8;
    }
    msg++;
  }
}

uint8_t bank = 0;

void UpdateFrame() {

  byte * f = frame[bank];
  for (uint16_t n = 0; n<384; n++) {
    PORTD = *f;      // We use the low nibble on PortD for Panel 1 & 2
    PORTB = *f++;    // We use the high nibble on PortB for Panel 3 & 4
    digitalWriteFast(PIN_CLK, LOW);
    digitalWriteFast(PIN_CLK, HIGH);
    }

  digitalWriteFast(PIN_OE,HIGH);     // disable output
  if (bank & 0x01) {
    digitalWriteFast(PIN_A0, HIGH);
  } else {
    digitalWriteFast(PIN_A0, LOW);
  }
  if (bank & 0x02) {
    digitalWriteFast(PIN_A1, HIGH);
  } else {
    digitalWriteFast(PIN_A1, LOW);
  }
  digitalWriteFast(PIN_LAT, HIGH);   // toggle latch
  digitalWriteFast(PIN_LAT, LOW);
  digitalWriteFast(PIN_OE, LOW);     // enable output

  if (++bank>3) bank=0;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Begin...");
  
  pinMode(PIN_D1, OUTPUT);
  pinMode(PIN_D2, OUTPUT);
  pinMode(PIN_D3, OUTPUT);
  pinMode(PIN_D4, OUTPUT);

  pinMode(PIN_D5, OUTPUT);
  pinMode(PIN_D6, OUTPUT);
  pinMode(PIN_D7, OUTPUT);
  pinMode(PIN_D8, OUTPUT);

  pinMode(PIN_A0, OUTPUT);
  pinMode(PIN_A1, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_LAT, OUTPUT);
  pinMode(PIN_OE, OUTPUT);

  digitalWrite(PIN_D1, LOW);
  digitalWrite(PIN_D2, LOW);
  digitalWrite(PIN_D3, LOW);
  digitalWrite(PIN_D4, LOW);

  digitalWrite(PIN_A0, LOW);
  digitalWrite(PIN_A1, LOW);

  digitalWrite(PIN_OE, HIGH);
  digitalWrite(PIN_LAT, LOW);
  digitalWrite(PIN_CLK, LOW);

  FillBuffer(0xFF);         // Set all LEDs on. (White)

  // initialize Timer1 ~400Hz
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 160;              // compare match register 16MHz/256/390Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_COMPA_vect){     // timer compare interrupt service routine
  UpdateFrame();
}

// Fill the frame with a specific colour (3 bit colour) with a delay to show a "wipe" effect.
void FillColour(byte c){
  for (int y=0; y<64; y++){     // assumes 4 panels. Y is 4 x 16 LEDs high
    for (int x=0; x<64; x++){   // Each panel has 64 LEDs on X
      setpixel(x, y, c);
      delay(2);
    }
  }
}

void loop(){

  // The screen will be white when we first enter the loop
  delay(2000);
  FillBuffer(0x00);     // Set black (all off)
  drawText(2, 0,7,"1111111111");
  drawText(2, 8,1,"2222222222");
  drawText(2,16,2,"3333333333");
  drawText(2,24,3,"4444444444");
  drawText(2,32,4,"5555555555");
  drawText(2,40,5,"6666666666");
  drawText(2,48,6,"7777777777");
  drawText(2,56,7,"8888888888");

  
/*  
  // Clours fade pattern. Cylces the whole display through all the 7 colours
  delay(1000);
  FillColour(0x01);
  FillColour(0x02);
  FillColour(0x03);
  FillColour(0x04);
  FillColour(0x05);
  FillColour(0x06);
  delay(1000);
  FillBuffer(0xFF);
  delay(4000);
*/  




}





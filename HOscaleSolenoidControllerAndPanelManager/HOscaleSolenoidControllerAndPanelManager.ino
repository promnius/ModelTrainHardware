

// LIBRARY INCLUDES
#include <SPI.h>              // We use this library, so it must be called here.
#include <MCP23S17.h>         // Here is the new class to make using the MCP23S17 easy.


int heartBeatCounter = 0;
boolean heartBeatStatus = false;

int intTurnoutStatus[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int intTurnoutDefaultState[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int intTurnoutSetting0[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int intTurnoutSetting1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int i = 0; // thats right, I declared global counter variables, and have two different nameing schemes because sometimes I'm in a
int counter = 0; // different mood. Sue me.

// PIN DECLARATIONS
const unsigned short pinHEARTBEAT = 9;
const unsigned short pinCS_LOCAL_MAGNETDRIVES = 7;
const unsigned short pinCS_CONTROLPANEL0 = 6;

// PORT EXPANDERS
MCP mcpSolenoidDrives0  (0, pinCS_LOCAL_MAGNETDRIVES);
MCP mcpSolenoidDrives1  (1, pinCS_LOCAL_MAGNETDRIVES);
MCP mcpButtons  (0, pinCS_CONTROLPANEL0);
MCP mcpPanelLEDs0  (1, pinCS_CONTROLPANEL0);
MCP mcpPanelLEDs1  (2, pinCS_CONTROLPANEL0);


// HEARTBEAT!!
ISR(TIMER2_COMPA_vect){
  heartBeatCounter ++;
  if (heartBeatCounter >= 50){
    heartBeatCounter = 0;
    if (heartBeatStatus){
      digitalWrite(pinHEARTBEAT,LOW);
      heartBeatStatus = false;
    }
    else{
      digitalWrite(pinHEARTBEAT,HIGH);
      heartBeatStatus = true;
    }
  }
}


void setup() {

  // SETTING UP HEARTBEAT
  cli();//stop interrupts
  //set timer0 interrupt at 100 hz
  TCCR2A = 0; // clear entire control register
  TCCR2B = 0;
  TCNT2  = 0; //initialize counter value to 0
  OCR2A = 155;// set compare match register for 100Hz increments. OCR0A = (16*10^6) / (100*1024) - 1 (must be <256)
  TCCR2A |= (1 << WGM21); // turn on CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Set CS01 and CS00 bits for 64 prescaler
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();//allow interrupts
  pinMode(pinHEARTBEAT, OUTPUT);
  digitalWrite(pinHEARTBEAT, LOW);

  //Serial.begin(115200);
  //Serial.println("Setup Begin");

  // initializing hardware pins (and default states where necessary).

  //Serial.println("Initializing MCP port expanders");
  delay(10);
  initializePortExpanders();
  delay(10);
  
  //Serial.println("Setup Done.");
  //Serial.println("");
  delay(500);
}

void loop() {
  checkButtons();
  updateTurnoutStatus();
  updatePanelLEDs();
}

// set up all the port expanders for beginning their journey
MCP mcpSolenoidDrives0  (0, pinCS_LOCAL_MAGNETDRIVES);
MCP mcpSolenoidDrives1  (1, pinCS_LOCAL_MAGNETDRIVES);
MCP mcpButtons  (0, pinCS_CONTROLPANEL0);
MCP mcpPanelLEDs0  (1, pinCS_CONTROLPANEL0);
MCP mcpPanelLEDs1  (2, pinCS_CONTROLPANEL0);
void initializePortExpanders(){  
  LEDs_1to16.begin();
  LEDs_17to32.begin();

  for (i = 1; i <= 16; i++) {    // Since we are only workign with one bit at a time, use a loop to take action each pin (0-15)
    // THERE IS A MUCH FASTER WAY TO DO THIS USING WORDS, BUT WE DON'T CARE ABOUT SPEED
    LEDs_1to16.pinMode(i, OUTPUT);      // set all LEDs as outputs, driven low to turn the LEDs off (they are probably low by default,
    LEDs_17to32.pinMode(i, OUTPUT);      // but no harm in making sure)
    LEDs_1to16.digitalWrite(i, LOW);
    LEDs_17to32.digitalWrite(i, LOW);
  }
}

// this function is what physically updates the real world for the MCP expanders. prior to calling this
// we are just messing around with the sofware mask. this makes for efficient hardware communications.
void MCPLEDsUpdate(){
  LEDs_1to16.digitalWrite((int)((lngLEDBarGraphBitMask << 16) >> 16));
  LEDs_17to32.digitalWrite((int)(lngLEDBarGraphBitMask >> 16));
}

// this function takes an array of desired states for the 16 turnouts, and cycles through them one at a
// time to ensure they are in the correct position, regardless of where they were (or where we thought they
// were) before. This can be used to establish default track settings, or to update a layout that may have
// had manual intervention.
void setFullTrackLayout(int intTurnoutStatusesToSet[16]){
  for (i=0; i<16; i++){
    setTurnout(i,intTurnoutStatusesToSet[i]);
  }
}

// this function takes a turnout number (0 thru 15) and a state (0 or 1, meaning straight or turned),
// and drives the solenoids to set this state, before returning all solenoids to the 'off' position.
void setTurnout(int intTurnout, int intNewState){
  
}

// scans the buttons and increments their counter if the button is held down. This provides a crude debounce
// and time the button has been held if this function is called at regular intervals. There are far better 
// ways of doing this and existing libraries to manage it, but this is pretty simple and hardware specific.
// it will not increment a button past 1000 to prevent overflows (who cares if it has been held down for days?)
void checkButtons(){
  for (i=0; i<16; i++){
    
  }
}

updateTurnoutStatus();

updatePanelLEDs();

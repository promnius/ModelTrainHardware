

// LIBRARY INCLUDES
#include <SPI.h>              // We use this library, so it must be called here.
#include <MCP23S17.h>         // Here is the new class to make using the MCP23S17 easy.

int buttonToTurnoutMap[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; // this

int heartBeatCounter = 0;
boolean heartBeatStatus = false;

int intTurnoutStatus[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // could be a mask, as each one is either 1 or 0, but since this information
// needs further decoding before driving hardware, it is just as functional like this.
int intTurnoutDefaultState[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int intTurnoutSetting0[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int intTurnoutSetting1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int intButtonStatus[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int intREDLEDStatusMASK = 0; // not used yet
int intGREENLEDStatusMASK = 0;


// PIN DECLARATIONS
const unsigned short pinHEARTBEAT = 9;
const unsigned short pinCS_LOCAL_MAGNETDRIVES = 10;
const unsigned short pinCS_CONTROLPANEL0 = 2;

// PORT EXPANDERS
MCP mcpSolenoidDrives0  (0, pinCS_LOCAL_MAGNETDRIVES);
MCP mcpSolenoidDrives1  (1, pinCS_LOCAL_MAGNETDRIVES);
MCP mcpButtons  (0, pinCS_CONTROLPANEL0);
MCP mcpPanel1LEDsGreen  (1, pinCS_CONTROLPANEL0);
MCP mcpPanel1LEDsRed  (2, pinCS_CONTROLPANEL0);


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

  Serial.begin(9600);
  //Serial.println("Setup Begin");

  // initializing hardware pins (and default states where necessary).

  //Serial.println("Initializing MCP port expanders");
  delay(10);
  initializePortExpanders();
  delay(500);

  loadStateFromDisk();

  establishDefaultState();

  updatePanelLEDs();
  
  //Serial.println("Setup Done.");
  //Serial.println("");
  //delay(500);
}

void loop() {
  checkButtons();
  interpretButtons();
  delay(30);
}

// set up all the port expanders for beginning their journey
void initializePortExpanders(){  
  mcpSolenoidDrives0.begin();
  mcpSolenoidDrives1.begin();
  mcpButtons.begin();
  mcpPanel1LEDsGreen.begin();
  mcpPanel1LEDsRed.begin();

  for (int i = 0; i < 16; i++) {    // Since we are only workign with one bit at a time, use a loop to take action each pin (1-16,
    // apparently MCP does not index from 0)
    // THERE IS A MUCH FASTER WAY TO DO THIS USING WORDS, BUT WE DON'T CARE ABOUT SPEED
    mcpSolenoidDrives0.pinMode(i+1, OUTPUT);      // set all Solenoids as outputs, driven low to turn them off (they are probably low by default,
    mcpSolenoidDrives1.pinMode(i+1, OUTPUT);      // but no harm in making sure)
    mcpSolenoidDrives0.digitalWrite(i+1, LOW);
    mcpSolenoidDrives1.digitalWrite(i+1, LOW);

    mcpButtons.pinMode(i+1, INPUT);// set all buttons as inputs with pullups
    mcpButtons.pullupMode(i+1, HIGH); 

    mcpPanel1LEDsGreen.pinMode(i+1, OUTPUT);// set all buttons as inputs// set all LEDs as outputs, turn none on for now. We'll turn them on as we establish a default state.
    mcpPanel1LEDsRed.pinMode(i+1, OUTPUT);
    mcpPanel1LEDsGreen.digitalWrite(i+1, HIGH);
    mcpPanel1LEDsRed.digitalWrite(i+1, HIGH);
  }
}


// this function takes an array of desired states for the 16 turnouts, and cycles through them one at a
// time to ensure they are in the correct position, regardless of where they were (or where we thought they
// were) before. This can be used to establish default track settings, or to update a layout that may have
// had manual intervention.
void setFullTrackLayout(int intTurnoutStatusesToSet[16]){
  for (int i=0; i<13; i++){
    setTurnout(i,intTurnoutStatusesToSet[i]);
  }
}

int intTurnoutIndex = 0;
// this function takes a turnout number (0 thru 15) and a state (0 or 1, meaning straight or turned),
// and drives the solenoids to set this state, before returning all solenoids to the 'off' position.
void setTurnout(int intTurnout, int intNewState){
  //Serial.print(intTurnout); Serial.print(",");
  //Serial.print(intNewState); Serial.print(",");
  intTurnoutIndex = -1; // lets figure out which pin to toggle.
  if (intNewState == 1){
    intTurnoutIndex = (intTurnout * 2) + 1;
  }
  else {
    intTurnoutIndex = intTurnout * 2;
  }
  //Serial.print(intTurnoutIndex);Serial.print(",");
  if (intTurnout < 8){
    //Serial.print("MCP0");
    mcpSolenoidDrives0.digitalWrite(intTurnoutIndex+1, HIGH);
  }
  else {
    //Serial.print("MCP1");
    mcpSolenoidDrives1.digitalWrite(intTurnoutIndex-15, HIGH);    
  }
  delay(500); // time for the solenoid to actually switch. Note that this is blocking so all button presses, or any
  // other library that doesn't use interrupts will freeze here. Could be fixed in the future with an event handler, but
  // for this use case there is no issue with the delay.
  
  mcpSolenoidDrives0.digitalWrite(0); // turn off all the solenoids so nothing lights on fire.
  mcpSolenoidDrives1.digitalWrite(0); // if we were smart about it we'd only need to turn off the one we just used,
  // but this is safe, and speed is not an issue.
  //Serial.println("");
}

// scans the buttons and increments their counter if the button is held down. This provides a crude debounce
// and time the button has been held if this function is called at regular intervals. There are far better 
// ways of doing this and existing libraries to manage it, but this is pretty simple and hardware specific.
// it will not increment a button past 1000 to prevent overflows (who cares if it has been held down for days?)
void checkButtons(){
  for (int i=0; i<16; i++){
    if (mcpButtons.digitalRead(i+1) == 0){ // the button is pressed
      //Serial.println("PRESSED");
      if (intButtonStatus[i] < 1){
        intButtonStatus[i] = 1;
      } else {
        intButtonStatus[i] ++; // keep track of how long the button has been pressed for
      }
      if (intButtonStatus[i] > 1000){ // prevent overflow
        intButtonStatus[i] = 1000;
      }
    }
    else { // the button is NOT pressed
      //Serial.println("NOT PRESSED");
      if (intButtonStatus[i] < 0) { // the button has been off for a cycle, if anything needed to happen it should
        // have been handled already. we can reset the button now.
        intButtonStatus[i] = 0;
      }
      if (intButtonStatus[i] > 0) { // the button has just been released, make the number negative to indicate
        // to anyone watching that the button has been released, and how long it was held for
        intButtonStatus[i] = 0-intButtonStatus[i];
      }
    }
  }
}

// THIS FUNCTION IS HARDWARE SPECIFIC
void interpretButtons(){
  for (int i=0; i<13; i++){ // the first 13 buttons just toggle turnout states
    if (intButtonStatus[i] < -1) { // button just released, the -1 debounces
      invertTurnoutStatus(i);
      updatePanelLEDs();
      setTurnout(buttonToTurnoutMap[i], intTurnoutStatus[i]);
      if (i == 1) { // button 1 is special, as this corresponds to the button that controls 2 turnouts
        intTurnoutStatus[15] = intTurnoutStatus[1];
        updatePanelLEDs();
        setTurnout(15, intTurnoutStatus[15]);
      }
      
    }
  }
  //Serial.println(intButtonStatus[14]);
  for (int i = 13; i<16; i++){ // buttons 13,14,and 15 correspond to special track settings
    if (intButtonStatus[i] > 150) { // roughly, button has been held for 5 seconds, assuming this function
      // is called at 30 hz. this breaks pretty heavily if this is ever run with a different code base.
      turnOnAllLights();
    }
    else if (intButtonStatus[i] < -150) { // button was held for a while, but has now been released
      for (int counter = 0; counter < 16; counter ++){
        if (i == 13){
          intTurnoutDefaultState[counter] = intTurnoutStatus[counter];
        } else if (i == 14){
          intTurnoutSetting0[counter] = intTurnoutStatus[counter];
        } else if (i == 15){
          intTurnoutSetting1[counter] = intTurnoutStatus[counter];
        }
      }
      updatePanelLEDs(); // the only thing that should change is we should be in this default state now, so that LED should turn on
      // (and we should be turning off all the extra lights we just turned on).
      saveStateToDisk();
    }
    else if (intButtonStatus[i] < -1){ // button was pressed.
      for (int counter = 0; counter < 16; counter ++){
        if (i == 13){
          intTurnoutStatus[counter] = intTurnoutDefaultState[counter];
        } else if (i == 14){
          intTurnoutStatus[counter] = intTurnoutSetting0[counter];
        } else if (i == 15){
          intTurnoutStatus[counter] = intTurnoutSetting1[counter];
        }
        updatePanelLEDs();
        setTurnout(counter, intTurnoutStatus[counter]); // This is REALLY inefficient since many turnouts may be in the correct position
        // already. AND it is a long blocking function.
      }
    }
  //updatePanelLEDs();
  }
}


void invertTurnoutStatus(int intTurnout){
  if (intTurnoutStatus[intTurnout] == 0){
    intTurnoutStatus[intTurnout] = 1;
  } else {
    intTurnoutStatus[intTurnout] = 0;
  }
}


// there could be a more intelligent way to do this . . . only updating the LEDs that changed, and only calling
// this when some LEDs changed, but this is simple too.
// could also be more efficient by building a mask and talking to the port expanders once.
void updatePanelLEDs(){
  for (int i=0; i<13; i++) { // the first 13 leds all correspond to their turnout state
    if (intTurnoutStatus[i] == 0){
      mcpPanel1LEDsGreen.digitalWrite(i+1,LOW);
      mcpPanel1LEDsRed.digitalWrite(i+1,HIGH);
    } else {
      mcpPanel1LEDsRed.digitalWrite(i+1,LOW);
      mcpPanel1LEDsGreen.digitalWrite(i+1,HIGH);
    }
  }
  if (intTurnoutStatus[15] == 0) {
    mcpPanel1LEDsGreen.digitalWrite(15,LOW);
    mcpPanel1LEDsRed.digitalWrite(16,HIGH);
  } else {
    mcpPanel1LEDsRed.digitalWrite(16,LOW);
    mcpPanel1LEDsGreen.digitalWrite(15,HIGH);
  }
  boolean booDefault = true;
  boolean booProgram0 = true;
  boolean booProgram1 = true;
  for (int i=0; i<16; i++) { // are we in state default, program0, or program1?
    if (intTurnoutStatus[i] != intTurnoutDefaultState[i]){
      booDefault = false;
    }
    if (intTurnoutStatus[i] != intTurnoutSetting0[i]){
      booProgram0 = false;
    }
    if (intTurnoutStatus[i] != intTurnoutSetting1[i]){
      booProgram1 = false;
    }
  }
  //Serial.print(booDefault); Serial.print(",");Serial.print(booProgram0);Serial.print(",");Serial.println(booProgram1);
  if (booDefault){mcpPanel1LEDsGreen.digitalWrite(14,HIGH);} else{mcpPanel1LEDsGreen.digitalWrite(14,LOW);}
  if (booProgram0){mcpPanel1LEDsRed.digitalWrite(14,HIGH);} else{mcpPanel1LEDsRed.digitalWrite(14,LOW);}
  if (booProgram1){mcpPanel1LEDsRed.digitalWrite(15,HIGH);} else{mcpPanel1LEDsRed.digitalWrite(15,LOW);}
}

void turnOnAllLights(){
  mcpPanel1LEDsGreen.digitalWrite(0xFFFF);
  mcpPanel1LEDsRed.digitalWrite(0xFFFF);
}

void saveStateToDisk(){
    
}

void loadStateFromDisk(){
  
}

void establishDefaultState(){
  
}

void startBlink(int intLEDNumber){
  
}

void stopBlink(){
  
}

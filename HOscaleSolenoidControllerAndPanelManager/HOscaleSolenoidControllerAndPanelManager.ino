

// Kyle Mayer
// 8/19/2019
// This code manages a 32 channel low side solenoid driver configured in 16 pairs for driving dual action
// solenoid turnouts on an HO scale model train set.
// It also handles all the interactions with an external control panel that provides a set of push buttons
// and LEDs that enables the user to control the solenoids and know their statuses graphically.

// LIBRARY INCLUDES
#include <SPI.h>              // We use this library within the MCP23S17 library, so it must be called here.
#include <MCP23S17.h>

// VARIABLE DECLARATIONS
int buttonToTurnoutMap[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; // this

int heartBeatCounter = 0; // only for dividing down timer2 to a visible speed
boolean heartBeatStatus = false; // keeps track of heartbeat on or off.

// the current status of each turnout, straight or turned. Note that this is NOT the state of the solenoid drive,
// since those always return to all off after the solenoid is fired. 0 means straight (or default), 1 means turned.
int intTurnoutStatus[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // could be a mask, as each one is either 1 or 0, but since this information
// needs further decoding before driving hardware, it is just as functional like this and allows walking through the elements with a loop
// with code that is easier to read than masking out each bit.
int intTurnoutDefaultState[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // the three preset states. they will be loaded from eeprom on startup.
int intTurnoutSetting0[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // could all be masks to save eeprom space as well.
int intTurnoutSetting1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int intButtonStatus[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // keeps track of the status of buttons for debouncing and timing (for functions
// that respond to how long a button has been held for.) This one actually does need to be an array, since the value is used for timing.

int intREDLEDStatusMASK = 0; // not used. currently all LED states are figured out from the turnout status, and are updated immediately
// when the turnout status changes. It could be more efficient, and allow additional effects like blinking lights etc. to keep a mask (or 
// array) that tracks the status of the LEDs separate from the status of the turnouts.
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

  // Serial.begin(9600); // We actually don't use serial in this project outside of debugging 

  // initializing hardware pins (and default states where necessary).
  initializePortExpanders();
  delay(500); // let the LEDs stay all on for a second

  loadStateFromDisk();

  establishDefaultState();

  updatePanelLEDs();
}

void loop() {
  checkButtons();
  interpretButtons(); // this will handle all the driving of the appropriate solenoids and the LEDs.
  delay(30); // set the refresh rate to roughly 30hz. The button handler is counting on this (and the low CPU usage of this
  // project will keep it roughly right), so don't mess with this or add any delays elsewhere unless you are ok with the buttons
  // not responding.
}

// set up all the port expanders for beginning their journey, to be called once.
void initializePortExpanders(){  
  mcpSolenoidDrives0.begin();
  mcpSolenoidDrives1.begin();
  mcpButtons.begin();
  mcpPanel1LEDsGreen.begin();
  mcpPanel1LEDsRed.begin();

  for (int i = 0; i < 16; i++) {    // Since we are only working with one bit at a time, use a loop to take action on each pin
    // THERE IS A MUCH FASTER WAY TO DO THIS USING WORDS, BUT WE DON'T CARE ABOUT SPEED
    mcpSolenoidDrives0.pinMode(i+1, OUTPUT);      // set all Solenoids as outputs, driven low to turn them off (they are probably low by default,
    mcpSolenoidDrives1.pinMode(i+1, OUTPUT);      // but no harm in making sure)
    mcpSolenoidDrives0.digitalWrite(i+1, LOW);    // the plus ones occur because apparently MCP does not index from 0
    mcpSolenoidDrives1.digitalWrite(i+1, LOW);    // could just change the counter variable range, but it would be more confusing since everything
    // else in this project indexes from 0.

    mcpButtons.pinMode(i+1, INPUT);// set all buttons as inputs with pullups enabled, since there are no onboard pullups.
    mcpButtons.pullupMode(i+1, HIGH); 

    mcpPanel1LEDsGreen.pinMode(i+1, OUTPUT); // set all LEDs as outputs, turn all of them on for now. This tests the hardware so burned out lights
    // are identifiable, but also lets the user know we are still in setup mode. The code will turn them to an appropriate state when setup is finished.
    mcpPanel1LEDsRed.pinMode(i+1, OUTPUT);
    mcpPanel1LEDsGreen.digitalWrite(i+1, HIGH);
    mcpPanel1LEDsRed.digitalWrite(i+1, HIGH);
  }
}

// this function takes a turnout number (0 thru 15) and a state (0 or 1, meaning straight or turned),
// and drives the solenoids to set this state, before returning all solenoids to the 'off' position.
// BLOCKING FUNCTION. There are far better ways to do this using an event handler or some timer based
// interrupts, but it just doesn't matter for this project. If the buttons don't respond while the solenoids
// are being driven, I almost consider that a good thing.
void setTurnout(int intTurnout, int intNewState){
  int intTurnoutIndex = -1;// lets figure out which pin to toggle, since our solenoids are split across
  // 2 port expanders
  if (intNewState == 1){
    intTurnoutIndex = (intTurnout * 2) + 1;
  }
  else {
    intTurnoutIndex = intTurnout * 2;
  }
  if (intTurnout < 8){
    mcpSolenoidDrives0.digitalWrite(intTurnoutIndex+1, HIGH);
  }
  else {
    mcpSolenoidDrives1.digitalWrite(intTurnoutIndex-15, HIGH);    
  }
  delay(500); // time for the solenoid to actually switch. Note that this is blocking so all button presses, or any
  // other library that doesn't use interrupts will freeze here. Could be fixed in the future with an event handler, but
  // for this use case there is no issue with the delay.
  mcpSolenoidDrives0.digitalWrite(0); // turn off all the solenoids so nothing lights on fire.
  mcpSolenoidDrives1.digitalWrite(0); // if we were smart about it we'd only need to turn off the one we just used,
  // but this is safe, and speed is not an issue.
}

// scans the buttons and increments their counter if the button is held down. It inverts the number when the button
// is released, to tell the button interpreter that an action can be performed if the button has been held long
// enough. This provides a crude debounce
// and time the button has been held if this function is called at regular intervals. There are far better 
// ways of doing this and existing libraries to manage it, but this is pretty simple and hardware specific.
// it will not increment a button past 1000 to prevent overflows (who cares if it has been held down for days?)
void checkButtons(){
  for (int i=0; i<16; i++){
    if (mcpButtons.digitalRead(i+1) == 0){ // the button is pressed
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

// HARDWARE SPECIFIC FUNCTION
// This function decides what a button press means, and activates the solenoids as needed,
// and updates the LEDs to match.
// this function may be a blocking function, as if any buttons have been pressed it may actuate the solenoids,
// and currently our solenoid drive is blocking. Depending on the button pressed (like the default buttons for 
// example), this function may need to actuate a great deal of solenoids and block for a long time.
// It is hardware specific because many buttons have specific, special functions.
void interpretButtons(){
  for (int i=0; i<13; i++){ // the first 13 buttons just toggle turnout states when pressed (when released)
    if (intButtonStatus[i] < -1) { // button just released, the -1 debounces
      invertTurnoutStatus(i);
      updatePanelLEDs();
      setTurnout(buttonToTurnoutMap[i], intTurnoutStatus[i]);
      if (i == 0) { // button 0 is special, as this corresponds to the button that controls 2 turnouts
        intTurnoutStatus[15] = intTurnoutStatus[1];
        updatePanelLEDs();
        setTurnout(15, intTurnoutStatus[15]);
      }
    }
  }
  for (int i = 13; i<16; i++){ // buttons 13,14,and 15 correspond to special track settings
    if (intButtonStatus[i] > 150) { // roughly, button has been held for 5 seconds, assuming this function
      // is called at 30 hz. this breaks pretty heavily if this is ever run with a different code base
      // that doesn't call this function at a fixed rate.
      turnOnAllLights(); // let the user know the button has been held long enough and can now be released.
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
      // (and we should be turning off all the extra lights we just turned on). ie, no solenoids need to fire.
      saveStateToDisk();
    }
    else if (intButtonStatus[i] < -1){ // button was pressed, but not held down, so now it is time to resume this state.
      // note that we are only fireing any solenoids that we think need to change, not all of them, so this does NOT
      // fix a situation where a solenoid has been changed by hand.
      for (int counter = 0; counter < 16; counter ++){
        if (i == 13){
          if (intTurnoutStatus[counter] != intTurnoutDefaultState[counter]){
            intTurnoutStatus[counter] = intTurnoutDefaultState[counter]; 
            updatePanelLEDs();
            setTurnout(counter, intTurnoutStatus[counter]);           
          }
        } else if (i == 14){
          if (intTurnoutStatus[counter] != intTurnoutSetting0[counter]){
            intTurnoutStatus[counter] = intTurnoutSetting0[counter]; 
            updatePanelLEDs();
            setTurnout(counter, intTurnoutStatus[counter]);           
          }
        } else if (i == 15){
           if (intTurnoutStatus[counter] != intTurnoutSetting1[counter]){
            intTurnoutStatus[counter] = intTurnoutSetting1[counter]; 
            updatePanelLEDs();
            setTurnout(counter, intTurnoutStatus[counter]);           
          }
        }
      }
    }
  }
}

// This function takes a turnout status and inverts it
void invertTurnoutStatus(int intTurnout){
  if (intTurnoutStatus[intTurnout] == 0){
    intTurnoutStatus[intTurnout] = 1;
  } else {
    intTurnoutStatus[intTurnout] = 0;
  }
}


// HARDWARE SPECIFIC FUNCTION
// Currently, this function updates the LEDs to match the status held in the intSolenoidStatus register.
// it doesn't actually modify the solenoids, so if a change is made to the register a command to modify the 
// solenoid must be sent separately.
// it is hardware specific because certain LEDs have real world significance, so we are not just matching LED0 to
// solenoid0, etc.
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
  if (intTurnoutStatus[15] == 0) { // the final turnout is a bit strange and got de-synced from its LEDs
    mcpPanel1LEDsGreen.digitalWrite(15,LOW);
    mcpPanel1LEDsRed.digitalWrite(16,HIGH);
  } else {
    mcpPanel1LEDsRed.digitalWrite(16,LOW);
    mcpPanel1LEDsGreen.digitalWrite(15,HIGH);
  }
  // now, lets figure out if any of the default states are active so we can turn on their LEDs
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
  if (booDefault){mcpPanel1LEDsGreen.digitalWrite(14,HIGH);} else{mcpPanel1LEDsGreen.digitalWrite(14,LOW);}
  if (booProgram0){mcpPanel1LEDsRed.digitalWrite(14,HIGH);} else{mcpPanel1LEDsRed.digitalWrite(14,LOW);}
  if (booProgram1){mcpPanel1LEDsRed.digitalWrite(15,HIGH);} else{mcpPanel1LEDsRed.digitalWrite(15,LOW);}
}

// a simple function to turn on every light. This is the only form of display rendering we have,
// since I've choosen not to use anything like blinking lights or other things that require tracking
// light status in the time domain.
void turnOnAllLights(){
  mcpPanel1LEDsGreen.digitalWrite(0xFFFF);
  mcpPanel1LEDsRed.digitalWrite(0xFFFF);
}

// saves all presets to disk. Could be more efficient and only save the ones that have changed, or swap up the address used
// to prevent wear, but it won't matter for this project. Useful anytime a preset changes.
void saveStateToDisk(){
    
}

// loads all presets from disk. Useful on startup.
void loadStateFromDisk(){
  
}

// this function drives every solenoid to the position the software thinks it is in. This is useful on startup when
// we don't know where anything is, or anytime the user thinks a solenoid has been modified by hand, as it reasserts
// all solenoids to known states. Note that it is blocking, so all other functionality is broken while this runs.
// current advise, turn all lights on while this is active to provide feedback.
void establishDefaultState(){
  for (int i = 0; i<16; i++){
    setTurnout(i, intTurnoutStatus[i]);
  }
}

// not supported, would be easier with an object oriented light that can have states, and an event handler that 
// monitors real time status of everything.
void startBlink(int intLEDNumber){
  
}

// not supported.
void stopBlink(){
  
}

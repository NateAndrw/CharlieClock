// PROJECT  :CharlieClock
// PURPOSE  :CharlieClock Code
// COURSE   :ISC3U-E
// AUTHOR   :N.ANDREW
// DATE     :2025 10 4
// MCU      :328P (Standalone)
// STATUS   :Working
// REFERENCE:

#define P1 7
#define P2 6
#define P3 5
#define P4 4
#define P5 3
#define P6 8
#define P7 A0
#define P8 A3
#define P9 A4
#define P10 A5
#define P11 A2
#define P12 A1

#define MODEBTN  2    // mode button pin (toggle run/set) - active HIGH
#define FIELDBTN 11   // field select pin (cycle sec/min/hr) - active HIGH
#define ADDBTN   12   // add/increment pin (increment selected field) - active HIGH

struct LED { uint8_t anode; uint8_t cathode; };

LED Hours[] = { {P9,P1},{P10,P1},{P11,P1},{P12,P1},{P12,P11},{P5,P1},
  {P4,P1},{P3,P1},{P2,P1},{P6,P1},{P7,P1},{P8,P1} };

LED Minutes[] = {
  {P8,P2},{P9,P2},{P10,P2},{P11,P2},{P12,P2},{P1,P12},{P2,P12},{P3,P12},{P4,P12},{P5,P12},
  {P6,P12},{P7,P12},{P8,P12},{P9,P12},{P10,P12},{P11,P12},{P10,P11},{P9,P11},{P8,P11},{P7,P11},
  {P6,P11},{P5,P11},{P4,P11},{P3,P11},{P2,P11},{P1,P11},{P12,P10},{P11,P10},{P9,P10},{P8,P10},
  {P7,P10},{P6,P10},{P5,P10},{P4,P10},{P3,P10},{P2,P10},{P1,P10},{P7,P4},{P6,P4},{P5,P4},
  {P3,P4},{P2,P4},{P1,P4},{P1,P3},{P2,P3},{P4,P3},{P5,P3},{P6,P3},{P7,P3},{P8,P3},{P9,P3},
  {P10,P3},{P11,P3},{P12,P3},{P1,P2},{P3,P2},{P4,P2},{P5,P2},{P6,P2},{P7,P2}
};

LED Seconds[] = {
  {P1,P5},{P2,P5},{P3,P5},{P4,P5},{P6,P5},{P7,P5},{P8,P5},{P9,P5},{P10,P5},{P11,P5},{P12,P5},
  {P8,P4},{P9,P4},{P10,P4},{P11,P4},{P12,P4},{P12,P9},{P11,P9},{P10,P9},{P8,P9},{P7,P9},
  {P6,P9},{P5,P9},{P4,P9},{P3,P9},{P2,P9},{P1,P9},{P12,P8},{P11,P8},{P10,P8},{P9,P8},{P7,P8},
  {P6,P8},{P5,P8},{P4,P8},{P3,P8},{P2,P8},{P1,P8},{P12,P7},{P11,P7},{P10,P7},{P9,P7},{P8,P7},
  {P6,P7},{P5,P7},{P4,P7},{P3,P7},{P2,P7},{P1,P7},{P1,P6},{P2,P6},{P3,P6},{P4,P6},{P5,P6},
  {P7,P6},{P8,P6},{P9,P6},{P10,P6},{P11,P6},{P12,P6}
};

const uint8_t NUMH = sizeof(Hours)/sizeof(LED);   // number of hour LEDs
const uint8_t NUMM = sizeof(Minutes)/sizeof(LED); // number of minute LEDs
const uint8_t NUMS = sizeof(Seconds)/sizeof(LED); // number of second LEDs

volatile unsigned long pulseCount = 0; // counts oscillator pulses (32.768kHz)
uint8_t secCount = 0;                  // seconds counter (0-59)
uint8_t minCount = 0;                  // minutes counter (0-59)
uint8_t hourCount = 12;                // hours counter (1-12), starts at 12
bool setMode = false;                  // false = run, true = set
uint8_t fieldSel = 0;                  // 0=sec,1=min,2=hour (selected in set mode)

bool modeState = false;                // last stable read for mode button
bool fieldState = false;               // last stable read for field button
bool addState = false;                 // last stable read for add button
unsigned long tMode = 0;               // timestamp for debounce mode
unsigned long tField = 0;              // timestamp for debounce field
unsigned long tAdd = 0;                // timestamp for debounce add

void hizAll(){                          // set all charlie pins to high-impedance
  pinMode(P1,INPUT); pinMode(P2,INPUT); pinMode(P3,INPUT); pinMode(P4,INPUT);
  pinMode(P5,INPUT); pinMode(P6,INPUT); pinMode(P7,INPUT); pinMode(P8,INPUT);
  pinMode(P9,INPUT); pinMode(P10,INPUT); pinMode(P11,INPUT); pinMode(P12,INPUT);
}

void lightLED(const LED &l){            // light a single LED (anode HIGH, cathode LOW)
  hizAll();                             // make sure all others are high-Z
  pinMode(l.anode,OUTPUT);              // set anode pin as output
  digitalWrite(l.anode,HIGH);           // drive anode HIGH
  pinMode(l.cathode,OUTPUT);            // set cathode pin as output
  digitalWrite(l.cathode,LOW);          // drive cathode LOW
}

ISR(PCINT0_vect){                       // PB1 pin-change ISR for oscillator input
  static uint8_t last = 0;              // keep last level
  uint8_t now = PINB & (1<<PB1);        // read PB1 level
  if(now && !last) pulseCount++;        // count rising edges only
  last = now;                           // save current level
}

bool pressedEdge(uint8_t pin, bool &stableState, unsigned long &stamp){ // edge detector w/debounce
  bool raw = digitalRead(pin);          // raw reading
  unsigned long now = millis();         // current time
  if(raw != stableState){               // if changed since last stable
    if(now - stamp >= 25){              // and stable for 25ms
      stableState = raw;                // accept new state
      stamp = now;                      // update stamp
      return stableState;               // return true if now HIGH (pressed)
    }
  }
  return false;                         // no new press
}

void setup(){                            // initialization
  pinMode(MODEBTN, INPUT);               // mode button input (active HIGH)
  pinMode(FIELDBTN, INPUT);              // field button input (active HIGH)
  pinMode(ADDBTN, INPUT);                // add button input (active HIGH)

  modeState = digitalRead(MODEBTN);      // seed initial stable button state
  fieldState = digitalRead(FIELDBTN);    // seed initial stable button state
  addState = digitalRead(ADDBTN);        // seed initial stable button state

  PCMSK0 |= (1<<PCINT1);                 // enable PCINT1 (PB1) mask
  PCICR  |= (1<<PCIE0);                  // enable pin-change interrupts for port B
}

void loop(){                             // main loop
  if(setMode){                           // if in set mode, freeze pulse counter
    noInterrupts(); pulseCount = 0; interrupts(); // clear pulses while setting
  } else {                               // running mode: accumulate seconds
    if(pulseCount >= 32768UL){           // 32768 pulses = 1 second
      noInterrupts(); pulseCount -= 32768UL; interrupts(); // subtract a second
      if(++secCount == 60){ secCount = 0; // rollover seconds
        if(++minCount == 60){ minCount = 0; // rollover minutes
          if(++hourCount == 13) hourCount = 1; // rollover hours 12->1
        }
      }
    }
  }

  if(pressedEdge(MODEBTN, modeState, tMode)){ // detect new MODE press
    if(!setMode){                          // entering set mode
      setMode = true; hourCount = 12; minCount = 0; secCount = 0; // reset to 12:00:00
      noInterrupts(); pulseCount = 0; interrupts(); // clear pulses
    } else {                               // leaving set mode
      setMode = false;                     // keep the set time and resume
      noInterrupts(); pulseCount = 0; interrupts(); // ensure clean start counting
    }
  }

  if(setMode){                             // only respond to these in set mode
    if(pressedEdge(FIELDBTN, fieldState, tField)){ // new FIELD press
      fieldSel = (fieldSel + 1) % 3;       // cycle 0->1->2->0
    }
    if(pressedEdge(ADDBTN, addState, tAdd)){ // new ADD press
      if(fieldSel == 0){ if(++secCount == 60) secCount = 0; } // increment sec
      else if(fieldSel == 1){ if(++minCount == 60) minCount = 0; } // increment min
      else { if(++hourCount == 13) hourCount = 1; } // increment hour
    }
  }

  lightLED(Hours[hourCount % NUMH]); delayMicroseconds(500); // display hour LED
  lightLED(Minutes[minCount % NUMM]); delayMicroseconds(500); // display minute LED
  lightLED(Seconds[secCount % NUMS]); delayMicroseconds(500); // display second LED
}

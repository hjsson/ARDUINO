        /*  2019-04-02  mrSson ,  First coding. 
         *  UPPERN 확인으로 상판인지 하판인지 구분하여 구동 
         *  Modified DATE. Author. Description............
         *  2019.04.05     Arnold. Arrange document style and hand-over from MrSson. New naming samrtFAN-v10.ino
         *  2019.04.06     Pin change A0 <--> A5 for communication test.
         *  2019.04.13     Change Naming to smartFanV2.ino
         *  2019.04.15     BUGFIX. PulseIn function time out setting. SIGNAL gabage filtering function and some bugfix aaded.  
         *  2019.04.17     FAN status detection logic changed.
         *  2019.04.19     * FAIL LIST & MODIFY LIST
         *                 - [**1] 상판 하판 레벨 확인. 왔다 갔다 함 [OK]
         *                 - [**2] PIN FAULT 신호가 HIGH 유지여야 하는데, 펄스 신호처럼 나옴. // 전역에서 핀에 대한 상태 모니터링으로 처리. fanFAULT 변수를 참조하게 함. []
         *                 - [**3] 자체 온도에 의해 팬이 돌 경우에도 장애 표현이 됨. []
         *  2019.04.21     *V3, V2.1 version (04.20) 폐기.
         *                 - [**4] 작동 온도 제어 기능 . 최초 기동 온도는 20도가 되지 않으면 작동 안함. 작동이 시작되면 10도까지는 작동함.
         *                 - [**5] ANALOG로 핀 읽어서 데이타 처리하는 방식으로 바꿈. 팬이 STOP할 경우 두가지의 상태 나타남. 
         *                         * 500 이상의 긴 PULSE 주기적으로 나타남.(5개 이상이 지속되면) - 알람 울림.
         *                         * 특정 VALUE로 지속적으로 나타남. (50개 이상이면)          - 알람 울림.
         *                         * 900 --> 1000 수정 본.
         * 2019.04.22      *V4 LOGIC 변경. 
         *                 - [**6] ADC --> DIGITAL, PULSE IN reading & count. RPM 측정 및 특정 시간당 HIGH COUNT.
         *                 
         * 2019.04.23      *V5 CALIBRATION VALUE 저장
         *                 - [**7] INIT VALUE 측정 하고 저장 .
         *                 
         * 2019.04.25      *V10 ANALOG BASED SOLUTION APPLIED. SPEED & HALT detection possible. 
         *                 - [**8] PULSE READING and ANALYZING LOGIC applied.. Very SIMPLE !!!!
         * 
         *                 
        */
        
        //#include <PinChangeInt.h>
        // This file is part of the PinChangeInt library for the Arduino.  This library will work on any ATmega328-based
// or ATmega2560-based Arduino, as well as the Sanguino or Mioduino.

// Most of the pins of an Arduino Uno use Pin Change Interrupts, and because of the way the ATmega interrupt
// system is designed it is difficult to trigger an Interrupt Service Request off of any single pin, and on
// any change of state (either rising, or falling, or both).  The goal of this library is to make it easy for
// the programmer to attach an ISR so it will trigger on any change of state on any Pin Change Interrupt pin.

// (NOTE TO SELF: Update the PCINT_VERSION define, below) -----------------
#define PCINT_VERSION 2402
/*
Copyright 2008 Chris J. Kiick
Copyright 2009-2011 Lex Talionis
Copyright 2010-2014 Michael Schwager (aka, "GreyGnome")
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * QUICKSTART
 *
 * For the beginners/lazy/busy/wreckless:
 * To attach an interrupt to your Arduino Pin, calling your function "userFunc", and acting on
 * the "mode", which is a change in the pin's state; either RISING or FALLING or CHANGE:
 *    attachPinChangeInterrupt(pin,userFunc,mode)
 * Your function must not return any values and it cannot take any arguments (that is, its definition
 * has to look like this:
 *    void userFunc() {
 *      ...your code here...
 *    }
 *
 * That's it. Everything else are details.
 *
 * If you need to exchange information to/from the interrupt, you can use global volatile variables.
 * See the example for more information.
 *
 *  You probably will not need to do this, but later in your sketch you can detach the interrupt:
 *    detachPinChangeInterrupt(pin)
 *
 *  If you want to see what the *last* pin that triggered an interrupt was, you can get it this way:
 *    getInterruptedPin()
 *  Note: If you have multiple pins that are triggering interrupts and they are sufficiently fast,
 *  you will not be able to find all the pins that interrupted.
*/

//
// For the beginners
//
#define detachPinChangeInterrupt(pin)       PCintPort::detachInterrupt(pin)
#define attachPinChangeInterrupt(pin,userFunc,mode) PCintPort::attachInterrupt(pin, &userFunc,mode)
#define getInterruptedPin()             PCintPort::getArduinoPin()

// We use 4-character tabstops, so IN VIM:  <esc>:set ts=4 sw=4 sts=4
// ...that's: ESCAPE key, colon key, then
//    "s-e-t SPACE key t-s = 4 SPACE key s-w = 4 SPACE key s-t-s = 4"

/*
 *  This is the PinChangeInt library for the Arduino.
  This library provides an extension to the interrupt support for arduino by adding pin change
  interrupts, giving a way for users to have interrupts drive off of any pin (ATmega328-based
  Arduinos) and by the Port B, J, and K pins on the Arduino Mega and its ilk (see the README file).
  See the README for license, acknowledgments, and other details (especially concerning the Arduino MEGA).
  See google code project for latest, bugs and info http://code.google.com/p/arduino-pinchangeint/
  See github for the bleeding edge code: https://github.com/GreyGnome/PinChangeInt
  For more information Refer to avr-gcc header files, arduino source and atmega datasheet.
  This library was inspired by and derived from Chris J. Kiick's PCInt Arduino Playground
  example here: http://www.arduino.cc/playground/Main/PcInt
  Nice job, Chris!
*/

//-------- define these in your sketch, if applicable ----------------------------------------------------------
//-------- These must go in your sketch ahead of the #include <PinChangeInt.h> statement -----------------------
// You can reduce the memory footprint of this handler by declaring that there will be no pin change interrupts
// on any one or two of the three ports.  If only a single port remains, the handler will be declared inline
// reducing the size and latency of the handler.
// #define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts
// #define NO_PORTC_PINCHANGES // to indicate that port c will not be used for pin change interrupts
// #define NO_PORTD_PINCHANGES // to indicate that port d will not be used for pin change interrupts
// --- Mega support ---
// #define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts
// #define NO_PORTJ_PINCHANGES // to indicate that port j will not be used for pin change interrupts
// #define NO_PORTK_PINCHANGES // to indicate that port k will not be used for pin change interrupts
// In the Mega, there is no Port C, no Port D.  Instead, you get Port J and Port K.  Port B remains.
// Port J, however, is practically useless because there is only 1 pin available for interrupts.  Most
// of the Port J pins are not even connected to a header connection.  // </end> "Mega Support" notes
// --- Sanguino, Mioduino support ---
// #define NO_PORTA_PINCHANGES // to indicate that port a will not be used for pin change interrupts

// You can reduce the code size by 20-50 bytes, and you can speed up the interrupt routine
// slightly by declaring that you don't care if the static variables PCintPort::pinState and/or
// PCintPort::arduinoPin are set and made available to your interrupt routine.
// #define NO_PIN_STATE        // to indicate that you don't need the pinState
// #define NO_PIN_NUMBER       // to indicate that you don't need the arduinoPin
// #define DISABLE_PCINT_MULTI_SERVICE // to limit the handler to servicing a single interrupt per invocation.
// #define GET_PCINT_VERSION   // to enable the uint16_t getPCIintVersion () function.
// The following is intended for testing purposes.  If defined, then a whole host of static variables can be read
// in your interrupt subroutine.  It is not defined by default, and you DO NOT want to define this in
// Production code!:
// #define PINMODE
//-------- define the above in your sketch, if applicable ------------------------------------------------------

/*
  VERSIONS found in moved to RELEASE_NOTES.
  See the README file for the License and more details.
*/

#ifndef PinChangeInt_h
#define PinChangeInt_h

#include "stddef.h"

// Maurice Beelen, nms277, Akesson Karlpetter, and Orly Andico
// sent in fixes to work with Arduino >= version 1.0
#include <Arduino.h>
#include <new.h>
#include <wiring_private.h> // cbi and sbi defined here

#undef DEBUG

/*
* Theory: For the IO pins covered by Pin Change Interrupts
* (== all of them on the Atmega168/328, and a subset on the Atmega2560),
* the PCINT corresponding to the pin must be enabled and masked, and
* an ISR routine provided.  Since PCINTs are per port, not per pin, the ISR
* must use some logic to actually implement a per-pin interrupt service.
*/

/* Pin to interrupt map, ATmega328:
* D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
* D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
* A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
*/

#undef  INLINE_PCINT
#define INLINE_PCINT
// Thanks to cserveny...@gmail.com for MEGA support!
#if defined __AVR_ATmega2560__ || defined __AVR_ATmega1280__ || defined __AVR_ATmega1281__ || defined __AVR_ATmega2561__ || defined __AVR_ATmega640__
  #define __USE_PORT_JK
  // Mega does not have PORTA, C or D
  #define NO_PORTA_PINCHANGES
  #define NO_PORTC_PINCHANGES
  #define NO_PORTD_PINCHANGES
  #if ((defined(NO_PORTB_PINCHANGES) && defined(NO_PORTJ_PINCHANGES)) || \
      (defined(NO_PORTJ_PINCHANGES) && defined(NO_PORTK_PINCHANGES)) || \
      (defined(NO_PORTK_PINCHANGES) && defined(NO_PORTB_PINCHANGES)))
    #define INLINE_PCINT inline
  #endif
#else
  #define NO_PORTJ_PINCHANGES
  #define NO_PORTK_PINCHANGES
  #if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
    #ifndef NO_PORTA_PINCHANGES
      #define __USE_PORT_A
    #endif
  #else
    #define NO_PORTA_PINCHANGES
  #endif
  // if defined only D .OR. only C .OR. only B .OR. only A, then inline it
  #if (   (defined(NO_PORTA_PINCHANGES) && defined(NO_PORTB_PINCHANGES) && defined(NO_PORTC_PINCHANGES)) || \
      (defined(NO_PORTA_PINCHANGES) && defined(NO_PORTB_PINCHANGES) && defined(NO_PORTD_PINCHANGES)) || \
      (defined(NO_PORTA_PINCHANGES) && defined(NO_PORTC_PINCHANGES) && defined(NO_PORTD_PINCHANGES)) || \
      (defined(NO_PORTB_PINCHANGES) && defined(NO_PORTC_PINCHANGES) && defined(NO_PORTD_PINCHANGES)) )
    #define INLINE_PCINT inline
  #endif
#endif

// Provide drop in compatibility with Chris J. Kiick's PCInt project at
// http://www.arduino.cc/playground/Main/PcInt
#define PCdetachInterrupt(pin)  PCintPort::detachInterrupt(pin)
#define PCattachInterrupt(pin,userFunc,mode) PCintPort::attachInterrupt(pin, userFunc,mode)
#define PCgetArduinoPin() PCintPort::getArduinoPin()

typedef void (*PCIntvoidFuncPtr)(void);

class PCintPort {
public:
  // portB=PCintPort(2, 1,PCMSK1);
  // index:   portInputReg(*portInputRegister(index)), 
  // pcindex: PCICRbit(1 << pcindex)
  // maskReg: portPCMask(maskReg)
  PCintPort(int index,int pcindex, volatile uint8_t& maskReg) :
  portInputReg(*portInputRegister(index)),
  portPCMask(maskReg),
  PCICRbit(1 << pcindex),
  portRisingPins(0),
  portFallingPins(0),
  firstPin(NULL)
#ifdef PINMODE
  ,intrCount(0)
#endif
  {
    #ifdef FLASH
    ledsetup();
    #endif
  }
  volatile  uint8_t&    portInputReg;
  static    int8_t attachInterrupt(uint8_t pin, PCIntvoidFuncPtr userFunc, int mode);
  static    void detachInterrupt(uint8_t pin);
  INLINE_PCINT void PCint();
  static volatile uint8_t curr;
  #ifndef NO_PIN_NUMBER
  static  volatile uint8_t  arduinoPin;
  #endif
  #ifndef NO_PIN_STATE
  static volatile uint8_t pinState;
  #endif
  #ifdef PINMODE
  static volatile uint8_t pinmode;
  static volatile uint8_t s_portRisingPins;
  static volatile uint8_t s_portFallingPins;
  static volatile uint8_t s_lastPinView;
  static volatile uint8_t s_pmask;
  static volatile char s_PORT;
  static volatile uint8_t s_changedPins;
  static volatile uint8_t s_portRisingPins_nCurr;
  static volatile uint8_t s_portFallingPins_nNCurr;
  static volatile uint8_t s_currXORlastPinView;
  volatile uint8_t intrCount;
  static volatile uint8_t s_count;
  static volatile uint8_t pcint_multi;
  static volatile uint8_t PCIFRbug;
  #endif
  #ifdef FLASH
  static void ledsetup(void);
  #endif

protected:
  class PCintPin {
  public:
    PCintPin() :
    PCintFunc((PCIntvoidFuncPtr)NULL),
    mode(0) {}
    PCIntvoidFuncPtr PCintFunc;
    uint8_t   mode;
    uint8_t   mask;
    uint8_t arduinoPin;
    PCintPin* next;
  };
  void    enable(PCintPin* pin, PCIntvoidFuncPtr userFunc, uint8_t mode);
  int8_t    addPin(uint8_t arduinoPin,PCIntvoidFuncPtr userFunc, uint8_t mode);
  volatile  uint8_t&    portPCMask;
  const   uint8_t     PCICRbit;
  volatile  uint8_t     portRisingPins;
  volatile  uint8_t     portFallingPins;
  volatile uint8_t    lastPinView;
  PCintPin* firstPin;
};

#ifndef LIBCALL_PINCHANGEINT // LIBCALL_PINCHANGEINT ***********************************************
volatile uint8_t PCintPort::curr=0;
#ifndef NO_PIN_NUMBER
volatile uint8_t PCintPort::arduinoPin=0;
#endif
#ifndef NO_PIN_STATE
volatile uint8_t PCintPort::pinState=0;
#endif
#ifdef PINMODE
volatile uint8_t PCintPort::pinmode=0;
volatile uint8_t PCintPort::s_portRisingPins=0;
volatile uint8_t PCintPort::s_portFallingPins=0;
volatile uint8_t PCintPort::s_lastPinView=0;
volatile uint8_t PCintPort::s_pmask=0;
volatile char  PCintPort::s_PORT='x';
volatile uint8_t PCintPort::s_changedPins=0;
volatile uint8_t PCintPort::s_portRisingPins_nCurr=0;
volatile uint8_t PCintPort::s_portFallingPins_nNCurr=0;
volatile uint8_t PCintPort::s_currXORlastPinView=0;
volatile uint8_t PCintPort::s_count=0;
volatile uint8_t PCintPort::pcint_multi=0;
volatile uint8_t PCintPort::PCIFRbug=0;
#endif

#ifdef FLASH
#define PINLED 13
volatile uint8_t *led_port;
uint8_t led_mask;
uint8_t not_led_mask;
boolean ledsetup_run=false;
void PCintPort::ledsetup(void) {
  if (! ledsetup_run) {
    led_port=portOutputRegister(digitalPinToPort(PINLED));
    led_mask=digitalPinToBitMask(PINLED);
    not_led_mask=led_mask^0xFF;
    pinMode(PINLED, OUTPUT); digitalWrite(PINLED, LOW);
    ledsetup_run=true;
  }
};
#endif

//
// ATMEGA 644 
//
#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) // Sanguino, Mosquino uino bobino bonanafannafofino, me my momino...

#ifndef NO_PORTA_PINCHANGES
PCintPort portA=PCintPort(1, 0,PCMSK0); // port PA==1  (from Arduino.h, Arduino version 1.0)
#endif
#ifndef NO_PORTB_PINCHANGES
PCintPort portB=PCintPort(2, 1,PCMSK1); // port PB==2  (from Arduino.h, Arduino version 1.0)
#endif
#ifndef NO_PORTC_PINCHANGES
PCintPort portC=PCintPort(3, 2,PCMSK2); // port PC==3  (also in pins_arduino.c, Arduino version 022)
#endif
#ifndef NO_PORTD_PINCHANGES
PCintPort portD=PCintPort(4, 3,PCMSK3); // port PD==4
#endif

#else // others

#ifndef NO_PORTB_PINCHANGES
PCintPort portB=PCintPort(2, 0,PCMSK0); // port PB==2  (from Arduino.h, Arduino version 1.0)
#endif
#ifndef NO_PORTC_PINCHANGES  // note: no PORTC on MEGA
PCintPort portC=PCintPort(3, 1,PCMSK1); // port PC==3  (also in pins_arduino.c, Arduino version 022)
#endif
#ifndef NO_PORTD_PINCHANGES  // note: no PORTD on MEGA
PCintPort portD=PCintPort(4, 2,PCMSK2); // port PD==4
#endif

#endif // defined __AVR_ATmega644__

#ifdef __USE_PORT_JK
#ifndef NO_PORTJ_PINCHANGES
PCintPort portJ=PCintPort(10,1,PCMSK1); // port PJ==10 
#endif
#ifndef NO_PORTK_PINCHANGES
PCintPort portK=PCintPort(11,2,PCMSK2); // port PK==11
#endif
#endif // USE_PORT_JK

static PCintPort *lookupPortNumToPort( int portNum ) {
    PCintPort *port = NULL;

  switch (portNum) {
#ifndef NO_PORTA_PINCHANGES
  case 1:
    port=&portA;
    break;
#endif
#ifndef NO_PORTB_PINCHANGES
  case 2:
    port=&portB;
    break;
#endif
#ifndef NO_PORTC_PINCHANGES
  case 3:
    port=&portC;
    break;
#endif
#ifndef NO_PORTD_PINCHANGES
  case 4:
    port=&portD;
    break;
#endif
#ifdef __USE_PORT_JK

#ifndef NO_PORTJ_PINCHANGES
  case 10:
    port=&portJ;
    break;
#endif

#ifndef NO_PORTK_PINCHANGES
  case 11:
    port=&portK;
    break;
#endif

#endif // __USE_PORT_JK
    }

    return port;
}


void PCintPort::enable(PCintPin* p, PCIntvoidFuncPtr userFunc, uint8_t mode) {
  // Enable the pin for interrupts by adding to the PCMSKx register.
  // ...The final steps; at this point the interrupt is enabled on this pin.
  p->mode=mode;
  p->PCintFunc=userFunc;
#ifndef NO_PORTJ_PINCHANGES
  // A big shout out to jrhelbert for this fix! Thanks!!!
  if ((p->arduinoPin == 14) || (p->arduinoPin == 15)) {
    portPCMask |= (p->mask << 1); // PORTJ's PCMSK1 is a little odd...
  }
  else {
    portPCMask |= p->mask;
  }
#else
    portPCMask |= p->mask;
#endif
  if ((p->mode == RISING) || (p->mode == CHANGE)) portRisingPins |= p->mask;
  if ((p->mode == FALLING) || (p->mode == CHANGE)) portFallingPins |= p->mask;
  PCICR |= PCICRbit;
}

int8_t PCintPort::addPin(uint8_t arduinoPin, PCIntvoidFuncPtr userFunc, uint8_t mode)
{
  PCintPin* tmp;

  tmp=firstPin;
  // Add to linked list, starting with firstPin. If pin already exists, just enable.
  if (firstPin != NULL) {
    do {
      if (tmp->arduinoPin == arduinoPin) { enable(tmp, userFunc, mode); return(0); }
      if (tmp->next == NULL) break;
      tmp=tmp->next;
    } while (true);
  }

  // Create pin p:  fill in the data.
  PCintPin* p=new PCintPin;
  if (p == NULL) return(-1);
  p->arduinoPin=arduinoPin;
  p->mode = mode;
  p->next=NULL;
  p->mask = digitalPinToBitMask(arduinoPin); // the mask

  if (firstPin == NULL) firstPin=p;
  else tmp->next=p; // NOTE that tmp cannot be NULL.

#ifdef DEBUG
  Serial.print("addPin. pin given: "); Serial.print(arduinoPin, DEC);
  int addr = (int) p;
  Serial.print(" instance addr: "); Serial.println(addr, HEX);
  Serial.print("userFunc addr: "); Serial.println((int)p->PCintFunc, HEX);
#endif

  enable(p, userFunc, mode);
#ifdef DEBUG
  Serial.print("addPin. pin given: "); Serial.print(arduinoPin, DEC), Serial.print (" pin stored: ");
  int addr = (int) p;
  Serial.print(" instance addr: "); Serial.println(addr, HEX);
#endif
  return(1);
}

/*
 * attach an interrupt to a specific pin using pin change interrupts.
 */
int8_t PCintPort::attachInterrupt(uint8_t arduinoPin, PCIntvoidFuncPtr userFunc, int mode)
{
  PCintPort *port;
  uint8_t portNum = digitalPinToPort(arduinoPin);
  if ((portNum == NOT_A_PORT) || (userFunc == NULL)) return(-1);

  port=lookupPortNumToPort(portNum);
  // Added by GreyGnome... must set the initial value of lastPinView for it to be correct on the 1st interrupt.
  // ...but even then, how do you define "correct"?  Ultimately, the user must specify (not provisioned for yet).
  port->lastPinView=port->portInputReg;
#ifdef DEBUG
  Serial.print("attachInterrupt- pin: "); Serial.println(arduinoPin, DEC);
#endif
  // map pin to PCIR register
  return(port->addPin(arduinoPin,userFunc,mode));
}

void PCintPort::detachInterrupt(uint8_t arduinoPin)
{
  PCintPort *port;
  PCintPin* current;
  uint8_t mask;
  uint8_t portNum = digitalPinToPort(arduinoPin);
  if (portNum == NOT_A_PORT) return;
  port=lookupPortNumToPort(portNum);
  mask=digitalPinToBitMask(arduinoPin);
  current=port->firstPin;
  while (current) {
    if (current->mask == mask) { // found the target
      uint8_t oldSREG = SREG;
      cli(); // disable interrupts
#ifndef NO_PORTJ_PINCHANGES
      // A big shout out to jrhelbert for this fix! Thanks!!!
      if ((arduinoPin == 14) || (arduinoPin == 15)) {
        port->portPCMask &= ~(mask << 1); // PORTJ's PCMSK1 is a little odd...
      }
      else {
        port->portPCMask &= ~mask; // disable the mask entry.
      }
#else
      port->portPCMask &= ~mask; // disable the mask entry.
#endif
      if (port->portPCMask == 0) PCICR &= ~(port->PCICRbit);
      port->portRisingPins &= ~current->mask; port->portFallingPins &= ~current->mask;
      // TODO: This is removed until we can add code that frees memory.
      // Note that in the addPin() function, above, we do not define a new pin if it was
      // once already defined.
      // ... ...
      // Link the previous' next to the found next. Then remove the found.
      //if (prev != NULL) prev->next=current->next; // linked list skips over current.
      //else firstPin=current->next; // at the first pin; save the new first pin
      SREG = oldSREG; // Restore register; reenables interrupts
      return;
    }
    current=current->next;
  }
}

// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.
void PCintPort::PCint() {

  #ifdef FLASH
  if (*led_port & led_mask) *led_port&=not_led_mask;
  else *led_port|=led_mask;
    #endif
  #ifndef DISABLE_PCINT_MULTI_SERVICE
  uint8_t pcifr;
  while (true) {
  #endif
    // get the pin states for the indicated port.
    #ifdef PINMODE
    PCintPort::s_lastPinView=lastPinView;
    intrCount++;
    PCintPort::s_count=intrCount;
    #endif
    uint8_t changedPins = (PCintPort::curr ^ lastPinView) &
                ((portRisingPins & PCintPort::curr ) | ( portFallingPins & ~PCintPort::curr ));

    #ifdef PINMODE
    PCintPort::s_currXORlastPinView=PCintPort::curr ^ lastPinView;
    PCintPort::s_portRisingPins_nCurr=portRisingPins & PCintPort::curr;
    PCintPort::s_portFallingPins_nNCurr=portFallingPins & ~PCintPort::curr;
    #endif
    lastPinView = PCintPort::curr;

    PCintPin* p = firstPin;
    while (p) {
      // Trigger interrupt if the bit is high and it's set to trigger on mode RISING or CHANGE
      // Trigger interrupt if the bit is low and it's set to trigger on mode FALLING or CHANGE
      if (p->mask & changedPins) {
        #ifndef NO_PIN_STATE
        PCintPort::pinState=PCintPort::curr & p->mask ? HIGH : LOW;
        #endif
        #ifndef NO_PIN_NUMBER
        PCintPort::arduinoPin=p->arduinoPin;
        #endif
        #ifdef PINMODE
        PCintPort::pinmode=p->mode;
        PCintPort::s_portRisingPins=portRisingPins;
        PCintPort::s_portFallingPins=portFallingPins;
        PCintPort::s_pmask=p->mask;
        PCintPort::s_changedPins=changedPins;
        #endif
        p->PCintFunc();
      }
      p=p->next;
    }
  #ifndef DISABLE_PCINT_MULTI_SERVICE
    pcifr = PCIFR & PCICRbit;
    if (pcifr == 0) break;
    PCIFR |= PCICRbit;
    #ifdef PINMODE
    PCintPort::pcint_multi++;
    if (PCIFR & PCICRbit) PCintPort::PCIFRbug=1; // PCIFR & PCICRbit should ALWAYS be 0 here!
    #endif
    PCintPort::curr=portInputReg;
  }
  #endif
}

#ifndef NO_PORTA_PINCHANGES
ISR(PCINT0_vect) {
  #ifdef PINMODE
  PCintPort::s_PORT='A';
  #endif
  PCintPort::curr = portA.portInputReg;
  portA.PCint();
}
#define PORTBVECT PCINT1_vect
#define PORTCVECT PCINT2_vect
#define PORTDVECT PCINT3_vect
#else
#define PORTBVECT PCINT0_vect
#define PORTCVECT PCINT1_vect
#define PORTDVECT PCINT2_vect
#endif

#ifndef NO_PORTB_PINCHANGES
ISR(PORTBVECT) {
  #ifdef PINMODE
  PCintPort::s_PORT='B';
  #endif
  PCintPort::curr = portB.portInputReg;
  portB.PCint();
}
#endif

#ifndef NO_PORTC_PINCHANGES
ISR(PORTCVECT) {
  #ifdef PINMODE
  PCintPort::s_PORT='C';
  #endif
  PCintPort::curr = portC.portInputReg;
  portC.PCint();
}
#endif

#ifndef NO_PORTD_PINCHANGES
ISR(PORTDVECT){ 
  #ifdef PINMODE
  PCintPort::s_PORT='D';
  #endif
  PCintPort::curr = portD.portInputReg;
  portD.PCint();
}
#endif

#ifdef __USE_PORT_JK
#ifndef NO_PORTJ_PINCHANGES
ISR(PCINT1_vect) {
  #ifdef PINMODE
  PCintPort::s_PORT='J';
  #endif
  PCintPort::curr = portJ.portInputReg;
  portJ.PCint();
}
#endif

#ifndef NO_PORTK_PINCHANGES
ISR(PCINT2_vect){ 
  #ifdef PINMODE
  PCintPort::s_PORT='K';
  #endif
  PCintPort::curr = portK.portInputReg;
  portK.PCint();
}
#endif

#endif // __USE_PORT_JK

#ifdef GET_PCINT_VERSION
uint16_t getPCIntVersion () {
  return ((uint16_t) PCINT_VERSION);
}
#endif // GET_PCINT_VERSION
#endif // #ifndef LIBCALL_PINCHANGEINT *************************************************************
#endif // #ifndef PinChangeInt_h *******************************************************************
        
        #define PULSE         0           // PULSE       ON:1 OFF:0
        #define DEBUG         0           // DEBUG       ON:1 OFF:0
        #define DEBUG_MORE    0           // DEBUG MORE  ON:1 OFF:0
        #define DEBUG_STAT    0           // DEBUG STATISTICS ON:1 OFF:0
        #define INTERRUPT     0           // INTERRUPT using MOTOR monitoring.
        #define FAST_ADC      1           // FAST ADC    ON:1 OFF:0
        #define sysDelay      20          // System delay time
        #define THERSHOLD_F   410        // Threshold Front FAN NORMAL 410
        #define THERSHOLD_B   200         // Threshold Back  FAN NORMAL, 200
        
        #define countFILTER   30         // FILTER for Release counter.
        #define countFILTER_O 30          // FILTER for FAULT ON count.
        #define countHIGH     100         // count MAX HIGH change period.
        
        #define UPPER         1         // UPPER FAN flag.
        #define LOWER         0         // LOWER FAN flag.
        #define speedFULL     250       // FAN FULL SPEED value.
        #define speedZERO     160         // FAN ZERO SPEED value.
        #define pwmMAX        2000      // PWM max signal value.
        #define fanFAIL       0         // FAN Fail value. 
        //#define MY_PIN      5         // PORT CHANGE INTERRUPT USING PIN we could choose any pin
        
        #define arrayCount    30        // arrayCount. SHOULD pairing NUM_AVERAGE !!!!
        #define NUM_AVERAGE   30        // average count
        #define NUM_TEMP_AVERAGE 100   // number of temperature average CNT
        
        // Temperature related variables. [**4]온도제어가 추가됨.
        #define tempMIN     21        // 21
        #define tempMAX     40        // 40

        // PULSE COUNT define.
        #define readingTIME   100     // 100mm sec duration
        
        
        // Interrupt variables.
        volatile int pwm_value = 0;
        volatile int prev_time = 0;
        uint8_t latest_interrupted_pin;
        
        // Pin mapping and variables.
        int alarmCNT=      0;
        int releaseCNT=    0;
        int debugCount =   0;
        int P_NOW_TEMP =  A7;          //온도센서                      //T_Sen : Input : Temperature Sensor /ADC7  PIN No A7    ---- Device TMP36
        int P_LED_ALM =   A0;          //팬 이상시 알람 LED             //LED_ALM : Output : Alarm  LED Turn On while FAN Fail /PC0 Pin No A0    ---- Normal Low / Fail High(Turn On)
        int P_LED_PWR =   A1;          //전체 파워 LED                 //LED_PWR : Output : Power LED Yrun On while Power On /PC1 Pin No A1     ---- Normal High / Fail Low(Turn Off- No Power)
        int P_F_FAN_RPM = A2;          //상위 팬 속도 확인               //FAN1_Mon : Input : Fan operation Monitor as Frequency Counter /PC2 Pin No A2  --- 2 pulse per one turn
        int P_B_FAN_RPM = A3;          //하위 팬 속도 확인               //FAN2_Mon : Input : Fan operation Monitor as Frequency Counter /PC3 Pin No A3  --- 2 Pulse per one turn
        int P_FAN_FAULT = A4;          //나의 팬 정상여부 전달 포트         //** FAN-Fault : Output : Fan Fail Alarm Output /PC6 Pin No A6 --- Fault High / Normal Low ==> Destination to MAIN monitoring system.  
                                                                     // 2019.04.18. P_FAN_FAULT pin mapping changed. /PC4 Pin No A4. Whole program download again !!! 
        int P_UPPERN =    A5;          //현재 상판인지 하판인지 구분 플레그   //UPPERN : Upper Tag : Upper status Check /PC5 Pin No A5  -----  Low is Upper / High is Lower 
                                                                                                                                    // HIGH is Upper / LOW is Lower . changed. 2019.04.19 FINAL !!!! 
        int P_F_FAN_PWM =  3;          //상위 팬 속도 주기               //PWM_FAN1 : Output : Fan#1 Speed Control PWM Signal /PD3 Pin No 3 --- 40% at 25C   100% at 31 / when No signal from other Unit, 100%
        int P_B_FAN_PWM =  5;          //하위 팬 속도 주기               //PWM_FAN2 : Output : Fan#1 Speed Control PWM Signal /PD5 Pin No 5 Same as FAN1
        
        int P_CONN_SIGNAL_OUT = 9;     //연결된 판과의 데이터 전송          //SENSE : Output & Input : Output PWN Signal when Upper Chhecked: Input Monitor when Lower checked / PB0(Output)Pin No 8  
        int P_CONN_SIGNAL_IN =  9;     //연결된 판과의 데이터 수신          //SENSE : Output & Input : Output PWN Signal when Upper Chhecked: Input Monitor when Lower checked / PB1(Input)Pin No 9
        boolean UP_DOWN =     false;       //false=하판, true=상판
        boolean fanFAULT =    false;      //false=NORMAL true=FAULT
        int tempOffset =  20;
        int fanSPEED=      0;
        int fanSPEEDCNT =  0;
        
        // fanFAULT 고도화 [**3], 자체 signal 신호가(MAX) 있으면서 알람 모드일 경우 3회 까지는 Reset. 연속 4회일 경우는 바로 알람 !!!
        int maxFanFaultCNT=   0;
        
        // [**4]온도관련 전역변수 : * 온도값을 return 형태는 작동을 위한 온도만 전달. 최저는 21도. 실제온도 관련 로직에서는 [**4]로직을 따름. 별도 관리용 변수.
        //                     * 온도값에 따라 20도 이상이면 작동 시작. 이때 tempOperation이 true가 됨. false가 되는 시점은 10도 이하로 내려가면.
        float nowTemperature =    0.0;
        boolean tempOperation = false;
        
        
        // PULSE Check variables.
        long startTimeF     = 0;  // RISING TIME F
        long endTimeF       = 0;  // FALLING TIME F
        long startTimeB     = 0;  // RISING TIME B
        long endTimeB       = 0;  // FALLING TIME B
        long durationTimeF  = 0;  // RISE - FALLING simple calculation.
        long durationTimeB  = 0;  // RISE - FALLING simple calculation.
        long durationError  = 0;  // Continued Error Time. specific time reached from Error occurred check there's Error continuing or NOT. (10sec / 10,000 milisec)
        boolean     isHIGHF = false;
        boolean     isHIGHB = false;
        int highCountF      = 0;  // 500 이상값을 count.
        int highCountB      = 0;  // 500 이상값을 count.
        long preValueF      = 0;
        int  sameCountF     = 0;
        long preValueB      = 0;
        int  sameCountB     = 0;
        int  continueFAULT  = 0;
        int  preContinueFAULT = 0;
        int  noFaultCNT       = 0;
        int  confirmError     = 0;      // Spot성 에러는 제거. 
        int  reportALAM       = 0;      // 상위 REPORT 알람은 더 신중하게
        boolean frontFAN       = true ;  // false : BACK true : FRONT
        int  frontFANCNT      = 0;

        //[**7] 최초 기동시에 PULSE값을 측정해서 관리.
        //      해당 값으로 장애 처리 및 관리.
        boolean       isFirstTime         = true;

        // VAULES
        unsigned int  rpmMINvalueF        = 0;
        unsigned int  rpmMAXvalueF        = 0;
        unsigned int  rpmMINvalueB        = 0;
        unsigned int  rpmMAXvalueB        = 0;

        // THRESHOLD VALUE
        unsigned int  rpmMINthresF        = 0;
        unsigned int  rpmMAXthresF        = 0;
        unsigned int  rpmMINthresB        = 0;
        unsigned int  rpmMAXthresB        = 0;
        
        int           errorContinueCNT    = 0;      // 3회 연속으로 threshold 이하 또는 이상의 값이 나오면,,, 장애 알림.
        int           errorReleaseCNT     = 0;      // 3회 연속으로 normal 상태이면 장애 해제.
        int           verifiedSegnal      = 0;      // 이전값 기억
        int           signalCNT           = 0;      // signal 3회 이상 0 값이 들어와야 
        
        
        // 통계처리 변수
        int   averageCNT                  = 0;      // STATISTICS COUNT
        int   averageTCNT                 = 0;      // STATISTICS COUNT FOR TEMPERATURE
        unsigned long   fFArray[arrayCount];            // Array for FRONT FAN
        unsigned long   fBArray[arrayCount];            // Array for BACK FAN
        float           fTArray[NUM_TEMP_AVERAGE];      // Array for TEMPERATURE, 따로 구성 해야 함 50개 sampling은 너무 많아 (10개 정도로
        
        // Direct ADC 
        unsigned long readSIGNAL;       // reading value MAIN pulse value. 


        // V4 TIMER 관련
        boolean           readPulse       = false;
        long              readPulseTime   = 0.0;
        unsigned long     pulseCNTF       = 0;
        unsigned long     pulseCNTB       = 0;
             
        
        // defines for setting and clearing register bits
        #ifndef cbi
            #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
        #endif
        #ifndef sbi
            #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
        #endif
        
        void setup(){
          
                  //Serial start !! for the first time. 
                  Serial.begin(115200);
        
                  Serial.println("smartFAN controller V10.00......");
                  delay(100);
        
                  #if FAST_ADC   // set prescale to 16     
                    sbi(ADCSRA,ADPS2) ;
                    cbi(ADCSRA,ADPS1) ;
                    cbi(ADCSRA,ADPS0) ;
                  #endif
            
                  
                  //PIN DEFINITION
                  //OUTPUT
                  pinMode(P_LED_ALM,          OUTPUT); 
                  pinMode(P_LED_PWR,          OUTPUT);  
                  pinMode(P_FAN_FAULT,        OUTPUT); 
                  pinMode(P_F_FAN_PWM,        OUTPUT); 
                  pinMode(P_B_FAN_PWM,        OUTPUT); 
                  pinMode(P_CONN_SIGNAL_OUT,  OUTPUT); 
                  
                  //INPUT          
                  pinMode(P_NOW_TEMP,         INPUT); 
                  pinMode(P_CONN_SIGNAL_IN,   INPUT); 
                  pinMode(P_F_FAN_RPM,        INPUT); 
                  digitalWrite(P_F_FAN_RPM,     LOW);
                  pinMode(P_B_FAN_RPM,        INPUT);
                  digitalWrite(P_B_FAN_RPM,     LOW);
                  pinMode(P_UPPERN,           INPUT);
                  digitalWrite(P_UPPERN,       HIGH);    // Pullup settings.
        
                  //INITIATION
                  UP_DOWN = false;
                  fanFAULT = false;
                  digitalWrite(P_FAN_FAULT,     LOW);
                  digitalWrite(P_LED_ALM,       LOW);

                  delay(300);
        
                  // SCENARIO 1 : For the first time BOOTING signal. POWER LED ON
                  // SCENARIO 2 : When it started.Two fan which installed on this should operate full speed (speedFULL) 
        
                  // [**1] UPPER / LOWER STATUS definition changed. FINAL and confirmed.
                  if(digitalRead(P_UPPERN) == HIGH){ //상판에 조립된 경우

                        UP_DOWN = true;
        
                        //SCENARIO 1 (POWER LED ON)
                        digitalWrite(P_LED_PWR,   HIGH);
        
                        //SCENARIO 2 (FAN OPERATION FULL SPEED during 3 SEC)
                        analogWrite(P_F_FAN_PWM,  255);
                        delay(1500);
                        analogWrite(P_B_FAN_PWM,  100); 
                        
                  }else{
                        //SCENARIO 1 (POWER LED blink twice)
                        digitalWrite(P_LED_PWR,   HIGH);
                        delay(500);   
                        digitalWrite(P_LED_PWR,   LOW);
                        delay(500);   
                        digitalWrite(P_LED_PWR,   HIGH);
        
                        //SCENARIO 2 (FAN OPERATION FULL SPEED during 3 SEC)
                        delay(1000);
                        analogWrite(P_F_FAN_PWM,  255);
                        delay(1500);
                        analogWrite(P_B_FAN_PWM,  100); 
                       
                  }
        
                  delay(500); 
                  
                  #if DEBUG
                      Serial.println("SETUP FINISHED......");
                      delay(500);
                      Serial.println("NOW IT STARTED !!!!!");
                  #endif
        
                  //INTERRUPT START
                  #if INTERRUPT
                      pinMode(MY_PIN, INPUT); digitalWrite(MY_PIN, HIGH);
                      Serial.begin(115200);
                      PCintPort::attachInterrupt(MY_PIN, &rising, RISING);
                  #endif

                  // INITIATION for TIMER
                  readPulseTime = millis();
                
        }

        
        
        void loop(){
        
                  float           nowTemp;
                  float           averageFront  = 0;
                  float           averageBack   = 0;
                  unsigned int    ADCValueF;
                  unsigned int    ADCValueB;   
                  double          VoltageF;
                  double          VoltageB;
                  double          Vcc;
                  float           averageTemp;

                  // FINAL
                  //FRONT FAN
                  unsigned long   timeDeltaF    = 0;
                  boolean         isHIGHF       = false;
                  int             highCNTF      = 0;
                  unsigned long   inPulsCNTF    = 0;
                  
                  //BACK FAN
                  unsigned long   timeDeltaB    = 0;
                  boolean         isHIGHB       = false;
                  int             highCNTB      = 0;
                  unsigned long   inPulsCNTB    = 0;
        
                 
        
                  // INIT
                  readSIGNAL= 0.0;

                  // PULSE READING & PROCESSING SEPERATION

                  pulseCNTF = 0;
                  pulseCNTB = 0;
                  readPulseTime = millis();

                  // FAN CALIBRATION & SETTING VALUE REGISTER
                  // JUST 1 time EXECUTE.
                  if(isFirstTime == true) {
                         // MAX VALUE CALCULATION

                         Serial.println(" For the First TIME !!! =============");

                         countPulse(100, 5000);  
                         delay(1000);
                         countPulse(255, 7000);


                         isFirstTime = false;

                  }

                 
                  
                  //Serial.println(" For the First TIME !!! FINISHED =============");
                  while((millis() - readPulseTime) < 2000){
                        
                         

                        // 특정 시간단위 PULSE HIGH COUNT.
                        ADCValueF   = analogRead(P_F_FAN_RPM); // P_B_FAN_RPM
                        ADCValueB   = analogRead(P_B_FAN_RPM); // P_F_FAN_RPM

                        if(ADCValueF > 100 && ADCValueF ) {
                              if(isHIGHF == true ) {
                                    highCNTF ++;
                                    //Serial.print(30);
                              } else {
                                    isHIGHF = true;
                                    highCNTF ++;
                                    timeDeltaF = micros();
                              }
                              
                        } else {
                              if(isHIGHF == true && highCNTF >= 4) {
                                    // FINAL TIME STAMP
                                    // Value INPUT
                                    inPulsCNTF = micros() - timeDeltaF;

                                    #if DEBUG_PULSE
                                          Serial.print("COUNT : ");
                                          Serial.print(highCNTF);
                                          Serial.print(" TIMESTAMP : ");
                                          Serial.print((micros() - timeDeltaF));  
                                          Serial.println(""); 
                                    #endif
                              } isHIGHF = false; 
                              highCNTF = 0;
                              timeDeltaF = micros();
                        }


                         if(ADCValueB > 100 && ADCValueB ) {
                              if(isHIGHB == true ) {
                                    highCNTB ++;
                                    //Serial.print(30);
                              } else {
                                    isHIGHB = true;
                                    highCNTB ++;
                                    timeDeltaB = micros();
                              }
          
                        } else {
                              if(isHIGHB == true && highCNTB >= 4) {
                                    // FINAL TIME STAMP

                                    inPulsCNTB = micros() - timeDeltaB;

                                    #if DEBUG_PULSE
                                            Serial.print("COUNT : ");
                                            Serial.print(highCNTB);
                                            Serial.print(" TIMESTAMP : ");
                                            Serial.print((micros() - timeDeltaB));  
                                            Serial.println(""); 
                                    #endif 
                              } isHIGHB = false; 
                              highCNTB = 0;
                              timeDeltaB = micros();
                        }


                        // STATISTICS for VALUES.

                        fFArray[averageCNT] = inPulsCNTF;
                        for (int tmpCountF = 0; tmpCountF < NUM_AVERAGE; tmpCountF ++) {
                            averageFront += fFArray[tmpCountF];
                        }
                        averageFront  /= NUM_AVERAGE;

                        fBArray[averageCNT] = inPulsCNTB;
                        for (int tmpCountB = 0; tmpCountB < NUM_AVERAGE; tmpCountB ++) {
                            averageBack += fBArray[tmpCountB];
                        }
                        averageBack  /= NUM_AVERAGE;
              
                        if((averageCNT - (NUM_AVERAGE-1)) >= 0) averageCNT = 0;     //Average count reset
                        else averageCNT++;


                  } // while END.

                  
                         
                  
                  // [**9] 하위 모든 로직은 그대로 활용하기 위해 변수에 넣어 줌.
                  pulseCNTF = averageFront;
                  pulseCNTB = averageBack;

                  delay(100);
  
                  // 팬 동작에 대한 Analog READ. 
                  readSIGNAL  =   pulseIn(P_CONN_SIGNAL_IN,HIGH,10000); 

                  // [**4]Read Temperature. Everytime in this loop.
                  nowTemp =  readNowTemp();

        
                  //FAN FAULT DETECTION & RELEASE NEW LOGIC
                  // THRESHOLD
//                    unsigned int  rpmMINthresF        = 0;
//                    unsigned int  rpmMAXthresF        = 0;
//                    unsigned int  rpmMINthresB        = 0;
//                    unsigned int  rpmMAXthresB        = 0;


                   // THERSHOLD 
                   #if PULSE
                      Serial.print("|PULSE[INIT] ");
                      Serial.print("|160 |F|");
                      Serial.print(rpmMINvalueF);
                      Serial.print(" |B|");
                      Serial.print(rpmMINvalueB);
                      Serial.print("| 250 |F|");
                      Serial.print(rpmMAXvalueF);
                      Serial.print(" |B|");
                      Serial.print(rpmMAXvalueB);
                        
                      Serial.print("| pulseCNTF |Value|");
                      Serial.print(pulseCNTF);
                      Serial.print("| MIN T|");
                      Serial.print(rpmMINthresF);
                      Serial.print("| MAX T|");
                      Serial.print(rpmMAXthresF);

                      Serial.print(" |pulseCNTB |Value|");
                      Serial.print(pulseCNTB);
                      Serial.print("| MIN T|");
                      Serial.print(rpmMINthresB);
                      Serial.print("| MAX T|");
                      Serial.print(rpmMAXthresB);
                      Serial.print("| nowTemp |");
                      Serial.print(nowTemp);
                  #endif

//                      rpmMAXvalueF = averageFront;
//                      rpmMAXvalueB = averageBack;
//               
//                      rpmMINvalueF = averageFront;
//                      rpmMINvalueB = averageBack;
                  
                  if(pulseCNTF > rpmMINvalueF || pulseCNTB > rpmMINvalueB ) {
                  
        
                      if(pulseCNTF < rpmMINthresF || pulseCNTF > rpmMAXthresF ) frontFAN = true;
                      else frontFAN = false; 
    

                      // FAN FAULT DETECTED for the first time.
                      if(fanFAULT != true && errorContinueCNT >= 6) {  //[**2] P_FAN_FAULT signal change from PWM type to continuous type
                              fanFAULT = true;   
                               
                              digitalWrite(P_FAN_FAULT, HIGH);
                              digitalWrite(P_LED_ALM,   HIGH);

                              //Error occurred for the first time. Timer start !!
                              durationError = millis();

                              noFaultCNT = 0;
                              
                      } else {
                              errorContinueCNT++;
                              noFaultCNT = 0;    
                      }
        
                  } else {
                          errorContinueCNT = 0;
                          noFaultCNT++;
                  }



                  if(noFaultCNT >= 2 && fanFAULT == true ) { 
        
                          if(fanFAULT == true) { 
                                // 에러 복구로 확인하고, 상태 복귀
                                fanFAULT = false; 
                                   
                                digitalWrite(P_FAN_FAULT, LOW);
                                digitalWrite(P_LED_ALM,   LOW);

                                durationError = 0;
                          }
                  }

        
        
                  #if PULSE
                          Serial.print(" PULSE READING DATA ");
                          Serial.print("| pulseCNTF |");
                          Serial.print(pulseCNTF);
                          Serial.print("| pulseCNTB |");
                          Serial.print(pulseCNTB); 
                          Serial.print("| nowTemp |");
                          Serial.print(nowTemp);
                          Serial.print("| Error TIME |");
                          if(durationError != 0) Serial.print(millis() - durationError);
                          else Serial.print("0");
                  #endif
        
        
                  //[**4] 온도에 따른 작동 방식을 제어함.
                  //      - 기동시에 온도가 20도 이하일 경우 무조건 작동 금지.
                  //      - 20도 이상이면 [한번이라도] flag 설정. 작동 시작.
                  //      - 작동 시작하면 10도 되는 최초 순간까지 작동. 다시 시작 하려면 최소 한번 20도가 되어야 함.
        
                  if(nowTemperature >= 20.0) tempOperation = true;
                  if(nowTemperature  < 10.0 && tempOperation == true) tempOperation = false;
                  
                  
                  //####################### 상판일 경우 #######################
                  if(UP_DOWN){
                            #if DEBUG
                                Serial.print("UPPER BOARD");
                                Serial.print(" | SIGNAL |");
                                Serial.print(readSIGNAL);
                                Serial.print(" | FANF | ");
                                Serial.print(VoltageF);
                                Serial.print(" | isFAIL | ");
                                Serial.print(fanFAULT);
                                Serial.print(" | FANFavr | ");
                                Serial.print(averageFront);
                                Serial.print(" | FANB |");
                                Serial.print(VoltageB);
                                Serial.print(" | FANBavr | ");
                                Serial.print(averageBack);
                                Serial.print(" | ");
                            #else
                                Serial.print("UPPER BOARD");
                            #endif
        
                          // 상판의 경우는 마스터가 되고. 온도에 따른 값을 전달 해 주면 됨
                           
                           
                           int targetRpm = map(nowTemp, tempMIN, tempMAX, speedZERO, speedFULL); //현재의 온도값을 읽어 팬의속도를 결정한다
        
                           // speedFULL option added. When fanFAULT == true.
                           if(readSIGNAL == 0 ||fanFAULT == true) targetRpm = speedFULL;
        
                           //[**4] targetRpm을 10으로 전달함. 하판에 온도때문에 작동 중지를 전달 하기 위함. pulse값을 전달 해야 함. 0과는 구분해야 함.
                           //      70 - 85 사이로 나옴. SIGNAL.
                           if(tempOperation == false) targetRpm = 10;
                           
                           commandFan(targetRpm,UPPER);
                           
                           #if DEBUG
                                Serial.print("COMMAND");
                                Serial.print(" | nowTEMP | ");
                                Serial.print(nowTemp);
                                Serial.print(" | FAN SPEED | ");
                                Serial.print(targetRpm);
                                Serial.print(" | SIGNAL | ");
                                
                                if(fanFAULT) Serial.print(speedZERO);
                                else Serial.print(targetRpm);
                                Serial.print(" | ");
                           #endif
                    
                  //####################### 하판일 경우 #######################
                  }else{
                            
                            #if DEBUG
                                Serial.print("LOWER BOARD");
                                Serial.print(" | nowTEMP | ");
                                Serial.print(nowTemp);
                                Serial.print(" | SIGNAL |");
                                Serial.print(readSIGNAL);
                                Serial.print(" | FANF | ");
                                Serial.print(pulseCNTF);
                                Serial.print(" | FANFavr | ");
                                Serial.print(averageFront);
                                Serial.print(" | FANB |");
                                Serial.print(pulseCNTB);
                                Serial.print(" | FANBavr | ");
                                Serial.print(averageBack);
                                Serial.print(" | ");
                            #else
                                Serial.print("LOWER BOARD |");
                            #endif
        
                            // DEBUG SCENARIO SIGNAL value filtering. 2019.04.15 EMERGENCY!!!
                            // [**4]비교값을 수정해야 함. 10의 펄스값을 확인하고, 적용 해야 함.
                            //if(readSIGNAL > 2000 || 1000 > readSIGNAL) readSIGNAL = 0;

                            // SIGNAL VALUE FILTERING
                            // int preSegnal // 이전값 기억
                            // int signalCNT // signal 3회 이상 0 값이 들어와야
                            
                            
//                            if(readSIGNAL != 0) signalCNT++;
//                            else                signalCNT = 0;
//
//                            if(readSIGNAL == 0 && signalCNT >= 4) verifiedSegnal = readSIGNAL;
//                            else verifiedSegnal = 0;
        
                            //상판이 정상작동 할 경우 상판의 팬속도와 동기화
                            if(readSIGNAL > 0){  
                                int out = map(readSIGNAL, 0, pwmMAX, speedZERO, speedFULL);
          
                                // FAN fail check
                                if(fanFAULT == true) out = speedFULL;
          
                                //[**4] 펄스값이 작동 중지 상태일 경우 스피드 0 으로 전달.
                                //      스피트 10으로 전달된 펄스값의 범위는 70 - 85 사이임.
                                if(readSIGNAL > 70 && readSIGNAL < 85) out = 0;
          
                                // Command to FAN
                                commandFan(out,LOWER);

                                #if DEBUG
                                    Serial.print("NORMAL MODE");
                                    Serial.print(" | ");
                                #else
                                    Serial.print("NORMAL MODE");
                                #endif
                              
                            }else{            //상판 팬 오류 또는 통신오류일경우 풀가동
                                //[**10] 상판 에러일 경우. 온도값을 보고 작동 중지. 10도 이하일 경우.
                                int out = 0; 

                                if(nowTemperature < 10) out = 0;  // 중지
                                else out = speedFULL;
                                
                                commandFan(out ,LOWER);   // 최대로 돈다.
                                  #if DEBUG
                                      Serial.print("ERROR MODE");
                                      Serial.print(" | ");
                                  #else
                                      Serial.print("ERROR MODE");
                                  #endif
                            }
                  } 

                  
                  // 2019.04.15 Just reporting 
                  #if DEBUG
                        Serial.println("");
                  #else
                        Serial.print("|PULSE[INIT] ");
                        Serial.print("|100 |F|");
                        Serial.print(rpmMINvalueF);
                        Serial.print(" |B|");
                        Serial.print(rpmMINvalueB);
                        Serial.print("| 250 |F|");
                        Serial.print(rpmMAXvalueF);
                        Serial.print(" |B|");
                        Serial.print(rpmMAXvalueB);
                        Serial.print("| pulseCNTF |");
                        Serial.print(pulseCNTF);
                        Serial.print("| pulseCNTB |");
                        Serial.print(pulseCNTB); 
                        Serial.print("| nowTemp |");
                        Serial.print(nowTemp);
                        Serial.print("| SIGNAL |");
                        Serial.print(readSIGNAL);
                        Serial.print("|C|");
                        Serial.print(signalCNT);
               
                  #endif
        
                  delay(sysDelay); 

                  Serial.println("");
        }
        
        long readVcc() {
                  // Read 1.1V reference against AVcc
                  // set the reference to Vcc and the measurement to the internal 1.1V reference
                  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
                    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
                  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
                    ADMUX = _BV(MUX5) | _BV(MUX0);
                  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
                    ADMUX = _BV(MUX3) | _BV(MUX2);
                  #else
                    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
                  #endif  
                
                  delay(2); // Wait for Vref to settle
                  ADCSRA |= _BV(ADSC); // Start conversion
                  while (bit_is_set(ADCSRA,ADSC)); // measuring
                
                  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
                  uint8_t high = ADCH; // unlocks both
                
                  long result = (high<<8) | low;
                
                  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
                  return result; // Vcc in millivolts
        }
        
        
        //fan 가동, 1: UPPER , 0:LOWER
        void commandFan(int fan_rpm,int flag) {

                  // change serial. 
                  analogWrite(P_B_FAN_PWM, fan_rpm);
                  analogWrite(P_F_FAN_PWM, fan_rpm);
                 

                  //fanSPEED 변수에 값을 설정, 전역에서 팬 스피트 체크 로직 추가를 위해 [**3]
                  fanSPEED = fan_rpm;
        
                  //상판일 경우 하판과의 동기화를 위해 전송 나의 팬 속도 전송. 상판의 팬 이상일 경우 PWM 데이타를 0으로 제공
                  if(flag == UPPER) {
                        if(fanFAULT == false)   analogWrite(P_CONN_SIGNAL_OUT, fan_rpm);
                        else if(fan_rpm == 10)  analogWrite(P_CONN_SIGNAL_OUT, fan_rpm); //[**4] FAN FAULT 상황이어도 팬작동 정지시에는 해당 정보를 전달 한다.
                        else                    analogWrite(P_CONN_SIGNAL_OUT, speedZERO);
                  } 
                  
                  #if DEBUG_FAN
                        Serial.print("FAN F SPEED : ");
                        Serial.print(fan_rpm);
                        Serial.print("FAN F SPEED : ");
                        Serial.print(fan_rpm);
                        if(flag == UPPER) {
                            Serial.print("PWM FAN F SPEED : ");
                            Serial.print(fan_rpm);
                        }
                        Serial.println("");
                  #endif

                 
\
                  // change serial. 
//                  analogWrite(P_B_FAN_PWM, 250);
//                  analogWrite(P_F_FAN_PWM, 250);
        }
        
        //온도측정
        float readNowTemp(){
                unsigned int  ADCValue;
                double        Voltage;
                double        Vcc;
                float         averageTemp;
                float         averageVolt;
                //int arrayTMP[10] = {21,23,25,27,29,31,33,35,37,40};
                int tmpCNT = 0;

                while( tmpCNT < 100){
                        tmpCNT++;
                        ADCValue = 0.0;
                        Vcc = readVcc()/1000.0;
                        ADCValue = analogRead(P_NOW_TEMP);   
                        Voltage = (ADCValue / 1023.0) * Vcc;
                
                        // RAW DATA의 통계 처리로 바꿈.=================================
                
                         // 누적통계 - TEMPERATURE STATISTICS applying.
                        fTArray[averageTCNT] = Voltage;
                        
                        for (int tmpCountT = 0; tmpCountT < NUM_TEMP_AVERAGE; tmpCountT ++) {
                            averageVolt += fTArray[tmpCountT];
                        }
                        
                        averageVolt  /= NUM_TEMP_AVERAGE;
                  
                        if((averageTCNT - (NUM_TEMP_AVERAGE-1)) >= 0) averageTCNT = 0;     //Average count reset
                        else averageTCNT++; // 누적통계 - TEMPERATURE STATISTICS applying.

                        delay(10);

                }
    
                // 통계처리 완료.=============================================
          
                float temperC = (averageVolt - 0.5) * 100 ; // -tempOffset
                averageTemp = temperC;
        
                // 통계 처리된 온도를 전역변수에 항상 넣어줌.[**4]
                nowTemperature = averageTemp;
        
                if(averageTemp < tempMIN){
                    averageTemp = tempMIN;
                }
                
                if(averageTemp > tempMAX){
                    averageTemp = tempMAX;
                }
        
        
                #if DEBUG_MORE
                  Serial.print("| analog V |");
                  Serial.print(averageVolt);
                  Serial.print("| read TEMP | ARRAY |");
                  for (int tmpCountD = 0; tmpCountD < NUM_TEMP_AVERAGE; tmpCountD ++) {
                      Serial.print(fTArray[tmpCountD]);
                      Serial.print(":"); 
                  }
                #endif
               
                return averageTemp;
                //return arrayTMP[testTMPCNT];
        }

        void countPulse(int rpm, int duration) {

                // FINAL
                //FRONT FAN
                unsigned long   timeDeltaF    = 0;
                boolean         isHIGHF       = false;
                int             highCNTF      = 0;
                unsigned long   inPulsCNTF    = 0;
                
                //BACK FAN
                unsigned long   timeDeltaB    = 0;
                boolean         isHIGHB       = false;
                int             highCNTB      = 0;
                unsigned long   inPulsCNTB    = 0;
          
                unsigned long   averageFront  = 0;
                unsigned long   averageBack   = 0;
                unsigned int    ADCValueF;
                unsigned int    ADCValueB;  

                unsigned int    readPulseTime = millis();

                analogWrite(P_B_FAN_PWM, rpm);
                analogWrite(P_F_FAN_PWM, rpm);

                Serial.println("INIT =======================================");
                
                for(int i=0;i<=30;i++)  {
                  fFArray[i] = 0;
//                  Serial.print("[");
//                  Serial.print(fFArray[i-1]);
//                  Serial.print("]");
//                  Serial.print("[");
//                  Serial.print(fFArray[i]);
//                  Serial.print("]");
                }
//                Serial.println("");
                for(int i=0;i<=30;i++)  {
                  fBArray[i] = 0;
//                  Serial.print("[");
//                  Serial.print(fBArray[i-1]);
//                  Serial.print("]");
//                  Serial.print("[");
//                  Serial.print(fBArray[i]);
//                  Serial.print("]");
                }
//                Serial.println("");
                

                while((millis() - readPulseTime) < duration){

                        // 특정 시간단위 PULSE HIGH COUNT.
                        ADCValueF   = analogRead(P_F_FAN_RPM); // P_B_FAN_RPM
                        ADCValueB   = analogRead(P_B_FAN_RPM); // P_F_FAN_RPM

                        Serial.print(ADCValueF);
                        Serial.print(" ");
                        Serial.print(ADCValueB);

                        if(ADCValueF > 100 ) {
                              if(isHIGHF == true ) {
                                    highCNTF ++;
                                    //Serial.print(30);
                              } else {
                                    isHIGHF = true;
                                    highCNTF ++;
                                    timeDeltaF = micros();
                              }
                              
                        } else {
                              if(isHIGHF == true && highCNTF >= 2) {
                                    // FINAL TIME STAMP
                                    // Value INPUT
                                    inPulsCNTF = micros() - timeDeltaF;

                                    #if DEBUG_PULSE
                                          Serial.print("FFFFFF COUNT : ");
                                          Serial.print(highCNTF);
                                          Serial.print(" TIMESTAMP : ");
                                          Serial.print((micros() - timeDeltaF));  
                                          Serial.println(""); 
                                    #endif
                              } isHIGHF = false; 
                              highCNTF = 0;
                              //timeDeltaF = micros();
                        }


                         if(ADCValueB > 100 ) {
                              if(isHIGHB == true ) {
                                    highCNTB ++;
                                    //Serial.print(30);
                              } else {
                                    isHIGHB = true;
                                    highCNTB ++;
                                    timeDeltaB = micros();
                              }
          
                        } else {
                              if(isHIGHB == true && highCNTB >= 2) {
                                    // FINAL TIME STAMP

                                    inPulsCNTB = micros() - timeDeltaB;

                                    #if DEBUG_PULSE
                                            Serial.print("BBBB COUNT : ");
                                            Serial.print(highCNTB);
                                            Serial.print(" TIMESTAMP : ");
                                            Serial.print((micros() - timeDeltaB));  
                                            Serial.println(""); 
                                    #endif 
                              } isHIGHB = false; 
                              highCNTB = 0;
                              //timeDeltaB = micros();
                        }


                        // STATISTICS for VALUES.

                        fFArray[averageCNT] = inPulsCNTF;
                        for (int tmpCountF = 0; tmpCountF < NUM_AVERAGE; tmpCountF ++) {
                            averageFront += fFArray[tmpCountF];
                        }
                        averageFront  /= NUM_AVERAGE;

                        fBArray[averageCNT] = inPulsCNTB;
                        for (int tmpCountB = 0; tmpCountB < NUM_AVERAGE; tmpCountB ++) {
                            averageBack += fBArray[tmpCountB];
                        }
                        averageBack  /= NUM_AVERAGE;
              
                        if((averageCNT - (NUM_AVERAGE-1)) >= 0) averageCNT = 0;     //Average count reset
                        else averageCNT++;

//                        Serial.print("averageF ");
//                        Serial.print(averageFront);
//                        Serial.print(" . ");
//                        Serial.print(" averageF");
//                        Serial.print(averageBack);
                        Serial.print(" inPulsCNTF");
                        Serial.print(inPulsCNTF);
                        Serial.print(" inPulsCNTB");
                        Serial.print(inPulsCNTB);
//                        Serial.print(" ADCValueF");
//                        Serial.print(ADCValueF);
//                        Serial.print(" ADCValueB");
//                        Serial.print(ADCValueB);
//                        Serial.print(" timeDeltaF");
//                        Serial.print(timeDeltaF);
//                        Serial.print(" timeDeltaB");
//                        Serial.print(timeDeltaB);
                        

                        
                        Serial.println("");

                        delay(1);

                  } // while END


                // VALUE REGISTER
                if(rpm >= speedFULL) {
                      // MAX VALUE REGISTER
                      rpmMAXvalueF = averageFront;
                      rpmMAXvalueB = averageBack;
                } else {
                      // MIN VALUE REGISTER
                      rpmMINvalueF = averageFront;
                      rpmMINvalueB = averageBack;   
                }               
        }
        
        // Timer triggering functions (rising(), falling())
        void rising()
        {
                latest_interrupted_pin=PCintPort::arduinoPin;
                PCintPort::attachInterrupt(latest_interrupted_pin, &falling, FALLING);
                prev_time = micros();
        }
         
        void falling() {
                latest_interrupted_pin=PCintPort::arduinoPin;
                PCintPort::attachInterrupt(latest_interrupted_pin, &rising, RISING);
                pwm_value = micros()-prev_time;
                Serial.println(pwm_value);
        }
         /**
         * Divides a given PWM pin frequency by a divisor.
         * 
         * The resulting frequency is equal to the base frequency divided by
         * the given divisor:
         *   - Base frequencies:
         *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
         *      o The base frequency for pins 5 and 6 is 62500 Hz.
         *   - Divisors:
         *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
         *        256, and 1024.
         *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
         *        128, 256, and 1024.
         * 
         * PWM frequencies are tied together in pairs of pins. If one in a
         * pair is changed, the other is also changed to match:
         *   - Pins 5 and 6 are paired on timer0
         *   - Pins 9 and 10 are paired on timer1
         *   - Pins 3 and 11 are paired on timer2
         * 
         * Note that this function will have side effects on anything else
         * that uses timers:
         *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
         *     millis() functions to stop working. Other timing-related
         *     functions may also be affected.
         *   - Changes on pins 9 or 10 will cause the Servo library to function
         *     incorrectly.
         * 
         * Thanks to macegr of the Arduino forums for his documentation of the
         * PWM frequency divisors. His post can be viewed at:
         *   https://forum.arduino.cc/index.php?topic=16612#msg121031
         */
        void setPwmFrequency(int pin, int divisor) {
          byte mode;
          if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
            switch(divisor) {
              case 1:     mode = 0x01; break;
              case 8:     mode = 0x02; break;
              case 64:    mode = 0x03; break;
              case 256:   mode = 0x04; break;
              case 1024:  mode = 0x05; break;
              default: return;
            }
            if(pin == 5 || pin == 6) {
              TCCR0B = TCCR0B & 0b11111000 | mode;
            } else {
              TCCR1B = TCCR1B & 0b11111000 | mode;
            }
          } else if(pin == 3 || pin == 11) {
            switch(divisor) {
              case 1:     mode = 0x01; break;
              case 8:     mode = 0x02; break;
              case 32:    mode = 0x03; break;
              case 64:    mode = 0x04; break;
              case 128:   mode = 0x05; break;
              case 256:   mode = 0x06; break;
              case 1024:  mode = 0x07; break;
              default: return;
            }
            TCCR2B = TCCR2B & 0b11111000 | mode;
          }
        }

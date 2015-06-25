/*******************************
  Sorter/Cap Manager 
  Original: David Burbach
  Rewrite: Adam Burbach
*******************************/

//                Board: Arduino UNO
//                    Pin Assignment

//  PORT        Arduino   hex    processor pin     
//  PORTB.0        8      0x01       14    Lower Proximity sensor (Pull-up) pcint 0
//  PORTB.1        9      0x02       15    
//  PORTB.2       10      0x04       16    Error LED
//  PORTB.3       11      0x08       17
//  PORTB.4       12      0x10       18    Upper Proximity sensor (Pull-up) pcint 4
//  PORTB.5       13      0x20       19    Motor indicator LED

//  PORTC.0       A0      0x01       23
//  PORTC.1       A1      0x02       24
//  PORTC.2       A2      0x04       25    
//  PORTC.3       A3      0x08       26    
//  PORTC.4       A4      0x10       27
//  PORTC.5       A5      0x20       28

//  PORTD.0        0      0x01        2    
//  PORTD.1        1      0x02        3    
//  PORTD.2        2      0x04        4    Selector input: Manual (Pull-up) pcint 18
//  PORTD.3        3      0x08        5    
//  PORTD.4        4      0x10        6    Motor operation
//  PORTD.5        5      0x20       11    
//  PORTD.6        6      0x40       12    
//  PORTD.7        7      0x80       13    Selector input: Automatic (Pull-up) pcint 23


//////
// Includes 
//////
#include <stdint.h>



#define SREG_MASK            0x80

/*
//////
// Timer 1 definitions
//    16 MHz clock
/////
#define TIMER1_CTRLA_MASK    0x00 // 
#define TIMER1_CTRLB_MASK    0x09 // CTC mode, no clock prescale
//#define TIMER1_CTRLC_MASK    0x00 //
#define TIMER1_COMPAH_MASK   0x80 // 
#define TIMER1_COMPAL_MASK   0x00 // 
#define TIMER1_COMPBH_MASK   0x10 //
#define TIMER1_COMPBL_MASK   0x00 // 
#define TIMER1_IMASK_MASK    0x06 // Interrupt on timer1 comp A and comp B 
*/

//////
// GPIO definitions
//////
#define PORTB_INIT_MASK    0x11
#define DDRB_INIT_MASK     0x24
#define PORTD_INIT_MASK    0x84
#define DDRD_INIT_MASK     0x10

#define ERROR_LED_MASK     0x04
#define MOTOR_MASK         0x10
#define MOTOR_LED_MASK     0x20
#define PROX_LOWER_MASK    0x01
#define PROX_UPPER_MASK    0x10
#define SW_MANUAL_MASK     0x04
#define SW_AUTO_MASK       0x80

//////
// Pin change interrupt definitions
//////
#define PROX_UPPER_PCINT   0x10
#define PROX_LOWER_PCINT   0x01
#define SW_MANUAL_PCINT    0x04
#define SW_AUTO_PCINT      0x80

#define PCICR_INIT         0x05
#define PCINT2_INIT        0x84
#define PCINT1_INIT        0x00
#define PCINT0_INIT        0x11
  
//////
// State Machine Definitions
//////

#define STATE_AUTO        2
#define STATE_MANUAL      1
#define STATE_OFF         0

//////
// Misc. definitions
//////
#define DEBOUNCE_MSEC  20
#define PROX_DELAY     2000
#define PROX_DELAY_3X  6000

/////
// Misc. Interrupt variables 
/////
uint8_t *g_SREG; // interrupt status register

/*
//////
// Timer 1 control, compare, and interrupt registers
//////
uint8_t *g_timer1_ctrlA;  // 
uint8_t *g_timer1_ctrlB;  //  
uint8_t *g_timer1_ctrlC;  //  
uint8_t *g_timer1_compAH; // Output compare A high bytes
uint8_t *g_timer1_compAL; // Output compare A low bytes
uint8_t *g_timer1_compBH; // Output compare B high bytes
uint8_t *g_timer1_compBL; // Output compare B low bytes
uint8_t *g_timer1_iMask;  //
uint8_t *g_timer1_iFlag;  //
*/

//////
// GPIO registers 
//////
volatile uint8_t *g_portB;
volatile uint8_t *g_DDRB;
volatile uint8_t *g_pinB;

volatile uint8_t *g_portD;
volatile uint8_t *g_DDRD;
volatile uint8_t *g_pinD;


//////
// Pin change interrupt registers
//////
volatile uint8_t *g_pcmsk2;
volatile uint8_t *g_pcmsk1;
volatile uint8_t *g_pcmsk0;
volatile uint8_t *g_pcifr;
volatile uint8_t *g_pcicr;


//////
// Global logic variables 
//////

volatile int8_t g_switchState;
volatile int8_t g_state;

uint8_t g_upperProxLastState;
uint8_t g_lowerProxLastState;
uint8_t g_upperProxState;
uint8_t g_lowerProxState;
uint8_t g_swManState;
uint8_t g_swManLastState;
uint8_t g_swAutoState;
uint8_t g_swAutoLastState;

// Timestamp Array
// [0:Upper Prox, 1:Lower Prox, 2: Selector Automatic, 3: Selector Manual, 4: UpperToggle, 5: LowerToggle]
volatile uint32_t g_timeStamps[6];


//////
// Function Prototypes 
//////
int8_t checkSwitch();
int8_t handleAuto();




//////
// Setup 
//////
void setup()
{
  // global interrupt settings
  g_SREG = (uint8_t*) 0x5F;
  
  *g_SREG = (*g_SREG & ~SREG_MASK) | SREG_MASK;
  
  //GPIO setup
  g_portB = (uint8_t*) 0x25;
  g_DDRB = (uint8_t*) 0x24;
  g_pinB = (uint8_t*) 0x23;
  
  *g_portB |= PORTB_INIT_MASK;
  *g_DDRB = DDRB_INIT_MASK;
  
  g_portD = (uint8_t*) 0x2B;
  g_DDRD = (uint8_t*) 0x2A;
  g_pinD = (uint8_t*) 0x29;
  
  *g_portD = PORTD_INIT_MASK;
  *g_DDRD = DDRD_INIT_MASK;
  
  // Pin change interrupt setup
  g_pcicr = (uint8_t*) 0x68;
  g_pcifr = (uint8_t*) 0x3B;
  g_pcmsk2 = (uint8_t*) 0x6D;
  g_pcmsk1 = (uint8_t*) 0x6C;
  g_pcmsk0 = (uint8_t*) 0x6B;
  
  *g_pcicr = PCICR_INIT;
  *g_pcmsk2 = PCINT2_INIT;
  *g_pcmsk1 = PCINT1_INIT;
  *g_pcmsk0 = PCINT0_INIT;
  /*
  // Timer 1 settings
  g_timer1_ctrlA = (uint8_t*) 0x80;
  g_timer1_ctrlB = (uint8_t*) 0x81;
  g_timer1_ctrlC = (uint8_t*) 0x82;
  g_timer1_compAH = (uint8_t*) 0x89;
  g_timer1_compAL = (uint8_t*) 0x88;
  g_timer1_compBH = (uint8_t*) 0x8B;
  g_timer1_compBL = (uint8_t*) 0x8A;
  g_timer1_iMask = (uint8_t*) 0x6F;
  g_timer1_iFlag = (uint8_t*) 0x36;
  
  
  *g_timer1_ctrlA = (*g_timer1_ctrlA & 0x0C) | TIMER1_CTRLA_MASK;
  *g_timer1_ctrlB = (*g_timer1_ctrlB & 0x20) | TIMER1_CTRLB_MASK;
  //*g_timer1_ctrlC = (*g_timer1_ctrlC & 0x3F) | TIMER1_CTRLC_MASK;
  *g_timer1_compAH = (*g_timer1_compAH & 0x00) | TIMER1_COMPAH_MASK;
  *g_timer1_compAL = (*g_timer1_compAL & 0x00) | TIMER1_COMPAL_MASK;
  *g_timer1_compBH = (*g_timer1_compBH & 0x00) | TIMER1_COMPBH_MASK;
  *g_timer1_compBL = (*g_timer1_compBL & 0x00) | TIMER1_COMPBL_MASK;
  *g_timer1_iMask = (*g_timer1_iMask & 0xD8) | TIMER1_IMASK_MASK;
  */
  
  // Timestamp array initialize
  g_timeStamps[0] = 0;
  g_timeStamps[1] = 0;
  g_timeStamps[2] = 0;
  g_timeStamps[3] = 0;
  g_timeStamps[4] = 0;
  g_timeStamps[5] = 0;

  // global logic variable init
  g_lowerProxState = *g_pinB | PROX_LOWER_MASK;
  g_lowerProxLastState = g_lowerProxState;
  g_upperProxState = *g_pinB | PROX_UPPER_MASK;
  g_upperProxLastState = g_upperProxState;
  
  // Poll for state
  g_swAutoState = *g_pinD & SW_AUTO_MASK;
  g_swAutoLastState = g_swAutoState;
  g_swManState = *g_pinD & SW_MANUAL_MASK;
  g_swManLastState = g_swManState;
  
  if (!(g_swManLastState))
    g_state = STATE_MANUAL;
  else if (!(g_swAutoLastState))
    g_state = STATE_AUTO;
  else
    g_state = STATE_OFF;
    
  // Selector switch shows both auto and manual then error
  if (!(g_swAutoLastState) && !(g_swManLastState))
    g_state = -1;
    
  g_switchState = 1;
 
}


//////
// Main Loop 
//////
void loop()
{

  if(g_switchState)
  {
    *g_portB &= ~ERROR_LED_MASK;
    switch (g_state)
    {
      case STATE_AUTO:
        switchToAuto();
      break;
      
      case STATE_MANUAL:
        switchToManual();
      break;
      
      case STATE_OFF:
        switchToOff();
      break;
      
      default:
        //ERROR
        *g_portB |= ERROR_LED_MASK;
      break;
    }
    g_switchState = 0;
  }
  
  if (g_state == STATE_AUTO)
  {
    handleAuto();
  }
  
  checkSwitch();
  
}

//////
// Function implementations 
//////
int8_t checkSwitch()
{
  uint32_t curTime;
  uint32_t delta;
  
  if(!(*g_pcmsk2 & SW_AUTO_PCINT))
  {
    curTime = millis();
    if (curTime >= g_timeStamps[2])
    {
      delta = curTime - g_timeStamps[2];
    }
    else
    {
      delta = 0xFFFFFFFF - g_timeStamps[2] + curTime;
    }
    if (delta >= DEBOUNCE_MSEC)
    {
      g_swAutoState = *g_pinD & SW_AUTO_MASK;
      if(g_swAutoState != g_swAutoLastState)
      {
        g_swAutoLastState = g_swAutoState;
        g_switchState = 1;
      }
      *g_pcmsk2 |= SW_AUTO_PCINT; 
    }    
  }
  
  if(!(*g_pcmsk2 & SW_MANUAL_PCINT))
  {
    curTime = millis();
    if (curTime >= g_timeStamps[3])
    {
      delta = curTime - g_timeStamps[3];
    }
    else
    {
      delta = 0xFFFFFFFF - g_timeStamps[3] + curTime;
    }
    if (delta >= DEBOUNCE_MSEC)
    {
      g_swManState = *g_pinD & SW_MANUAL_MASK;
      if(g_swManState != g_swManLastState)
      {
        g_swManLastState = g_swManState;
        g_switchState = 1;
      }
      *g_pcmsk2 |= SW_MANUAL_PCINT;
    }
  }
  

  if(g_switchState)
  {
    if (!(g_swManLastState))
      g_state = STATE_MANUAL;
    else if (!(g_swAutoLastState))
      g_state = STATE_AUTO;
    else
      g_state = STATE_OFF;
    
    // Selector switch shows both auto and manual then error
    if (!(g_swAutoState) && !(g_swManState))
      g_state = -1;
  }
  return 1;
}

int8_t handleAuto()
{
  uint32_t curTime;
  uint32_t delta;
  
  
  if(!(*g_pcmsk0 & PROX_UPPER_PCINT))//pinFireUpper
  {
    curTime = millis();
    if (curTime >= g_timeStamps[0])
    {
      delta = curTime - g_timeStamps[0];
    }
    else
    {
      delta = 0xFFFFFFFF - g_timeStamps[0] + curTime;
    }
    if(delta >= DEBOUNCE_MSEC)
    {
      g_upperProxState = *g_pinB & PROX_UPPER_MASK;
      if(g_upperProxState != g_upperProxLastState)
      {
        g_upperProxLastState = g_upperProxState;
        g_timeStamps[4] = curTime;
      }
      *g_pcmsk0 |= PROX_UPPER_PCINT;
    }
  }
  
  if(!(*g_pcmsk0 & PROX_LOWER_PCINT))//pinFireLower
  {
    curTime = millis();
    if (curTime >= g_timeStamps[1])
    {
      delta = curTime - g_timeStamps[1];
    }
    else
    {
      delta = 0xFFFFFFFF - g_timeStamps[1] + curTime;
    }
    if (delta >= DEBOUNCE_MSEC)
    {
      g_lowerProxState = *g_pinB & PROX_LOWER_MASK;
      if(g_lowerProxState != g_lowerProxLastState)
      {
        g_lowerProxLastState = g_lowerProxState;
        g_timeStamps[5] = curTime;
      }
      *g_pcmsk0 |= PROX_LOWER_PCINT;
    }
  }
  
  
  curTime = millis();
  // Upper prox sensor covered for two seconds: stop motor
  if(!g_upperProxLastState)
  {
    if(curTime >= g_timeStamps[4])
    {
      delta = curTime - g_timeStamps[4];
    }
    else
    {
      delta = 0xFFFFFFFF - g_timeStamps[4] + curTime;
    }
    if(delta >= PROX_DELAY)
    {
      *g_portD &= ~MOTOR_MASK;
    }
  }
  else if(g_lowerProxLastState)
  {
    if(curTime >= g_timeStamps[5])
    {
      delta = curTime - g_timeStamps[5];
    }
    else
    {
      delta = 0xFFFFFFFF - g_timeStamps[5] + curTime;
    }
    if (delta >= PROX_DELAY)
    {
      *g_portD |= MOTOR_MASK;
    }
    if (delta >= PROX_DELAY_3X)
    {
      *g_portB |= ERROR_LED_MASK;
    }
  }
  return 1;
}

void inline switchToAuto()
{
    *g_portB |= MOTOR_LED_MASK; 
    *g_portD |= MOTOR_MASK;
}


void inline switchToManual()
{
  *g_portD |= MOTOR_MASK;
  *g_portB |= MOTOR_LED_MASK;
  
}

void inline switchToOff()
{
  *g_portB &= ~MOTOR_LED_MASK;
  *g_portD &= ~MOTOR_MASK;
}



//////
// Interrupts 
//////

/*
ISR(TIMER1_COMPA_vect)
{
  
}

ISR(TIMER1_COMPB_vect)
{
  
}
*/
/* Set in TIMER1_IMASK_MASK
ISR(TIMER1_OVF_vect)
{

}
*/

//
ISR(PCINT0_vect)
{
  if((*g_pinB & PROX_UPPER_MASK) != g_upperProxLastState)
  {
    *g_pcmsk0 &= ~PROX_UPPER_PCINT;
    g_timeStamps[0] = millis();
  }
  else
  {
    *g_pcmsk0 &= ~PROX_LOWER_PCINT;
    g_timeStamps[1] = millis();
  }
}

/*
ISR(PCINT1_vect)
{
  
}
*/

ISR(PCINT2_vect)
{
  if((*g_pinD & SW_MANUAL_MASK) != g_swManLastState)
  {
    *g_pcmsk2 &= ~SW_MANUAL_PCINT;
    g_timeStamps[3] = millis();
  }
  else
  {
    *g_pcmsk2 &= ~SW_AUTO_PCINT;
    g_timeStamps[2] = millis();
  }
}
/*
ISR(USART_RX_vect)
{
  
}

ISR(USART_TX_vect)
{
  
}

ISR(USART_UDRE_vect)
{
  
}
*/

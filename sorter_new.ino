 
/**************************
       RESERVED PINS     
None

***************************/


#define SREG_MASK            0x80

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

/////
// Misc. Interrupt variables 
/////
unsigned char *g_SREG; // interrupt status register

//////
// Timer 1 control, compare, and interrupt registers
//////
unsigned char *g_timer1_ctrlA;  // 
unsigned char *g_timer1_ctrlB;  //  
unsigned char *g_timer1_ctrlC;  //  
unsigned char *g_timer1_compAH; // Output compare A high bytes
unsigned char *g_timer1_compAL; // Output compare A low bytes
unsigned char *g_timer1_compBH; // Output compare B high bytes
unsigned char *g_timer1_compBL; // Output compare B low bytes
unsigned char *g_timer1_iMask;  //
unsigned char *g_timer1_iFlag;  //

void setup()
{
  // global interrupt settings
  g_SREG = (unsigned char*) 0x5F;
  
  *g_SREG = (*g_SREG & ~SREG_MASK) | SREG_MASK;
  
  // Timer 1 settings
  g_timer1_ctrlA = (unsigned char*) 0x80;
  g_timer1_ctrlB = (unsigned char*) 0x81;
  g_timer1_ctrlC = (unsigned char*) 0x82;
  g_timer1_compAH = (unsigned char*) 0x89;
  g_timer1_compAL = (unsigned char*) 0x88;
  g_timer1_compBH = (unsigned char*) 0x8B;
  g_timer1_compBL = (unsigned char*) 0x8A;
  g_timer1_iMask = (unsigned char*) 0x6F;
  g_timer1_iFlag = (unsigned char*) 0x36;
  
  
  *g_timer1_ctrlA = (*g_timer1_ctrlA & 0x0C) | TIMER1_CTRLA_MASK;
  *g_timer1_ctrlB = (*g_timer1_ctrlB & 0x20) | TIMER1_CTRLB_MASK;
  //*g_timer1_ctrlC = (*g_timer1_ctrlC & 0x3F) | TIMER1_CTRLC_MASK;
  *g_timer1_compAH = (*g_timer1_compAH & 0x00) | TIMER1_COMPAH_MASK;
  *g_timer1_compAL = (*g_timer1_compAL & 0x00) | TIMER1_COMPAL_MASK;
  *g_timer1_compBH = (*g_timer1_compBH & 0x00) | TIMER1_COMPBH_MASK;
  *g_timer1_compBL = (*g_timer1_compBL & 0x00) | TIMER1_COMPBL_MASK;
  *g_timer1_iMask = (*g_timer1_iMask & 0xD8) | TIMER1_IMASK_MASK;
}

void loop()
{
  
}


ISR(TIMER1_COMPA_vect)
{
  
}

ISR(TIMER1_COMPB_vect)
{
  
}

/* Set in TIMER1_IMASK_MASK
ISR(TIMER1_OVF_vect)
{

}
*/
ISR(USART_RX_vect)
{
  
}

ISR(USART_TX_vect)
{
  
}

ISR(USART_UDRE_vect)
{
  
}

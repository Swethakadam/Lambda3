Integrated code

https://www.tinkercad.com/things/jXeQFt95PXg-bcmintegratedlambda3/editel

#include<avr/io.h>
#include<avr/interrupt.h>
#include<stdint.h>
#include <util/delay.h>
#define adcpin 0
#define SET_BIT(PORT,BIT) PORT |=(1<<BIT)
#define CLR_BIT(PORT,BIT) PORT &= ~(1<<BIT)
#define SET_BIT(PORT, PIN) PORT |= (1<<PIN)
#define CLR_BIT(PORT, PIN) PORT &= ~(1<<PIN)

//Door lock
struct 
{
volatile unsigned int FLAG_ISR1:1;
}FLAG1_BIT;
volatile uint16_t us_value=0;
volatile uint16_t *pus_value;
typedef void (*funpointer)(void);
funpointer array_fp[3]={extint1,adc_init};
//Power window
unsigned int  flag=0;//GLOBAL VARIABLE DECLERATION

struct

{

volatile unsigned int ISR1:1; // Switch up interrupt

volatile unsigned int ISR2:1; // Switch down interrupt

}FLAG;
//Wiper system
struct{
  		volatile unsigned int FLAG:1;
}FLAG_BIT;
uint16_t value;


//Functions
//Door lock functions
void extint1()
{
EICRA|=(1<<ISC10);
EICRA&=~(1<<ISC11);
EIMSK|=(1<<INT1);
  sei();
}


void adc_init()
{
  ADMUX=0x00;
  ADMUX |=(1<<REFS0);
  ADCSRA|=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE);
  ADCSRA|=(1<<ADSC);
  while(ADCSRA & (1<<ADSC));
 sei();
  
}

//Wiper system functions
uint16_t adc_read(uint8_t adc)
{
  ADMUX |= (1<<REFS0);
  ADMUX|=(adc & 0x0f);
  ADCSRA|=(1<<ADSC);
    while(ADCSRA &(1<<ADSC))
  {
  }
  return ADC;
}

//Main door lock function
void door()
{
if(FLAG1_BIT.FLAG_ISR1==1)
  {
  cli();
  array_fp[1]();
   }
  else if(FLAG1_BIT.FLAG_ISR1==0) 
   {
    int PIN_READ=0x00;
  PIN_READ=PINB;
        //SET_BIT(PORTD,PD5);

  if(PIN_READ & (1<<PB1)){ 
    //_delay_ms(1000);
    CLR_BIT(PORTB,PB3);
    CLR_BIT(PORTB,PB2); 
    
  }
    else
    {
     SET_BIT(PORTB,PB3);
    CLR_BIT(PORTB,PB2);
    }
    
  
  }
   else{
      CLR_BIT(PORTB,PB2);
     CLR_BIT(PORTB,PB3);
   }
      
  }

//main wiper system function
void wiper( )
{
while(FLAG_BIT.FLAG==1)
    {
      PORTD|=(1<<PD7);
      ADCSRA|=(1<<ADEN);
	  value=adc_read(adcpin);
      Serial.println(value);
      if(value<=100)
      {
        OCR0B=0;   //Wiper off,motor speed is 0,rain intensity is 0
      }
      else if(value>100 && value<=250)
      {
        OCR0B=64;  //DC motor at low rpm,rain intensity is low
      }
      else if(value>250 && value<=750)
      {
        OCR0B=128;   //DC motor at medium rpm,rain intensity is medium
      }
      else //if(value>750)
      {
        OCR0B=192;  //DC motor at high rpm,rain intensity is high
      }
    }
    PORTD&=~((1<<PD6)|(1<<PD7));
  }


//main power window function
void window()
{
   if((FLAG.ISR1==0) && (FLAG.ISR2==0)) 

    {

      CLR_BIT(PORTD,PORTD4);     

      CLR_BIT(PORTD,PORTD5);

    }

 //DC motor rotates clockwise, window moves upward

    else if ((FLAG.ISR1==1) && (FLAG.ISR2==0))

    {

      PORTD |=(1<<PD5);                   

    }
 //DC motor rotates ant-clockwise, window moves downward

else if((FLAG.ISR1==0) && (FLAG.ISR2==1))

      {

        PORTD |=(1<<PD4);

      }

}

// main sunroof function

void sunroof()
{
if (((PINC&(1<<PC2))) && (!(PINC&(1<<PC3))))
        {
           SET_BIT(PORTB, 5);
        }
      else
      {
        CLR_BIT(PORTB, 5);
      }
}

//main function
int main()
{
sei();
    //door lock
pus_value=&us_value;
    SET_BIT(DDRB,PB2); //GREEN LED  LOCKDOOR_INDICATION     
    SET_BIT(DDRB,PD3); //YELLOW LED UNLOCKDOOR_INDICATION
    CLR_BIT(PORTB,PB2);        
    CLR_BIT(PORTB,PB7); 
    
    SET_BIT(DDRB,PB1);
 //CLR_BIT(DDRB,PB1);
    Serial.begin(9600);
  	array_fp[0]();

//power window
  PCICR|=(1<<PCIE2);
  PCMSK2|=(1<<PCINT17);
  PCICR|=(1<<PCIE0);
  PCMSK0|=(1<<PCINT0);
  //Interrupt 0
  EICRA &=~(1<<ISC01);
  EICRA |=(1<<ISC00);
  EIMSK|=(1<<INT0); //Enable interrupt 
 //Interrupt 
  EICRA &=~(1<<ISC11);
  EICRA |=(1<<ISC10);
  EIMSK|=(1<<INT1); //Enable interrupt 
  PCICR|=(1<<PCIE0);
  PCMSK0|=(1<<PCINT4);
  SREG |=(1<<7);   //Enable global interrupts
  
  // wiper system

  DDRD|=((1<<PD6));
  DDRD|=((1<<PD7));   //Motor and LED
  DDRD&=~(1<<PB0);             //Switch
  PORTD&=~(1<<PB0);
  PORTD&=~(1<<PD6);            //Oscilloscope
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT8 to trigger an interrupt on state change


// sunroof
CLR_BIT(DDRC, 2);
  	SET_BIT(PORTC,2);
    CLR_BIT(DDRC, 3);
  	SET_BIT(PORTC,3);
   SET_BIT(DDRB, 5);


    while(1)
  {
    if(flag==1)
    {
          door();
    window();
      wiper();
      sunroof();
    }
  }
}

     //ISR
//Door lock
ISR(INT1_vect)
            {
   			cli();
            FLAG1_BIT.FLAG_ISR1=!FLAG1_BIT.FLAG_ISR1;
            sei(); 
            }

ISR(ADC_vect)
{ 
if(ADC<=200)
{
     _delay_ms(10000);
      SET_BIT(PORTB,PB2);
     CLR_BIT(PORTB,PB3);
   }
    else if(ADC>200)
    {
     
      _delay_ms(5000);
      //SET_BIT(PORTD,PD5);
      SET_BIT(PORTB,PB2);
        CLR_BIT(PORTB,PB3);
      }
  else
  {
    CLR_BIT(PORTB,PB2);
      CLR_BIT(PORTB,PB3);
  }

  
}

//ISR power window
ISR(PCINT2_vect) //INTERRUPT FUNCTION
{
 flag=!flag;//TOGGLEING THE VALUE OF FLAG
}



// Window Up ISR
ISR(INT0_vect)
{
FLAG.ISR1=!FLAG.ISR1;
}

//ISR(PCINT0_vect)
//{
   //FLAG.ISR2=!FLAG.ISR2;
//}


//ISR wiper system
ISR(PCINT0_vect)
{
  cli();
  FLAG.ISR2=!FLAG.ISR2;
  if(FLAG_BIT.FLAG==1)
  {
    FLAG_BIT.FLAG=0;
  	TCCR0B=0x00;
  }
  else 
  {
    FLAG_BIT.FLAG=1;
    TCNT0=0x00;
  	TCCR0A|=0x00;
    TCCR0B|=((1<<CS02)|(1<<CS00));
    TIMSK0|=((1<<OCIE0A)|(1<<OCIE0B));
    OCR0A=255;
  }
}

ISR(TIMER0_COMPA_vect)
{
  PORTD|=(1<<PD6);
}

ISR(TIMER0_COMPB_vect)
{
  PORTD&=~(1<<PD6);
}



#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define NUMBER_OF_COMPARE_MTACHES_PER_SECOND 1
///////////////functions/////////////////
void Increment_Mode(void);
void Decrement_Mode(void);
void Count_UP_Mode(void);
void Count_Down_Mode(void);
void Timer1_CTC_Init(void);
void Init_sevensegment(void);
void LED_Init(void);
void Time_Adjustment(void);
void Alarm_Init(void);
void Toggle_mode_Init(void);
void INT0_Init(void);
void INT1_Init(void);
void INT2_Init(void);
///////////////////////////////////////////
unsigned char g_tick=0;

unsigned char flag=0, flag1 = 0 ,flag2 = 0 , flag3 = 0;

unsigned char sec = 0 , sec_digit1 = 0 , sec_digit2 = 0;
unsigned char min = 0 , min_digit1 = 0 , min_digit2 = 0;
unsigned char hr = 0 ,hr_digit1 = 0 , hr_digit2 = 0;

int main(void)
{
	//Intializations
	Timer1_CTC_Init();
	Alarm_Init();
	Toggle_mode_Init();
	LED_Init();
	INT0_Init();              // Enable external INT0 (reset)
	INT1_Init();              // Enable external INT1 (pause)
	INT2_Init();              // Enable external INT1 (Resume)
	Time_Adjustment();
	Init_sevensegment();

	SREG|=(1<<7);

    while(1)
    {
    	if(!(PINB & (1<<PB7)))
    	{
    		_delay_ms(15);
    		if(!(PINB & (1<<PB7)))
    		{
    			if(flag==0)
    			{
    			    flag3^=1;
    			    PORTD^=(1<<PD4);  //RED Led
    			    PORTD^=(1<<PD5); //YELLOW Led
    			    flag=1;
    			}
    		}
    	}
    	else
    	{
    		flag=0;
    	}
    	Decrement_Mode();
    	Increment_Mode();

    	sec_digit2=sec/10;
    	sec_digit1=sec % 10;

    	min_digit2=min/10;
   		min_digit1=min % 10;

   		hr_digit2=hr/10;
   		hr_digit1=hr % 10;

    	PORTA=(1<<PA5);
    	PORTC = (PORTC & 0xF0) | (sec_digit1 & 0x0F);
    	_delay_ms(2);

    	PORTA=(1<<PA4);
    	PORTC = (PORTC & 0xF0) | (sec_digit2 & 0x0F);
       	_delay_ms(2);

       	PORTA=(1<<PA3);
       	PORTC = (PORTC & 0xF0) | (min_digit1 & 0x0F);
       	_delay_ms(2);

       	PORTA=(1<<PA2);
       	PORTC = (PORTC & 0xF0) | (min_digit2 & 0x0F);
       	_delay_ms(2);

       	PORTA=(1<<PA1);
       	PORTC = (PORTC & 0xF0) | (hr_digit1 & 0x0F);
       	_delay_ms(2);

       	PORTA=(1<<PA0);
       	PORTC = (PORTC & 0xF0) | (hr_digit2 & 0x0F);
       	_delay_ms(2);
    }
}

void Increment_Mode(void)
{
	if(!(PINB & (1<<PB6))) //seconds Increment
		{
			_delay_ms(10);
			if(!(PINB & (1<<PB6)))
			{
			    if(flag1 == 0)
			    {
			        sec++;
			        if(sec==60)
			        {
				    	sec=0;
			        	min++;
			        }
			        if(min==60)
			        {
			        	min=0;
			        	hr++;
			        }
			        if(hr==24)
				    {
			    	    hr=0;
				    }
			        flag1=1;
			    }
		    }
		}
		else if(!(PINB & (1<<PB4))) //Minutes Increment
		{
			_delay_ms(10);
			if(!(PINB & (1<<PB4)))
			{
			    if(flag1 == 0)
			    {
			        min++;
			        if(min==60)
			        {
			        	min=0;
			        	hr++;
			        }
			        if(hr==24)
			        {
			        	hr=0;
			        }
			        flag1=1;
			    }
		    }
		}
		else if(!(PINB & (1<<PB1))) //hours Increment
		{
			_delay_ms(10);
			if(!(PINB & (1<<PB1)))
			{
			    if(flag1 == 0)
			    {
			        hr++;
			        if(hr==24)
			        {
			        	hr=0;
			        }
			        flag1=1;
			    }
		    }
		}
		else
		{
			// button is released reset the button flag to value 0 again.
			flag1 = 0;
		}
}
void Decrement_Mode(void)
{
	if(!(PINB & (1<<PB5))) //seconds Decrement
	{
		_delay_ms(10);
		if(!(PINB & (1<<PB5)))
		{
			if(flag2 == 0)
			{
				if(sec!=0)
				{
					sec--;
				}
				else if(sec==0 && min!=0)
			    {
			    	min--;
	 		    	sec=59;
			    }
				else if(sec==0 && min==0 &&hr!=0)
			    {
					sec=59;
			    	min=59;
			    	hr--;
			    }
			    flag2=1;
			}
		}
	}
	else if(!(PINB & (1<<PB3))) //Minutes Decrement
	{
		_delay_ms(10);
		if(!(PINB & (1<<PB3)))
		{
			if(flag2 == 0)
			{
				if(min>0)
				{
					min--;
				}
				else if(min==0 && hr!=0)
			    {
			    	min=59;
			    	hr--;
			    }
				else if(min==0 && hr==0)
				{
					min=0;
				}
			    flag2=1;
			}
		}
		}
		else if(!(PINB & (1<<PB0))) //hours Decrement
		{
			_delay_ms(10);
			 if(!(PINB & (1<<PB0))){
			if(flag2 == 0)
			{
				if(hr>0)
				{
					hr--;
				}
				else if(hr==0)
				{
					hr=0;
				}
			    flag2=1;
			}
		}
		}
		else
		{
			// button is released reset the button flag to value 0 again.
			flag2 = 0;
		}
}
void Count_UP_Mode(void)
{
	sec++;
	if(sec>59)
	{
		sec=0;
		min++;
	}
	if(min>59)
	{
		min=0;
		hr++;
	}
	if(hr>24)
	{
		hr=0;
	}
}

void Count_Down_Mode(void)
{
	if(sec>0)
	{
	    sec--;
	}
	else if(sec==0 && min>0)
	{
	    min--;
	    sec=59;
	}
	else if(sec==0 && min==0 && hr>0)
	{
		hr--;
		min=59;
		sec=59;
	}
	if(sec==0 && min==0 && hr==0)
	{
		PORTD |= (1<<PD0); //BUZZER ON
	}
}
ISR(TIMER1_COMPA_vect)
{
	g_tick++;
	if(g_tick==NUMBER_OF_COMPARE_MTACHES_PER_SECOND)
	{
		if(flag3==0)
		{
			Count_UP_Mode();
			g_tick=0;
		}
		else
		{
			Count_Down_Mode();
			g_tick=0;
		}
	}
}
void Timer1_CTC_Init(void)
{
	TCNT1 = 0;                          // initial value for timer
	/* Configure timer control register TCCR1A
	 * 1. Disconnect OC1A and OC1B  COM1A1=0 COM1A0=0 COM1B0=0 COM1B1=0
	 * 2. FOC1A=1 FOC1B=0
	 * 3. CTC Mode WGM10=0 WGM11=0 (Mode Number 4)
	 */
	TCCR1A |= (1<<FOC1A);
	/* Configure timer control register TCCR1B
	 * 1. CTC Mode WGM12=1 WGM13=0 (Mode Number 4)
	 * 2. Prescaler = F_CPU/1024 CS10=1 CS11=0 CS12=1
	 */
	TCCR1B |= (1<<WGM12) | (1<<CS12) |(1<<CS10);
	OCR1A=15625;                         //set compare value
	TIMSK |= (1<<OCIE1A);                 // Enable Timer1 Compare A Interrupt
	TIFR |= (1<<OCF1A);
}

void Init_sevensegment(void)
{
	DDRC |= (0x0F);       // first 4 pins in PORTC output (sevensegments)
	PORTC &= ~(0xF0);
	DDRA |= (0x3F);       //first 6 pins in PORTA are enable for the seven segments

}
void LED_Init(void)
{
	DDRD |= (1<<PD4);          // LED Indicators:Counting Up LED (Red)
	DDRD |= (1<<PD5);          // LED Indicators:Counting Down LED (yellow)
	PORTD|=(1<<PD4);           //RED Led on
	PORTD&=~(1<<PD5);          //YELLOW Led off
}
void Time_Adjustment(void)
{
	DDRB &= ~(1<<PB1);         // Hours Adjustment: Increment Hours
	DDRB &= ~(1<<PB0);         // Hours Adjustment: Decrement Hours
	DDRB &= ~(1<<PB4);         // Minutes Adjustment: Increment Minutes
	DDRB &= ~(1<<PB3);         // Minutes Adjustment: Decrement Minutes
	DDRB &= ~(1<<PB6);         // Seconds Adjustment: Increment Seconds
	DDRB &= ~(1<<PB5);         // Seconds Adjustment: Decrement Seconds
	// ALL by Internal pull up
	PORTB |=(1<<PB0) | (1<<PB1) | (1<<PB3) |(1<<PB4) | (1<<PB5) | (1<<PB6);
}
void Alarm_Init(void)
{
	DDRD |= (1<<PD0);          // Alarm/Buzzer
	PORTD &= ~(1<<PD0);
}
void Toggle_mode_Init(void)
{
	DDRB &= ~(1<<PB7);         // Mode Toggle Button
	PORTB |=(1<<PB7) ;         // Internal pull up
}
void INT0_Init(void)
{
	DDRD &= ~(1<<PD2);                   // Reset Button
	PORTD |=(1<<PD2);                    // Internal pull up
	MCUCR |= (1<<ISC01);                 // Trigger INT0 with the falling edge
	GICR  |= (1<<INT0);                  // Enable external interrupt pin INT0
}
ISR(INT0_vect)
{
	sec=0 ;
	sec_digit1 = 0 ;
	sec_digit2 = 0;
	min = 0 ;
	min_digit1 = 0 ;
	min_digit2 = 0;
	hr = 0 ;
	hr_digit1 = 0 ;
	hr_digit2 = 0;
}
void INT1_Init(void)
{
	DDRD&=~(1<<PD3);                     // Pause Button
	MCUCR |= (1<<ISC11) | (1<<ISC10);    // Trigger INT1 with the raising edge
	GICR  |= (1<<INT1);    // Enable external interrupt pin INT1
}
ISR(INT1_vect)
{
	TCCR1B &= ~(1<<CS10) & ~(1<<CS11) & ~(1<<CS12);  // No clock source (Timer/Counter stopped)
}
void INT2_Init(void)
{
	DDRB&=~(1<<PB2);                     // Resume Button
	PORTB |=(1<<PB2);                    //Internal pull up
	MCUCSR &= ~(1<<ISC2) ;               // Trigger INT1 with the falling edge
	GICR  |= (1<<INT2);                  // Enable external interrupt pin INT2
}
ISR(INT2_vect)
{
	TCCR1B |= (1<<CS10) | (1<<CS12);     //activate F_CPU/1024 CS10=1 CS11=0 CS12=1
}


#define F_CPU 8000000UL // Clock Speed#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
//PB5 - RESET
//PB3 - SCK
//PB4 - DO - Thermistor
//GND


//VCC
//PB2 - potentiometer
//PB1 - LATCH
//PB0 - Fan

//#define BAUD 9600
//#define MYUBRR F_CPU/16/BAUD-1
//PC1 - Thermistor
//PD6 - PWM

/*void USART_Init( unsigned int ubrr){
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;//setup usart bits
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);//tx Rx enable, RX interrupt enable
}*/
uint8_t REF = 30,ovf_counter=0,ovf_flag=0,sregctr=0;
uint8_t receivedchars[] = {0,0,0,0};//Pc = 60
const uint8_t 	kp 	= 50,//kc = 3.5
				ki 	= 1,
				kd 	= 30;
int16_t p=0,i=0,d=0;//Several methods for tuning the PID loop exist. The choice of method will depend largely on whether the process can be taken off-line for tuning or not. Ziegler-Nichols method is a well-known online tuning strategy. The first step in this method is setting the I and D gains to zero, increasing the P gain until a sustained and stable oscillation (as close as possible) is obtained on the output. Then the critical gain Kc and the oscillation period Pc is recorded and the P, I, and D values adjusted accordingly using the table below.
//Kc = critical gain
//Pc = oscillation period (~10s)
//		Kp		Ti		Td
//P		0.5*Kc	/		/
//PD	0.5*Kc	/		0.12Pd
//PI	0.45Kc	0.85Pc	/
//PID	0.65Kc	0.5Pc	0.12Pd	

void latch(void){//function used in miltiplexing the 7seg display through the shift register
	PORTB |= (1<<PB1);
	asm("nop");
	PORTB &= ~(1<<PB1);
}
void Timer_Init(void){//
	TCCR0A = (1<<COM0A1)|(1<<WGM01)|(1<<WGM00);//fast pwm on timer0
	TCCR0B = (1<<CS01)|(1<<CS00);//585.9375
	OCR0A = 0;//Timer0 PWM value init
	TIMSK0 = (1<<TOIE0);//OVF EN

	//TCCR1B = (1<<CS11)|(1<<CS12)|(1<<CS13); //timer 1 clockdiv 
	//OCR1A = (1<<7)|(1<<6);//timer 1 overflow rate high byte
	//TIMSK |= (1<<OCIE1A);//timer 1 interrupt enable
	
}/*
void ADC_init(void){//
	ADMUX = (1<<MUX0)|(1<<ADLAR);//ADC pin PB2
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADIE);//ADC EN, ~125 KHz ADC, ADC INT EN
	//ADCSRB = (1<<ADTS2);//manual interrupt
}*/
void IO_init(void){
	DDRB = 0xFB;
}
uint8_t readADC_potentiometer(void){
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADIE);//ADC EN, ~125 KHz ADC, ADC INT EN
	ADMUX = (1<<MUX0)|(1<<ADLAR);//ADC pin PB2
	ADCSRA |= (1<<ADSC);
	asm("SEI");//set up interrupts
	MCUCR = (1<<SE);//sleep 	enable
	asm("sleep");//sleep
	ADCSRA = 0;
	return ADCH;
}
uint8_t readADC_thermistor(void){

	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADIE);//ADC EN, ~125 KHz ADC, ADC INT EN
	PORTB &= ~(1<<PB4);//no pullup
	DDRB &= ~(1<<PB4);//input
	ADMUX = (1<<MUX1)|(1<<ADLAR);//ADC pin PB2
	ADCSRA |= (1<<ADSC);
		
	asm("SEI");//set up interrupts
	MCUCR = (1<<SE);//sleep enable
	asm("sleep");//sleep

	DDRB |= (1<<PB4);
	ADCSRA = 0;
	return ADCH;
}
void send(uint8_t input){//transmit byte to the shift register
	for(uint8_t i=0;i<8;i++){//run 8 times
		if((input & 0x01)== 0x01)//if 1, set output to 1
			PORTB |= (1<<PB4);//data set
		else
			PORTB &= ~(1<<PB4);//else 0
		input = input>>1;//shift over 1
		PORTB |= (1<<PB3);//turn on clock
		PORTB &= ~(1<<PB3);//turn off clock
	}
}	


uint8_t PID_Error_PWM(int8_t error[2]){
	uint16_t total;//made 16 bit so there's no overflows
	
	p =  kp*error[0];//P term
	if(p >= 255)
		p = 255;//no overflows or underflows
	if(p < 0)
		p=0;
		
	i = i + ki*error[0];//I term
	if(i >= 200)
		i = 150;//no overflows or underflows
	else if(i < 0)
		i=0;
		
	d =  kd * (error[1]-error[0]);//d term
	if(p+i == 0)
		d=0;
	total = p+i+d;
	if(total >= 255)//no overflows
		return 255;
	return (uint8_t)total;//mask top 8 bits
}
//0 = 0xFC	5 = 0xB6	
//1 = 0x60	6 = 0xBE
//2 = 0xDA	7 = 0xE0
//3 = 0xF2	8 = 0xFE
//4 = 0x66	9 = 0xF6
uint8_t numtodisp(uint8_t disp){
	if(disp == 1)return 0x60;
	else if(disp == 2)return 0xDA;
	else if(disp == 3)return 0xF2;
	else if(disp == 4)return 0x66;
	else if(disp == 5)return 0xB6;
	else if(disp == 6)return 0xBE;
	else if(disp == 7)return 0xE0;
	else if(disp == 8)return 0xFE;
	else if(disp == 9)return 0xF6;
	return 0xFC;
}
void temp_to_disp(uint8_t temp,uint8_t bit){//bit=0,2
	
	uint8_t counter = 0;
	while(temp >= 10){
		counter++;
		temp-=10;
	}		
	receivedchars[bit] = numtodisp(counter);
	receivedchars[bit+1] = numtodisp(temp);
	

}


uint8_t ADCtoTEMP(uint8_t ADCval){
		return (uint8_t)(((uint16_t)105*ADCval)/256 - 17);//19 at end//let the comiler decide on whether to shift, swap nibbles, or something crazy, etc. 105/256 was picked because 0.41 is needed, and division should be base 2
}
ISR(TIM0_OVF_vect){//585.9375 hz
	if(ovf_counter >= 117){//5 hz
		ovf_flag = 1;
		ovf_counter = 0;
	}else
		ovf_counter++;
	if(sregctr == 0x80){//fancy thing done here, sregctr is the second thing sent. It acts as the cathode on the 7seg, and is also used for deciding which digit is sent
		send( receivedchars[2]);
	}else if(sregctr == 0x40){
		send( receivedchars[3]);
	}else if(sregctr == 0x20){
		send( receivedchars[0]);
	}else if(sregctr == 0x10){
		send( receivedchars[1]);
	}else
		sregctr = 0x08;
	send(~sregctr);//send the inverse of sregctr
	latch();//latch the shift register, displaying the values.
	if(sregctr == 0x80)//if it is on the 4th digit, set it as just before the 1st digit
		sregctr = 0x08;

		sregctr = sregctr<<1;//shift over sregctr
}

ISR(ADC_vect){}
uint8_t PottoTemp(uint8_t pot){
	return pot>>2;//0 to 65c, quite linear
}



int main(void){

	DDRB = 0xFF;

	Timer_Init();//init timer

	IO_init();
	uint8_t realTemp[]={0,0},goalTemp_filtered = 0,realTemp_ADC = 0,goalTemp_ADC = 0,goalTemp[] = {0,0},realTemp_filtered=0;
	int8_t error[] = {0,0};
	while(1){
		asm("SEI");//set up interrupts
		MCUCR = (1<<SE);//sleep enable
		asm("sleep");//sleep
		
		goalTemp_ADC = readADC_potentiometer();
		realTemp_ADC = readADC_thermistor();
		goalTemp[0] = goalTemp[1];
		goalTemp[1] = PottoTemp(goalTemp_ADC);
		goalTemp_filtered = (goalTemp[0] + goalTemp[1])>>1; 
		
		realTemp[0] = realTemp[1];
		realTemp[1] = ADCtoTEMP(realTemp_ADC);//convert to temperature
		realTemp_filtered = (realTemp[0] + realTemp[1])>>1; 
		temp_to_disp(goalTemp_filtered,0);
		temp_to_disp(realTemp_filtered,2);
		
		if(ovf_flag){
		ovf_flag=0;
		
		error[0] = realTemp_filtered - goalTemp_filtered;//save error
		OCR0A = PID_Error_PWM(error);//convert to PWM value using PID
		
		error[1] = error[0];
		}
	}
	return 0;
}


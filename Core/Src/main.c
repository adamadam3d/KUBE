#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include <math.h>
#define direction 1
#define enable 7
#define brake 4
#define RAD_TO_DEG 57.2957795
float roll=0;
float pitch=0;
float yaw=0;
float rollCal=0;
float pitchCal=0;
float X=0;
float Y=0;
float Z=0;
int a=0;
int16_t AccX=0;
int16_t AccY=0;
int16_t AccZ=0;
int16_t err=0;
int16_t KP=15;

void delay (int x){
	TIM2->PSC=x-1;
	TIM2->ARR=8000-1;
	TIM2->CR1|=0b1;
	while(!(TIM2->SR&0b1));
	TIM2->SR&=~(0b1);
	TIM2->CR1&=~(0b1);
	TIM2->ARR=0xffffffff;
}

int read_roll();
void calibrate();


int main() {
// Configurations
    RCC->APB2ENR = (0b1<<0) | (0b11<<2) |(0b11<<0);
    RCC->APB1ENR = (0b11<<0) | (1<<21);

    GPIOA->CRL = 0x3A333333;
    GPIOB->CRL = 0xff444244;
    GPIOB->CRH = 0x33333333;


//PWM
	TIM3->ARR = 4000-1;
	TIM3->PSC = 355-1;
	TIM3->CCMR1 |= (0b110 << 4) | (1 << 3);                //PWM Mode
	TIM3->CCER  |= (1 << 0);                               //channels 1 enable
	TIM3->CR1   |= (1 << 7);                               //auto ARR PreLoad
	TIM3->CR1   |= (0b10 << 5);
	TIM3->EGR   |= (1 << 0);
	TIM3->CR1   |= (1 << 0);
//I2C
    I2C1->CR1|=0b1;
    I2C1->CR1&=~(1<<0);
    I2C1->CR1|=(1<<15);
    I2C1->CR1&=~(1<<15);
    I2C1->CR2=0x8;
    I2C1->CCR|=0b101000;
    I2C1->TRISE=9;
    I2C1->CR1|=0b1;

    while(I2C1->SR2&(1<<1));
    I2C1->CR1|=(1<<8);
    while(!(I2C1->SR1&(1<<0)));
    a=I2C1->SR1;
    for(int i=0;i<10;i++);
    I2C1->DR=0b11010000;
    while(!(I2C1->SR1&(1<<1)));
    a=I2C1->SR1;a=I2C1->SR2;
    while(!(I2C1->SR1&(1<<7)));
    I2C1->DR=0b01101011;
    while(!(I2C1->SR1&(1<<7)));
    I2C1->DR=0x00;
    while(!(I2C1->SR1&(1<<2)));
    I2C1->CR1|=(1<<9);

    delay(50);
    calibrate();

//Loop
    while (1) {
    	int roll = read_roll();
       	err= roll-0;                                         //TO BE CHANGED

//MOTOR CONTROL
       	GPIOB->ODR ^= (1<<9);                                // TEST LED

       	if(err>5){
    	    TIM3->CCR1 = (TIM3->ARR -(KP * err));
    	    GPIOA->ODR |= (1 << enable) | (1 << brake);                   // PA7=1, PA4=1 (Enable & Brake = OFF)

    	    GPIOA->ODR &= ~(1<< direction);                          // PA1=0 → Direction: CCW
    	    delay(500);
    	    GPIOA->ODR |= (1 << direction);                          // PA1=1 → Direction: CW
       	}
       	else if(err<-5){
    	    TIM3->CCR1 = (TIM3->ARR - (KP *-1*err));
    	    GPIOA->ODR |= (1 << enable) | (1 << brake);                   // PA7=1, PA4=1 (Enable & Brake = OFF)

    	    GPIOA->ODR |= (1 << direction);                          // PA1=1 → Direction: CW
    	    delay(500);
    	    GPIOA->ODR &= ~(1<< direction);                          // PA1=0 → Direction: CCW
       	 }

       	delay(500);
	    GPIOA->ODR &= ~(1 << brake);               // Brake till next reading
       	delay(1000);


    }
}

int read_roll(){
	I2C1->CR1&=~(1<<9);
	while(I2C1->SR2&(1<<1));
	I2C1->CR1|=(1<<8);
	while(!(I2C1->SR1&(1<<0)));
    a=I2C1->SR1;
	    for(int i=0;i<10;i++);
	    I2C1->DR=0b11010000;
	    while(!(I2C1->SR1&(1<<1)));
	    a=I2C1->SR1;a=I2C1->SR2;
    while(!(I2C1->SR1&(1<<7)));
   I2C1->DR=0x1C;
	    while(!(I2C1->SR1&(1<<7)));
	    I2C1->DR=0x10;
	    while(!(I2C1->SR1&(1<<2)));
	    I2C1->CR1|=(1<<9);

	I2C1->CR1&=~(1<<9);
	while(I2C1->SR2&(1<<1));
	I2C1->CR1|=(1<<8);
	while(!(I2C1->SR1&(1<<0)));
	a=I2C1->SR1;
	for(int i=0;i<10;i++);
	I2C1->DR=0b11010000;
	while(!(I2C1->SR1&(1<<1)));
	a=I2C1->SR1;a=I2C1->SR2;
	while(!(I2C1->SR1&(1<<7)));
    I2C1->DR=0x3B;
	    while(!(I2C1->SR1&(1<<2)));
	    I2C1->CR1|=(1<<9);
	I2C1->CR1&=~(1<<9);
	while(I2C1->SR2&(1<<1));
	I2C1->CR1|=(1<<8);
	I2C1->CR1|=(1<<10);
	while(!(I2C1->SR1&(1<<0)));
	a=I2C1->SR1;
	for(int i=0;i<10;i++);
	I2C1->DR=0b11010001;
	while(!(I2C1->SR1&(1<<1)));
	a=I2C1->SR1;a=I2C1->SR2;

   	while(!(I2C1->SR1&(1<<6)));
   	AccX=I2C1->DR;
   	AccX=AccX<<8;
   	while(!(I2C1->SR1&(1<<6)));
   	AccX|=I2C1->DR;
   	X=(float)AccX/4096.0;

   	while(!(I2C1->SR1&(1<<6)));
   	AccY=I2C1->DR;
   	AccY=AccY<<8;
   	while(!(I2C1->SR1&(1<<6)));
   	AccY|=I2C1->DR;
   	Y=(float)AccY/4096.0;

   	while(!(I2C1->SR1&(1<<6)));
   	AccZ=I2C1->DR;
   	AccZ=AccZ<<8;
   	I2C1->CR1|=(1<<9);
   	I2C1->CR1&=~(1<<10);
   	while(!(I2C1->SR1&(1<<6)));
   	AccZ|=I2C1->DR;
   	Z=(float)
   	AccZ/4096.0;
   	a=I2C1->DR;

//CALCULATIONS
   	roll = atan(Y / sqrt(X * X + Z * Z));
   	roll *= RAD_TO_DEG;
   	roll-=rollCal;
   	return (int)roll;
}

void calibrate(){
	int y=0;
	float pitchSum=0;
	float rollSum=0;
	while (y<40) {
		 I2C1->CR1&=~(1<<9);
	    	while(I2C1->SR2&(1<<1));
	    	I2C1->CR1|=(1<<8);
	    	while(!(I2C1->SR1&(1<<0)));
	        a=I2C1->SR1;
	  	    for(int i=0;i<10;i++);
	   	    I2C1->DR=0b11010000;
	   	    while(!(I2C1->SR1&(1<<1)));
	   	    a=I2C1->SR1;
	   	    a=I2C1->SR2;
	        while(!(I2C1->SR1&(1<<7)));
	       I2C1->DR=0x1C;
	   	    while(!(I2C1->SR1&(1<<7)));
	   	    I2C1->DR=0x10;
	   	    while(!(I2C1->SR1&(1<<2)));
	   	    I2C1->CR1|=(1<<9);

	    	I2C1->CR1&=~(1<<9);
	    	while(I2C1->SR2&(1<<1));
	    	I2C1->CR1|=(1<<8);
	    	while(!(I2C1->SR1&(1<<0)));
	    	a=I2C1->SR1;
	    	for(int i=0;i<10;i++);
	    	I2C1->DR=0b11010000;
	    	while(!(I2C1->SR1&(1<<1)));
	    	a=I2C1->SR1;
	    	a=I2C1->SR2;
	    	while(!(I2C1->SR1&(1<<7)));
	        I2C1->DR=0x3B;
	   	    while(!(I2C1->SR1&(1<<2)));
	   	    I2C1->CR1|=(1<<9);
	   	    //delay(50);
	    	I2C1->CR1&=~(1<<9);
	    	while(I2C1->SR2&(1<<1));
	    	I2C1->CR1|=(1<<8);
	    	I2C1->CR1|=(1<<10);
	    	while(!(I2C1->SR1&(1<<0)));
	    	a=I2C1->SR1;
	    	for(int i=0;i<10;i++);
	    	I2C1->DR=0b11010001;
	    	while(!(I2C1->SR1&(1<<1)));
	    	a=I2C1->SR1;
	       	a=I2C1->SR2;

	       	while(!(I2C1->SR1&(1<<6)));
	       	AccX=I2C1->DR;
	       	AccX=AccX<<8;
	       	while(!(I2C1->SR1&(1<<6)));
	       	AccX|=I2C1->DR;
	       	X=(float)AccX/4096.0;

	       	while(!(I2C1->SR1&(1<<6)));
	       	AccY=I2C1->DR;
	       	AccY=AccY<<8;
	       	while(!(I2C1->SR1&(1<<6)));
	       	AccY|=I2C1->DR;
	       	Y=(float)AccY/4096.0;

	       	while(!(I2C1->SR1&(1<<6)));
	       	AccZ=I2C1->DR;
	       	AccZ=AccZ<<8;
	       	I2C1->CR1|=(1<<9);
	       	I2C1->CR1&=~(1<<10);
	       	while(!(I2C1->SR1&(1<<6)));
	       	AccZ|=I2C1->DR;
	       	Z=(float)AccZ/4096.0;
	       	a=I2C1->DR;
	       	roll = atan(Y / sqrt(X * X + Z * Z));
	       	roll *= RAD_TO_DEG;
	       	rollSum+=roll;
	       	pitch = atan(-X / sqrt(Y * Y + Z * Z));
	       	pitch *= RAD_TO_DEG;
	       	pitchSum+=pitch;
	       	delay(50);
	       	y++;
	    }
		rollCal=(float)rollSum/40;
		pitchCal=(float)pitchSum/40;
}


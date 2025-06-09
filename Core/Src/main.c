 // Bonus
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include <math.h>
#define RAD_TO_DEG 57.2957795

float roll=0;
float roll2 = 0.0f;
float gyro_roll=0;
float pitch=0;
float gyro_pitch=0;

float rollCal=0;
float pitchCal=0;

float X=0;
float Y=0;
float Z=0;
//GYRO
float bias = 0.0f;
float P00 = 0.0f, P01 = 0.0f;
float P10 = 0.0f, P11 = 0.0f;
float Q_angle = 0.001f;
float Q_bias = 0.003f;
float R_measure = 0.03f;
float gX=0;
float gY=0;
float gZ=0;
float gX_fi=0;
float gY_fi=0;
float gZ_fi=0;

float dt = 0; //to be changed
int prev = 0;

int a=0;//TEMP

int16_t AccX=0;
int16_t AccY=0;
int16_t AccZ=0;

int16_t GyroX=0;
int16_t GyroY=0;
int16_t GyroZ=0;

uint16_t PWM_Cotrol=0;




void SetUp(){
	// Configurations
	    RCC->APB2ENR = (0b1<<0) | (0b11<<2) |(0b11<<0);
	    RCC->APB1ENR = (0b11<<0) | (1<<21);

	    GPIOA->CRL = 0x3A333333;
	    GPIOB->CRL = 0xff444244;
	    GPIOB->CRH = 0x33333333;
}

void delay (int x){
	TIM2->PSC=x-1;
	TIM2->ARR=8000-1;
	TIM2->CR1|=0b1;
	while(!(TIM2->SR&0b1));
	TIM2->SR&=~(0b1);
	TIM2->CR1&=~(0b1);
	TIM2->ARR=0xffffffff;
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

void PWM_Cnfig(){
	TIM3->ARR = 4000-1;
	TIM3->PSC = 355-1;
	TIM3->CCMR1 |= (0b110 << 4) | (1 << 3);                //PWM Mode
	TIM3->CCER  |= (1 << 0);                               //channels 1 enable
	TIM3->CR1   |= (1 << 7);                               //auto ARR PreLoad
	TIM3->CR1   |= (0b10 << 5);
	TIM3->EGR   |= (1 << 0);
	TIM3->CR1   |= (1 << 0);
}

void I2C_Config(){
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
}



int main() {
	SetUp();
    PWM_Cnfig();
    I2C_Config();
    delay(50);
    calibrate();

    while (1) {
// ACC
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
       	while(!(I2C1->SR1&(1<<6)));
       	AccZ|=I2C1->DR;
       	Z=(float)
       	AccZ/4096.0;
       	a=I2C1->DR;
//ACC Calculations
       	roll = atan(Y / sqrt(X * X + Z * Z));
       	roll *= RAD_TO_DEG;
       	roll-=rollCal;
//-----------------------------------------------------------------------------------------------------------------------------------
//TEMP Reads
       	while(!(I2C1->SR1&(1<<6)));
        a=I2C1->DR;
       	while(!(I2C1->SR1&(1<<6)));
       	a=I2C1->DR;
//GYRO
		while(!(I2C1->SR1&(1<<6)));
		GyroX=I2C1->DR;
		GyroX=GyroX<<8;
		while(!(I2C1->SR1&(1<<6)));
		GyroX|=I2C1->DR;
		gX=(float)GyroX/131;

		while(!(I2C1->SR1&(1<<6)));
		GyroY=I2C1->DR;
		GyroY=GyroY<<8;
		while(!(I2C1->SR1&(1<<6)));
		GyroY|=I2C1->DR;
		gY=(float)GyroY/131;

		while(!(I2C1->SR1&(1<<6)));
		GyroZ=I2C1->DR;
		GyroZ=GyroZ<<8;
		I2C1->CR1|=(1<<9);
		I2C1->CR1&=~(1<<10);
		while(!(I2C1->SR1&(1<<6)));
		GyroZ|=I2C1->DR;
		gZ=(float)GyroZ/131;
		a=I2C1->DR;
//Low pass filter
gX_fi = 0.6*gX + (1 - 0.6)*gX_fi;
gY_fi = 0.6*gY + (1 - 0.6)*gY_fi;
gZ_fi = 0.6*gZ + (1 - 0.6)*gZ_fi;


//dt


		float current = TIM3->CNT;

		if(current >prev){
			dt = current - prev;
		}else{
			dt = current + (((int)(TIM3->ARR) + 1) - prev);
		}
		prev = current;

		dt = dt * (TIM3->PSC + 1) / 72000000.0f;//in seconds

		//GYRO complementry filter

		gyro_roll += gX_fi*dt;
		gyro_pitch += gY_fi*dt;

		float roll_fi = 0.98*gyro_roll + 0.02*roll;
		float pitch_fi = 0.98*gyro_pitch + 0.02*pitch;





		//GYRO Calculations kalman filter

		float rate = gY - bias;
		roll2 += dt * rate;

		P00 += dt * (dt * P11 - P01 - P10 + Q_angle);
		P01 -= dt * P11;
		P10 -= dt * P11;
		P11 += Q_bias * dt;

		float S = P00 + R_measure;
		float K0 = P00 / S;
		float K1 = P10 / S;

		float y = roll - roll2;
		roll += K0 * y;
		bias  += K1 * y;
		roll2=(int)roll2;
		float P00_temp = P00;
		float P01_temp = P01;

		P00 -= K0 * P00_temp;
		P01 -= K0 * P01_temp;
		P10 -= K1 * P00_temp;
		P11 -= K1 * P01_temp;
//-----------------------------------------------------------------------------------------------------------------------------------
//MOTOR CONTROL
		if(roll2>40 | roll2<-40){
			GPIOA->ODR &= ~(1<< 7);
		}
       	if(roll2>5){
       		PWM_Cotrol = TIM3->ARR -(((20*roll2 + 6*gY)*1.8*4000)/17);
    	    TIM3->CCR1 = PWM_Cotrol;
    	    GPIOA->ODR |= (1 << 7) | (1 << 4);                   // PA7=1, PA4=1 (Enable & Brake = OFF)


    	    GPIOA->ODR |= (1 << 1);                          // PA1=1 → Direction: CW
//    	    delay(50);
//    	    GPIOA->ODR &= ~(1<< 1);                          // PA1=0 → Direction: CCW


       	}
       	else if(roll2<-5){
       		PWM_Cotrol = TIM3->ARR -(((20*roll2 + 6*gY)*-1.8*4000)/17);
    	    TIM3->CCR1 = PWM_Cotrol;                             // control between 100 - 500
    	    GPIOA->ODR |= (1 << 7) | (1 << 4);                   // PA7=1, PA4=1 (Enable & Brake = OFF)


    	    GPIOA->ODR &= ~(1<< 1);                          // PA1=0 → Direction: CCW
//    	    delay(50);
//    	    GPIOA->ODR |= (1 << 1);                          // PA1=1 → Direction: CW

       	 }
//      delay(100);
       	delay(20);
       	GPIOB->ODR ^= (1<<9);                                // TEST LED

    }
}







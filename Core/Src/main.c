#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include <math.h>
#define RAD_TO_DEG 57.2957795
#define GY_BUF_SIZE 4
#define roll_BUF_SIZE 40 //  to be tested


float roll=0;
float roll2 = 0.0f;
float pitch=0;
float rollCal=0;
float pitchCal=0;
float X=0;
float Y=0;
float Z=0;

float gyro_pitch=0;
float gyro_roll=0;
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
float prev = 0;
float gY_buffer[GY_BUF_SIZE] = {0};
float roll_buffer[roll_BUF_SIZE] = {0};

int counter=0;
int a=0;//TEMP
int gY_index = 0;
int roll_index = 0;


int16_t AccX=0;
int16_t AccY=0;
int16_t AccZ=0;

int16_t GyroX=0;
int16_t GyroY=0;
int16_t GyroZ=0;
int16_t filtered_gY =0;
int16_t filtered_roll=0;

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
	TIM3->PSC = 2-1;
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

float filter_gY(float input) {
	gY_buffer[gY_index] = input;
	gY_index = (gY_index + 1) % GY_BUF_SIZE;

	float sum = 0;
	for (int i = 0; i < GY_BUF_SIZE; i++)
		sum += gY_buffer[i];

	return sum / GY_BUF_SIZE;
	}

float filter_roll(float input) {
	roll_buffer[roll_index] = input;
	roll_index = (roll_index + 1) % roll_BUF_SIZE;

	float sum = 0;
	for (int i = 0; i < roll_BUF_SIZE; i++)
		sum += roll_buffer[i];
	return sum / roll_BUF_SIZE;
	}


int constrain(int value, int min_val, int max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
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
       	roll = (int) roll;
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
//Calculations
		filtered_gY = filter_gY(gY);
		filtered_roll = filter_roll(roll);

//Low pass filter
gX_fi = 0.6*gX + (1 - 0.6)*gX_fi;
gY_fi = 0.6*gY + (1 - 0.6)*gY_fi;
gZ_fi = 0.6*gZ + (1 - 0.6)*gZ_fi;

//dt
		float current = tim3->cnt;

		if(current >prev){
			dt = current - prev;
		}else{
			dt = current + (((int)(tim3->arr) + 1) - prev);
		}
		prev = current;

		dt = dt * (tim3->psc + 1) / 72000000.0f;//in seconds

		//gyro complementry filter

		gyro_roll += gx_fi*dt;
		gyro_pitch += gy_fi*dt;

		float roll_fi = 0.98*gyro_roll + 0.02*roll;
		float pitch_fi = 0.98*gyro_pitch + 0.02*pitch;




//MOTOR CONTROL
		if(fabs(roll)>45){
			GPIOA->ODR &= ~(1<< 7);                  			// Stop can't balance such Angle
		}
		else if(roll>5){
			PWM_Cotrol= constrain(TIM3->ARR -((17*roll + 2.5*filtered_gY)*3), 2500, 3800);
    	    TIM3->CCR1 = PWM_Cotrol;
    	    GPIOA->ODR |= (1 << 7) | (1 << 4);                   // PA7=1, PA4=1 (Enable & Brake = OFF)
    	    GPIOA->ODR |= (1 << 1);                              // PA1=1 → Direction: CW

       	}
       	else if(roll<-5){
       		PWM_Cotrol = constrain(TIM3->ARR - ((-17*roll + 2.5*filtered_gY)*3), 2500, 3800);
    	    TIM3->CCR1 = PWM_Cotrol;
    	    GPIOA->ODR |= (1 << 7) | (1 << 4);                   // PA7=1, PA4=1 (Enable & Brake = OFF)
    	    GPIOA->ODR &= ~(1<< 1);                              // PA1=0 → Direction: CCW
       	 }
       	else{
       		GPIOA->ODR &= ~(1<< 7); //Brake
       	}
		delay(50);


		if(counter>=15 | fabs(roll)>30 & fabs(roll)<45){
			if((filtered_roll-roll<3) & (filtered_roll-roll>-3) & ((roll<-5) | (roll>5))){
				if(fabs(roll)<15 & fabs(roll)>5){
					TIM3->CCR1 = constrain(TIM3->ARR -((17*fabs(roll) + 2.5*filtered_gY)*3), 3000, 3800);
				}
				else if(fabs(roll)>15 & fabs(roll)<30 ){
					TIM3->CCR1 = constrain(TIM3->ARR -((17*fabs(roll) + 2.5*filtered_gY)*4), 1800, 2800);
				}
				else if(fabs(roll)>30 & fabs(roll)<45){
					TIM3->CCR1 = constrain(TIM3->ARR -((17*fabs(roll) + 2.5*filtered_gY)*5), 1000, 1800);
				}
				delay(400);
				GPIOA->ODR ^= (1 << 1);                              // Toggle Direction
				delay(1300);
			}
			counter=0;
		}
		counter++;
       	GPIOB->ODR ^= (1<<9);                                // TEST LED

    }
}
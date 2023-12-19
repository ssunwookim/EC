# Project



## LAB: EC Design Problem

**Date:** 2023-12-19

**Author:** 22000090 김선우

**Github:** 

**Demo Video:** [[LAB] EC Design Problem](https://www.youtube.com/watch?v=kuYH3mSGvxc)



## I. Introduction

 In this final project, I endeavored to create a compact smart factory. Recognizing the prominence of smart factories in contemporary society, I believed that directly downsizing and developing a smart factory would contribute to a better understanding of the concept. The paramount focus in a smart factory lies in the precise execution of tasks through communication. The objective was to establish a robust network among devices and create a system where commands are issued and executed accurately. For the implementation of this system, I utilized stm32, the HAL library, and various sensors. This project aimed to underscore the significance of communication in smart factories, emphasizing the establishment of a seamless network for efficient task execution.

![image-20231219084245719](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219084245719.png)

### Requirement

#### Hardware

- **MCU**
  - NUCLEO-F411RE x3
- **Actuator**
  - DC Motor Driver (LS9110s)
  - DC Motor (SZH-GNP193) x2
  - RC Servo Motor (SG90) x7
  - DC Motor (TT Motor) 
- **Sensor**
  - IR Reflective Sensor (TCRT 5000) x2
  - Ultra Sonic Sensor (HC-SR04)
  - Piezoelectric sensor (FSR 406) x2
  - Bluetooth Module(HC-06)

#### Software

- Keil uVision
- EC_HAL library



## II. Problem

This study is a system that delivers chocolate to the desired location by entering the desired chocolate color.

1. Chocolate Factory
   - The serving system must be different depending on the three chocolate inputs.
   - The robotic arm should automatically move the chocolate.
   - It must be accurately delivered to the vehicle through a conveyor belt.
   - The starting and stopping of the vehicle must be controlled by a blocking door.

2. Delivery Car
   - It must follow the line and reach the delivery point exactly.
   - The status of receiving the logistics is inputted and the status is displayed with an LED.
   - If there is an obstacle, you must stop.
   - It must be possible to provide the status of delivery vehicles in real time.
3. Delivery address
   - Entry and exit of vehicles must be controlled by blocking doors.
   - It must be possible to confirm that the delivered goods have been properly delivered to the delivery destination.

#### Configuration

1. Chocolate Factory

![image-20231219121112366](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219121112366.png)

2. Delivery Car

![image-20231219121250241](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219121250241.png)

3. Delivery address

![image-20231219121309616](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219121309616.png)

#### Circuit Diagram

1. Chocolate Factory

![image-20231219122632486](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219122632486.png)

2. Delivery Car

![image-20231219131333511](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219131333511.png)

3. Delivery address

![image-20231219131815798](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219131815798.png)

## III. Algorithm



### Logic Design

![image-20231219102718172](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219102718172.png)



### Code

1. Factory System 1

```c
#include "stm32f411xe.h"
#include "math.h"
#include "ecSTM32F411.h"

//chocolate arm sys
#define PWM_turn	PA_0
//blokking door
#define PWM_block1 PA_1
//robot arm
#define PWM_PIN1 PA_6
#define PWM_PIN2 PC_7
#define PWM_PIN3 PC_8
#define PWM_PIN4 PC_9

static volatile uint8_t BT_Data = 1;
volatile static int state = 1;
volatile static int state_m = 1;
volatile static int duty_1 = 0;
volatile static int duty_2 = 0;
volatile static int duty_3 = 0;

volatile static int duty_turn = 0;
volatile static int duty_block1 = 0;
volatile static int flag_s = 0;
volatile static int count = 0;



void setup(void);
void Robot_arm(int state_m);

int main(void) {
	// Initialization --------------------------------------------------
	setup();	
	
	// Infinite Loop ---------------------------------------------------
	while(1){
		
		PWM_duty(PWM_turn, (float) (1 - (0.025 + (duty_turn * 0.005) )	) );
		
	  PWM_duty(PWM_block1, (float) (1 - (0.025 + (duty_block1 * 0.005) ) ) );
		
		// robot arm
		PWM_duty(PWM_PIN1, (float) (1 - (0.025 + (duty_1 * 0.005) )	) );
	  PWM_duty(PWM_PIN2, (float) (1 - (0.085 + (duty_2 * 0.005) ) ) );
		PWM_duty(PWM_PIN3, (float) (1 - (0.025 + (duty_2 * 0.005) ) ) );
		PWM_duty(PWM_PIN4, (float) (1 - (0.025 + (duty_3 * 0.005) ) ) );
		
		
		switch(flag_s){
			case 0:
				duty_turn = 0		;
				duty_block1 = 0	;
				duty_1 = 0			;
				duty_2 = 0			;
				duty_3 = 0			;
				break;
			
			case 1:

				switch(state){			
					
					case 1:
            duty_turn = 3;					
						state = 2;
						break;
					
					case 2:
						switch (state_m){
								
								case 1:
									duty_1 = 4;
									state_m = 2;
									break;
								case 2:
									if(duty_3 < 20) duty_3++;
									else state_m = 3;
									break;
								case 3:
									if(duty_2 < 5) duty_2++;
									else state_m = 4;
									break;
								case 4:
									duty_1 = 0;
									state_m = 5;
									break;
								case 5:
									if(duty_2 > 0) duty_2--;
									else state_m = 6;
									break;
								case 6:
									if(duty_3 > 0) duty_3--;
									else state_m = 7;
									break;
								case 7:
									duty_1 = 4;
									state_m = 8;
									break;
								case 8:
									duty_1 = 0;
									state_m = 1;
									state = 3;
									break;
							}
						break;
					case 3:
						duty_block1 = 0;
						count++;
						if(count >= 10 ){
							state = 4;
							count = 0;
						}
						break;
					case 4:
						duty_block1 = 10;
						count++;
						if(count >= 20){
							count = 0;
							state = 5;
						}							
						break;
					case 5:
						duty_block1 = 0;
						state = 1;	
						flag_s = 0;
						break;
				}
				break;
			case 2:
				
				switch(state){			
					
					case 1:
            duty_turn = 6;					
						state = 2;
						break;
					
					case 2:
						switch (state_m){
								
								case 1:
									duty_1 = 4;
									state_m = 2;
									break;
								case 2:
									if(duty_3 < 20) duty_3++;
									else state_m = 3;
									break;
								case 3:
									if(duty_2 < 5) duty_2++;
									else state_m = 4;
									break;
								case 4:
									duty_1 = 0;
									state_m = 5;
									break;
								case 5:
									if(duty_2 > 0) duty_2--;
									else state_m = 6;
									break;
								case 6:
									if(duty_3 > 0) duty_3--;
									else state_m = 7;
									break;
								case 7:
									duty_1 = 4;
									state_m = 8;
									break;
								case 8:
									duty_1 = 0;
									state_m = 1;
									state = 3;
									break;
							}
						break;
					case 3:
						duty_block1 = 0;
						count++;
						if(count >= 20 ){
							state = 4;
							count = 0;
						}
						break;
					case 4:
						duty_block1 = 10;
						count++;
						if(count >= 20){
							count = 0;
							state = 5;
						}							
						break;
					case 5:
						duty_block1 = 0;
						state = 1;	
						flag_s = 0;
						break;
				}
				break;
			case 3:
				
				switch(state){			
					
					case 1:
            duty_turn = 9;					
						state = 2;
						break;
					
					case 2:
						switch (state_m){
								
								case 1:
									duty_1 = 4;
									state_m = 2;
									break;
								case 2:
									if(duty_3 < 20) duty_3++;
									else state_m = 3;
									break;
								case 3:
									if(duty_2 < 5) duty_2++;
									else state_m = 4;
									break;
								case 4:
									duty_1 = 0;
									state_m = 5;
									break;
								case 5:
									if(duty_2 > 0) duty_2--;
									else state_m = 6;
									break;
								case 6:
									if(duty_3 > 0) duty_3--;
									else state_m = 7;
									break;
								case 7:
									duty_1 = 4;
									state_m = 8;
									break;
								case 8:
									duty_1 = 0;
									state_m = 1;
									state = 3;
									break;
							}
						break;
					case 3:
						duty_block1 = 0;
						count++;
						if(count >= 20 ){
							state = 4;
							count = 0;
						}
						break;
					case 4:
						duty_block1 = 10;
						count++;
						if(count >= 20){
							count = 0;
							state = 5;
						}							
						break;
					case 5:
						duty_block1 = 0;
						state = 1;	
						flag_s = 0;
						break;
				}
				break;				
			
		}
		
		delay_ms(500);
	}
}

// Initialiization QQ
void setup(void) {	
	//clock
	RCC_PLL_init();
	SysTick_init();
	
	UART2_init();
	UART2_baud(BAUD_9600);
	
	PWM_init(PWM_turn);
	PWM_period(PWM_turn, 20);   
	
	PWM_init(PWM_block1);
	PWM_period(PWM_block1, 20); 
	
	//servo motor 1
	PWM_init(PWM_PIN1);
	PWM_period(PWM_PIN1, 20);   
	
	PWM_init(PWM_PIN2);
	PWM_period(PWM_PIN2, 20); 
	
	PWM_init(PWM_PIN3);
	PWM_period(PWM_PIN3, 20); 
	
	PWM_init(PWM_PIN4);
	PWM_period(PWM_PIN4, 20); 
	
}

void USART1_IRQHandler(){         //USART1 INT 
	if(is_USART_RXNE(USART1)){
		BT_Data = USART_read(USART1);
		USART_write(USART1,(uint8_t*) "BT sent : ", 10);
		USART_write(USART1, &BT_Data, 1); 
		USART_write(USART1, "\r\n", 2);		
			
	}
}

void USART2_IRQHandler(){         //USART1 INT 
	if(is_USART_RXNE(USART2)){
		BT_Data = USART_read(USART2);
		USART_write(USART2,(uint8_t*) "BT sent : ", 10);
		USART_write(USART2, &BT_Data, 1); 
		USART_write(USART2, "\r\n", 2);
		
		switch (BT_Data){
		
		case 'R' :
			flag_s = 1;
			break;
		case 'B' :
			flag_s = 2;
			break;
		case 'G' :
			flag_s = 3;
			break;
		
		}
		
	}
}
```

- main
  - R
  - The dispenser rotates as many red chocolates are present.
  - Robotic arm moves according to state
  - The blocking door goes up and down
  - G
  - The dispenser rotates as many green chocolates are present.
  - Robotic arm moves according to state
  - The blocking door goes up and down
  - B
  - The dispenser rotates as many blue chocolates are present.
  - Robotic arm moves according to state
  - The blocking door goes up and down
- Usart
  - Receiving data from computer

2. Factory System 2

```c
#include "stm32f4xx.h"
#include "ecSTM32F411.h"
#include "string.h"
#include "math.h"

// Motor pin difine
#define M1 PA_0 
// IR sensor pin
PinName_t seqCHn[1] = {PB_0}

void set_up (void);
void Auto_mode (void)												;
void USART_print(void)											;

//IR parameter//
uint32_t value1;
int flag = 0;
int cnt = 0;

int flag_vel = 0;
int flag_delivery = 0;
int count = 0;

int flag_ss = 0;
int block = 0;

int main(void){
	
	set_up();
	
	while(1){
		
		PWM_duty(M1, (float) (1 - (0.025 + (block * 0.005) ) ) );
		
		printf("value1 = %d \r\n",value1);
		printf("block = %d \r\n",block);
	  printf("\r\n");
		
		count++;
		
		if(value1 > 800){
			flag_ss = 1;
		}
		
		if(flag_ss==1){
			block = 10;
			if(count >= 50){
				block = 0;
				flag_ss = 0;
				count = 0;
			}
		}

		delay_ms(100);
	}
}

void set_up (void){
	
	RCC_PLL_init();
	SysTick_init();
	
	UART2_init();
	UART2_baud(BAUD_9600);
	
	// ADC Init
	ADC_init(PB_0);
	// ADC channel sequence setting
	ADC_sequence(seqCHn, 1);
	
	// PWM M1 of 20 msec:  TIM2_CH2 (PA_0 AFmode)
	PWM_init(M1);	
	PWM_period(M1, 20);   // 1 msec PWM perio
	
}

void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();
	if(is_ADC_EOC()){		// after finishing sequence
		if (flag==0)			value1 = ADC_read();  
	}
}

```

- main
  - When the pressure sensor exceeds a certain value, the circuit breaker turns on and off.

3. Factory Car

```c
#include "stm32f4xx.h"
#include "ecSTM32F411.h"
#include "string.h"
#include "math.h"

// Motor pin difine
#define M1 PA_0 // left
#define M2 PA_1 // right

// Ultrasoinc pin difine
#define TRIG PA_6
#define ECHO PB_6

#define LED_PIN 5
#define BUTTON_PIN 13

// IR sensor pin
PinName_t seqCHn[3] = {PB_0, PB_1, PA_7};

void set_up (void);
void Auto_mode (void)												;
void USART_print(void)											;

int mode = 2		;
int dir = 0;
uint32_t ovf_cnt = 0				;
float distance = 0				  ;
float last_distance = 0			;
float timeInterval = 0			;
float time1 = 0							;
float time2 = 0							;

//IR parameter//
uint32_t value1, value2, value3; // val1 : left , val2 : right
int flag = 0;
int cnt = 0;

int flag_vel = 0;
int flag_delivery = 0;
int count = 0;
int cnt_2 = 0;

int main(void){
	
	set_up();
	
	while(1){
		
		Auto_mode();
		
		printf("value1 = %d \r\n",value1);
		printf("value2 = %d \r\n",value2);
		printf("value3 = %d \r\n",value3);
		printf("distance = %f \r\n",distance);
	  printf("\r\n");
		
		delay_ms(150);
		
	}
}

void set_up (void){
	
	RCC_PLL_init();
	SysTick_init();
	
	UART1_init();
	UART1_baud(BAUD_9600);
	
	UART2_init();
	UART2_baud(BAUD_9600);
	
	// ADC Init
	ADC_init(PB_0);
	ADC_init(PB_1);
	ADC_init(PA_7);
	// ADC channel sequence setting
	ADC_sequence(seqCHn, 3);
	
	// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG);							  	// PA_6: Ultrasonic trig pulse
	PWM_period_us(TRIG, 50000);     // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG, 10);    // PWM pulse width of 10us
	
	// Input Capture configuration -----------------------------------------------------------------------	
	ICAP_init(ECHO);    							// PB_6 as input caputre
 	ICAP_counter_us(ECHO, 10);   			// ICAP counter step time as 10us
	ICAP_setup(ECHO, IC_1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO, IC_2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
	
	// PWM M1 of 20 msec:  TIM2_CH2 (PA_0 AFmode)
	PWM_init(M1);	
	PWM_period(M1, 20);   // 1 msec PWM period
	// PWM M1 of 20 msec:  TIM2_CH2 (PA_0 AFmode)
	PWM_init(M2);	
	PWM_period(M2, 20);   // 1 msec PWM period
	// Direction pin 1
	GPIO_init(GPIOC, 2, OUTPUT);
	GPIO_otype(GPIOC, 2, EC_OUT_PU);
	GPIO_ospeed(GPIOC, 2, EC_MEDIUM);
	GPIO_pupd(GPIOC, 2, EC_PU);
	// Direction pin 2
	GPIO_init(GPIOC, 3, OUTPUT);
	GPIO_otype(GPIOC, 3, EC_OUT_PU);
	GPIO_ospeed(GPIOC, 3, EC_MEDIUM);
	GPIO_pupd(GPIOC, 3, EC_PU);
	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	
	GPIO_init(GPIOA, LED_PIN, OUTPUT);
	GPIO_pupd(GPIOC, LED_PIN, EC_PU);
	
	TIM_UI_init(TIM5, 1);
	
}

void Auto_mode (void){
	
				GPIO_write(GPIOC, 2, 1);
				GPIO_write(GPIOC, 3, 1);
				
				distance = (float) timeInterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
	
				if( (distance > 0 && distance <10) || (last_distance > 0 && last_distance <10) ){
						PWM_duty(M1, 0);
						PWM_duty(M2, 0);
						flag_vel = 0;
				}
				else{
					if(value1 > 1500 && value3 > 1500){
						PWM_duty(M1, 0);
						PWM_duty(M2, 0);
						flag_vel = 0;
					}	
					else if(value3 > 1500){
						PWM_duty(M1, 0.6);
						PWM_duty(M2, 0.0);
						flag_vel = 1;
					}
					else if(value1 > 1500){
						PWM_duty(M1, 0.0);
						PWM_duty(M2, 0.6);
						flag_vel = 1;

					}
					else {
						PWM_duty(M1, 0.6);
						PWM_duty(M2, 0.6);
						flag_vel = 1;
					}
				}
				last_distance = distance;
				
				if(value2 > 600){
					GPIO_write(GPIOA, LED_PIN, HIGH);
					flag_delivery = 1;
				}
				else if( GPIO_read(GPIOC, BUTTON_PIN)==0 ){
					GPIO_write(GPIOA, LED_PIN, LOW);
					flag_delivery = 0;
				}
				
				delay_ms(50);
}

void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();
	if(is_ADC_EOC()){		// after finishing sequence
		if (flag==0)
			value1 = ADC_read();  
		else if (flag==1)
			value2 = ADC_read();
		else if (flag==2)
			value3 = ADC_read();
		flag++;
		if(flag>2) flag = 0;
	}
}

void TIM4_IRQHandler(void){
	if(is_UIF(TIM4)){                     			// Update interrupt 
		ovf_cnt++			 ;													// overflow count
		clear_UIF(TIM4);  							   				// clear update interrupt flag
	}
	if(is_CCIF(TIM4, IC_1)){ 										// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1 = ICAP_capture(TIM4, IC_1);					// Capture TimeStart
		clear_CCIF(TIM4, 1);                			// clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM4, IC_2)){ 							// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2 = ICAP_capture(TIM4, IC_2);					// Capture TimeEnd
		timeInterval = ((time2 - time1) + ovf_cnt * (TIM2->ARR + 1)) / 100; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		ovf_cnt = 0;                        			// overflow reset
		clear_CCIF(TIM4,2);								  			// clear capture/compare interrupt flag 
	}
}

void USART_print(void){
	
		USART1_write((uint8_t*)"MOD: ", 5);
		USART1_write((uint8_t*)"Auto", 4);
		USART1_write((uint8_t*)" ", 1);
		USART1_write((uint8_t*)"CAR STATE: ", 11);
	
		switch (flag_delivery){
			case 0:
				switch(flag_vel){
					case 0:
						USART1_write((uint8_t*)"Waiting for item: ", 18);
						break;
					case 1:
						USART1_write((uint8_t*)"On the way back: ", 17);
						break;
				}					
				break;
				
			case 1:
				USART1_write((uint8_t*)"On delivery", 11);
				break;
		}
		
		USART1_write((uint8_t*)"\r\n", 2);

}

void TIM5_IRQHandler(void){
	if(is_UIF(TIM5)){			// Check UIF(update interrupt flag)
		cnt_2++;
		
		if(cnt_2 > 1000){
			USART_print();
			cnt_2=0;
		}
		
	}
	clear_UIF(TIM5); 		// Clear UI flag by writing 0
}


```

- main
  - while
  - Auto mode operating
- Auto
  - Control motor motion along the line
  - Stop when encountering an obstacle
  - LED on and off according to pressure sensor input
- Usart print
  - Car status output according to flag
- TIM5 IRQ
  - Car status updates every second

## IV. Results and Demo

![image-20231219133033058](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219133033058.png)

![image-20231219084245719](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219084245719.png)

![image-20231219133040955](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231219133040955.png)

- The process of providing goods from the factory and delivering them by car is sent in real time. A system that can repeat this process was configured as above.



**Demo vedio:** [[LAB] EC Design Problem](https://www.youtube.com/watch?v=kuYH3mSGvxc)



## V. Reference

github: [Past Projects - EC (gitbook.io)](https://ykkim.gitbook.io/ec/ec-course/project/past-projects)

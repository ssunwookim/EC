# LAB: Line Tracing RC Car

**Date:** 2023-11-26

**Author/Partner:** 김선우/양두원

**Github:** repository link

**Demo Video:** [EC_LAB: Line Tracing RC Car_김선우](https://www.youtube.com/watch?v=qC-VMJANpxM)



## Introduction

Design an embedded system to control an RC car to drive on the racing track. The car is controlled either manually with wireless communication or automatically to drive around the track. When it sees an obstacle on the driving path, it should temporarily stop until the obstacle is out of the path.

![image-20231126112743682](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126112743682.png)

### Requirement

#### Hardware

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others: Minimum
  - Bluetooth Module(HC-06)
  - DC motor x2, DC motor driver(L9110s)
  - IR Reflective Sensor (TCRT 5000) x2
  - HC-SR04
  - additional sensor/actuators are acceptable

#### Software

- Keil uVision, CMSIS, EC_HAL library



## Preparation

### Tutorials:

Complete the following tutorials:

1. Managing library header files

```c
#include "stm32f411xe.h"

#include "ecEXTI.h"
#include "ecPinNames.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecUART_simple.h"
#include "ecICAP.h"
#include "ecStepper.h"
#include "ecUART.h"
#include "ecADC.h"
```

1. Custom initialization

```c
void MCU_init(void)														;

void MCU_init(void){
	RCC_PLL_init();
	SysTick_init();
	
	UART2_init();
	UART2_baud(BAUD_9600);
	// USART1: BT serial init 
	UART1_init();
	UART1_baud(BAUD_9600);
	// PWM M1 of 20 msec:  TIM2_CH2 (PA_0 AFmode)
	PWM_init(M1);	
	PWM_period(M1, 3);   // 1 msec PWM period
	// PWM M1 of 20 msec:  TIM2_CH2 (PA_0 AFmode)
	PWM_init(M2);	
	PWM_period(M2, 3);   // 1 msec PWM period	
	// Direction pin 1
	GPIO_init(GPIOC, 2, OUTPUT);
	// Direction pin 2
	GPIO_init(GPIOC, 3, OUTPUT);
	// LED pin setting
	GPIO_init(GPIOA, 5, OUTPUT);
	// ADC Init
	ADC_init(PB_0);
	ADC_init(PB_1);
	// ADC channel sequence setting
	ADC_sequence(seqCHn, 2);
	// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG);							  	// PA_6: Ultrasonic trig pulse
	PWM_period_us(TRIG, 50000);     // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG, 10);    // PWM pulse width of 10us
	
	// Input Capture configuration -----------------------------------------------------------------------	
	ICAP_init(ECHO);    							// PB_6 as input caputre
 	ICAP_counter_us(ECHO, 10);   			// ICAP counter step time as 10us
	ICAP_setup(ECHO, IC_1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO, IC_2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
	
	TIM_UI_init(TIM5, 1);
}
```



### LABS:

You should review previous labs for help

1. LAB: ADC IR Sensor
2. LAB: USART Bluetooth
3. LAB: Timer & PWM



## Problem Definition

Design your RC car that has the following functions:

1. Line tracing on the given racing track
2. has 2 control modes: **Manual Mode** to **AUTO Mode**
3. stops temporally when it detects an object nearby on the driving path

On the PC, connected to MCU via bluetooth

- Print the car status every 1 sec such as “ ( “ MOD: A DIR: F STR: 00 VEL: 00 ”)



### Manual Mode

- Mode Change( MOD):
  - When 'M' or 'm' is pressed, it should enter **Manual Mode**
  - LD2 should be ON in Manual Mode
- Speed (VEL):
  - Increase or decrease speed each time you push the arrow key “UP” or “DOWN”, respectively.
  - You can choose the speed keys
  - Choose the speed level: V0 ~ V3
- Steer (STR):
  - Steering control with keyboard keys
  - Increase or decrease the steering angles each time you press the arrow key “RIGHT” or “LEFT”, respectively.
  - Steer angles with 3 levels for both sides: e.g: -3, -2, -1, 0, 1, 2, 3 // '-' angle is turning to left
- Driving Direction (DIR)
  - Driving direction is forward or backward by pressing the key “F” or “B”, respectively.
  - You can choose the control keys
- Emergency Stop
  - RC car must stop running when key “S” is pressed.



### Automatic Mode

- Mode Change:

  - When 'A' or 'a' is pressed, it should enter **AUTO Mode**

- LD2 should blink at 1 second rate in AUTO Mode

- It should drive on the racing track continuously

- Stops temporally when it detects an object nearby on the driving path

- If the obstacle is removed, it should drive continuouslyTutorials:

  Complete the following tutorials:



### LABS:

You should review previous labs for help

1. LAB: ADC IR Sensor
2. LAB: USART Bluetooth
3. LAB: Timer & PWM



## Procedure

1. Discuss with the teammate how to design an algorithm for this problem

![image-20231126113457282](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126113457282.png)

2. In the report, you need to explain concisely how your system works with state tables/diagram or flow-chart.

![image-20231126113610236](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126113610236.png)

3. Select appropriate configurations for the design problem. Fill in the table.

| **Functions**             | **Register**          | **PORT_PIN**                                              | **Configuration**                                            |
| ------------------------- | --------------------- | --------------------------------------------------------- | ------------------------------------------------------------ |
| System Clock              | RCC                   |                                                           | PLL 84MHz                                                    |
| delay_ms                  | SysTick               |                                                           |                                                              |
| Motor DIR                 | Digital Out           | PC_2, PC_3                                                | output                                                       |
| DC Motor PWM pin          | PWM2(TIM2_CH1 or CH2) | PA_0, PA_1                                                | AF, Push-Pull, Pull-Up, Fast                                 |
| LED pin                   | Digital Out           | PA_5                                                      | output                                                       |
| Ultrasonic PWM            | TIM3_CH1              | PA6                                                       | AF, Push-Pull, No Pull-up Pull-down, / FastPWM period: 50msec pulse width: 10usec |
| Ultrasonic Input capture  | TIM4_CH1              | PB6                                                       | AF, No Pull-up Pull-down / Counter Clock : 0.1MHz (10us) TI4 -> IC1 (rising edge) TI4 -> IC2 (falling edge |
| ADC                       | ADC                   | PB_0, PB_1                                                | Analog Mode No Pull-up Pull-down                             |
| ADC sampling trigger      | PWM3                  | PB_0: ADC1_CH8 (1st channel) PB_1: ADC1_CH9 (2nd channel) | ADC Clock Prescaler /8 12-bit resolution, right alignment Continuous Conversion mode Scan mode: Two channels in regular group External Trigger (Timer3 Trigger) @ 1kHz Trigger Detection on Rising Edge |
| RS-232 USB cable(ST-LINK) | USART2                |                                                           | No Parity, 8-bit Data, 1-bit Stop bit 38400 baud-rate        |
| Bluetooth                 | USART1                | TXD: PA9 RXD: PA10                                        | No Parity, 8-bit Data, 1-bit Stop bit 9600 baud-rate         |



### Circuit Diagram

![image-20231126003616781](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126003616781.png)

### Code

1. main code

```c
int main(void){
	
	MCU_init ();
	
	USART1_write((uint8_t *)"Hello\r\n",7);
	
	while(1){
		
		switch(mode){
			case 1:
				manual_mode();
				manipulate();
				PWM_duty(M1, left_duty );
				PWM_duty(M2, right_duty);
				break;
			case 2:
				Auto_mode();
				break;
		}
		
	}
}
```

- 핀 설정이나 인터럽트 설정과 같은 초기 세팅
- while()
  - mode 1: manual mode 실행
  - mode 2: Auto mode 실행

2. MCU init

```c
void MCU_init(void){
	RCC_PLL_init();
	SysTick_init();
	
	UART2_init();
	UART2_baud(BAUD_9600);
	// USART1: BT serial init 
	UART1_init();
	UART1_baud(BAUD_9600);
	// PWM M1 of 20 msec:  TIM2_CH2 (PA_0 AFmode)
	PWM_init(M1);	
	PWM_period(M1, 3);   // 1 msec PWM period
	// PWM M1 of 20 msec:  TIM2_CH2 (PA_0 AFmode)
	PWM_init(M2);	
	PWM_period(M2, 3);   // 1 msec PWM period	
	// Direction pin 1
	GPIO_init(GPIOC, 2, OUTPUT);
	// Direction pin 2
	GPIO_init(GPIOC, 3, OUTPUT);
	// LED pin setting
	GPIO_init(GPIOA, 5, OUTPUT);
	// ADC Init
	ADC_init(PB_0);
	ADC_init(PB_1);
	// ADC channel sequence setting
	ADC_sequence(seqCHn, 2);
	// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG);							  	// PA_6: Ultrasonic trig pulse
	PWM_period_us(TRIG, 50000);     // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG, 10);    // PWM pulse width of 10us
	
	// Input Capture configuration -----------------------------------------------------------------------	
	ICAP_init(ECHO);    							// PB_6 as input caputre
 	ICAP_counter_us(ECHO, 10);   			// ICAP counter step time as 10us
	ICAP_setup(ECHO, IC_1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO, IC_2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
	
	TIM_UI_init(TIM5, 1);
}
```

3. USART1_IRQHandler

```c
void USART1_IRQHandler(){         //USART1 INT 
	if(is_USART_RXNE(USART1)){
		BT_Data = USART_read(USART1);
		USART_write(USART1,(uint8_t*) "BT sent : ", 10);
		USART_write(USART1, &BT_Data, 1); 
		USART_write(USART1, "\r\n", 2);		
		
		// switch mode
		switch(BT_Data){
			case 'm':
			case 'M':
				mode = 1;
				break;
			case 'q':
			case 'Q':
				mode = 2;
				break;
		}
		// switch vel,str
		switch(BT_Data){
			case 'h':
			case 'H':
				if(vel<3) vel++;
				break;
			case 'n':
			case 'N':
				if(vel>0) vel--;
				break;
			case 'a':
			case 'A':
				str2 = 0;
				if(str1<3) str1++;
				break;
			case 'd':
			case 'D':
				str1 = 0;
				if(str2<3) str2++;
				break;
		}
	}
}
```

- Read & Write bt data
- switch data
  - BT data : M -> manual mode (mode = 1)
  - BT data : Q -> Auto mode (mode = 2)
- switch vel ,str
  - BT data : H -> speed up (0 -> 3)
  - BT data : N -> speed down(3 -> 0)
  - BT data : A -> left steer ++ (0 -> -3)
  - BT data : D -> right steer ++(0 -> 3)

4. manual mode

```c
void manual_mode(void){
	
	switch(BT_Data){
        case 'M':
        case 'm':
        case 'S':
        case 's':
            drive_state = 0	;
                    vel = -1;
            break;
        //GO STRAIGHT
        case 'W':
        case 'w':
            drive_state = 1;
            dir = 1;
            vel = 0;
            str1 = 0;
            str2 = 0;
            GPIO_write(GPIOC, 2, dir);
            GPIO_write(GPIOC, 3, dir);
            break;

        //GO BACKWARD
        case 'B':
        case 'b':
            drive_state = 1	;
            dir = 0;
            vel = 0;
            str1 = 0;
            str2 = 0;
            GPIO_write(GPIOC, 2, dir);
            GPIO_write(GPIOC, 3, dir);
            break;
        //Go LEFT
        case 'a':
        case 'A':
            drive_state = 2;
            break;
		// GO RIGHT
        case 'd':
        case 'D':
            drive_state = 2;
            break;
      }
 }

```

- Stop
  - BT data : M -> initial pwm stop
  - BT data : S -> stop mode(drive state = 0)
- Straight
  - BT data : W / dir = 1/ dirve state = 1
- back
  - BT data : B / dir = 0/ dirve state = 1
- Letf
  - BT data : A / dirve state = 2
- Right
  - BT data :D / dirve state = 2

5. manipulate

```c
void manipulate(void) {
	
		DIR = 1 - dir ;
	
    switch (drive_state) {
			case 0:  //STOP
					left_duty  = fabs(DIR - 0);
					right_duty = fabs(DIR - 0);
					break;
			case 1:  
					DIR = 1 - dir ;
					left_duty  = 	fabs(DIR - (0.4 + (0.1 * vel))) ;
					right_duty =  fabs(DIR - (0.4 + (0.1 * vel)))	;
					break;
			case 2:  
					left_duty  = fabs(DIR - (0.6 + (str2 * 0.1)));
					right_duty = fabs(DIR - (0.6 + (str1 * 0.1)));
					break;
    }
}
```

- drive state 0: stop
- dirve state 1
  - straight : dir 1
  - back : dir 0
  - speed controll by vel
- dirve state 2
  - Left: str1++
  - Right: str2++

6. Auto mode

```c
void Auto_mode (void){
				
				dir = 1;
				GPIO_write(GPIOC, 2, 1);
				GPIO_write(GPIOC, 3, 1);
				
				distance = (float) timeInterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
				
				if( (distance > 0 && distance <10) || (last_distance > 0 && last_distance <10) ){
						PWM_duty(M1, 0);
						PWM_duty(M2, 0);
						vel_Auto = -1;
						str_Auto = 0;
				}
				else{
					if(value1 > 1500 && value2 > 1500){
						PWM_duty(M1, 0);
						PWM_duty(M2, 0);
						vel_Auto = -1;
						str_Auto = 0;
					}	
					else if(value2 > 1500){
						PWM_duty(M1, 0.9);
						PWM_duty(M2, 0.4);
						vel_Auto = 0;
						str_Auto = 1;
					}
					else if(value1 > 1500){
						PWM_duty(M1, 0.4);
						PWM_duty(M2, 0.9);
						vel_Auto = 0;
						str_Auto = -1;
					}
					else {
						PWM_duty(M1, 1);
						PWM_duty(M2, 1);
						vel_Auto = 0;
						str_Auto = 0;
					}
				}
				last_distance = distance;
			
				delay_ms(50);
}
```

- distance < 10 : stop
- val1 & val2 > 1500 : stop
- val 1 > 1500 : Go left
- val2 > 1500 : Go right
- val1 & val2 < 1500 : straight

7. Usart print

```c
void USART_print(void) {


    if (mode == 1 || mode == 2)
        USART1_write((uint8_t*)"MOD: ", 5);

    switch (mode) {
    case 1:
        USART1_write((uint8_t*)"M", 1);
        break;
    case 2:
        USART1_write((uint8_t*)"A", 1);
        break;
    }

    if (mode == 1) {
        USART1_write((uint8_t*)" DIR: ", 5);
        switch (dir) {
        case 0:
            USART1_write((uint8_t*)"0", 1);
            break;
        case 1:
            USART1_write((uint8_t*)"1", 1);
            break;
        }
				
				str = str2 - str1;
				
        USART1_write((uint8_t*)" STR: ", 5);
				
        switch (str) {
        case -3:
            USART1_write((uint8_t*)"-3", 2);
            break;
        case -2:
            USART1_write((uint8_t*)"-2", 2);
            break;
        case -1:
            USART1_write((uint8_t*)"-1", 2);
            break;
        case 0:
            USART1_write((uint8_t*)"0", 2);
            break;
        case 1:
            USART1_write((uint8_t*)"1", 2);
            break;
        case 2:
            USART1_write((uint8_t*)"2", 2);
            break;
        case 3:
            USART1_write((uint8_t*)"3", 2);
            break;
        }


        USART1_write((uint8_t*)" VEL: ", 5);

        switch (vel) {
        case -1:
            USART1_write((uint8_t*)"STOP", 4);
            break;
        case 0:
            USART1_write((uint8_t*)"V0", 2);
            break;
        case 1:
            USART1_write((uint8_t*)"V1", 2);
            break;
        case 2:
            USART1_write((uint8_t*)"V2", 2);
            break;
        case 3:
            USART1_write((uint8_t*)"V3", 2);
            break;
        }

        USART1_write((uint8_t*)"\r\n", 2);
    }


    else if (mode == 2) {
        USART1_write((uint8_t*)" DIR: ", 5);
        USART1_write((uint8_t*)"1", 1);
        USART1_write((uint8_t*)" STR: ", 5);
        switch (str_Auto) {
					case -1:
            USART1_write((uint8_t*)"-1", 2);
            break;
					case 0:
            USART1_write((uint8_t*)"0", 2);
            break;
					case 1:
            USART1_write((uint8_t*)"1", 2);
            break;
        }
        USART1_write((uint8_t*)" VEL: ", 5);
        switch (vel_Auto) {
					case -1:
            USART1_write((uint8_t*)"STOP", 4);
            break;
					case 0:
            USART1_write((uint8_t*)"V0", 2);
            break;
        }
        USART1_write((uint8_t*)"\r\n", 2);

    }
	}
```

- mode 1 -> MOD: M DIR: 0 or 1 STR: (-3 ~ 3) VEL: (Stop ~ V3)
- mode 1 -> MOD: A DIR: 0 STR: (-1 ~ 1) VEL: (Stop ~ V0)

8. IR sensor

```c
void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();
	if(is_ADC_EOC()){		// after finishing sequence
		if (flag==0)
			value1 = ADC_read();  
		else if (flag==1)
			value2 = ADC_read();
		flag =! flag;		// flag toggle
	}
}
```

9. Ultra sonic sensor

```c
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
```

10. TIM5_IRQHandler

```c
void TIM5_IRQHandler(void){
	if(is_UIF(TIM5)){			// Check UIF(update interrupt flag)
		cnt_1++;
		
		if(cnt_1 > 1000){
			USART_print();
			if(mode == 1){
				GPIO_write(GPIOA, 5, 1);
			}
			else if(mode == 2){
				LED_toggle();
			}
			cnt_1=0;
		}
		
	}
	clear_UIF(TIM5); 		// Clear UI flag by writing 0
}
```

- every 1 second
  - USART print
  - mode 1: LED on
  - mode 2: LED toggle

## Results

1. manual mode

![image-20231126004950491](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126004950491.png)

| stop     | ![image-20231126005102118](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126005102118.png) |
| -------- | ------------------------------------------------------------ |
| straight | ![image-20231126005159765](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126005159765.png) |
| back     | ![image-20231126005333686](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126005333686.png) |
| left     | ![image-20231126005716779](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126005716779.png) |
| right    | ![image-20231126005849334](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126005849334.png) |

2. Auto mode

![image-20231126010232700](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126010232700.png)

| stop     | ![image-20231126010512575](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126010512575.png) |
| -------- | ------------------------------------------------------------ |
| straight | ![image-20231126010813682](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126010813682.png) |
| left     | ![image-20231126010913872](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126010913872.png) |
| right    | ![image-20231126010844135](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126010844135.png) |

Demo vedio link: [EC_LAB: Line Tracing RC Car_김선우](https://www.youtube.com/watch?v=qC-VMJANpxM)

## Reference

1. Gitbook: [LAB: Line Tracing RC Car - EC (gitbook.io)](https://ykkim.gitbook.io/ec/ec-course/lab/lab-line-tracing-rc-car)
2. TU_SYStic&EXTI

![image-20231126121721265](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231126121721265.png)



## Troubleshooting



### 1. motor PWM duty ratio for different DIR

When, DIR=0 duty=0.8--> PWM 0.8 // 실제 모터에 전달되는 pwm

Whe, DIR=1 duty=0.8--> PWM 0.2 // 실제 모터에 전달되는 PWM

***solution***

```c
left_duty  = 	fabs(DIR - (0.4 + (0.1 * vel))) ;
right_duty =  fabs(DIR - (0.4 + (0.1 * vel)))	;

// change the pwm by dir
```



### 2. Print a string for BT (USART1)

- example

```c
#define _CRT_SECURE_NO_WARNINGS    // sprintf 보안 경고로 인한 컴파일 에러 방지
#include <stdio.h>     // sprintf 함수가 선언된 헤더 파일

char BT_string[20]=0;

int main()
{
	sprintf(BT_string, "DIR:%d PWM: %0.2f\n", dir, duty);    // 문자, 정수, 실수를 문자열로 만듦
	USART1_write(BT_string, 20);
	// ...
}
```

- my code

```c
   .
   .
    if (mode == 1 || mode == 2)
        USART1_write((uint8_t*)"MOD: ", 5);

    switch (mode) {
    case 1:
        USART1_write((uint8_t*)"M", 1);
        break;
    case 2:
        USART1_write((uint8_t*)"A", 1);
        break;
	.
    .
```

### 3. Motor does not run under duty 0.5

- First we check the dead zone of motor
-  Configure motor PWM period as 1kHa
- Use motor with limit pwm set to 0.5

### 4. Check and give different Interrupt Priority

In a microcontroller system, NVIC is responsible for managing interrupt priorities. Priority levels usually range from 0 to 15, where 0 is the highest priority, and 15 is the lowest priority. When handling multiple interrupts, it's crucial to assign priorities appropriately to ensure that critical tasks get processed first.

1. **Original Settings:**
   - TIMER priority: 2
   - ADC priority: 2
   - USART priority: 1
2. **Adjustments:**
   - TIMER priority adjusted to 3 for higher priority.
   - USART priority adjusted to 10 for a lower priority.
3. **Resulting Priorities:**
   - TIMER priority: 3
   - ADC priority: 2 (unchanged)
   - USART priority: 10

### 5. Ultrasoninc sensor does not measure properly when MCU is connected with motor driver

SOL) Give independent voltage source to motor driver. Giving DC power from MCU to motor driver is not recommended

I wrote code to set up a workspace to eliminate factors affecting behavior where signals are too small or values are too large, and allow the values of past signals to be added to construct the current state.

```c
distance = (float) timeInterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
				
				if( (distance > 0 && distance <10) || (last_distance > 0 && last_distance <10) ){
						PWM_duty(M1, 0);
						PWM_duty(M2, 0);
						vel_Auto = -1;
						str_Auto = 0;
				}
.
.
.    
    			last_distance = distance;
			
				delay_ms(50);
}
```

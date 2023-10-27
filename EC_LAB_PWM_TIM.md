# LAB: Timer & PWM



## LAB: **PWM – Servo motor and DC motor**

**Date:** 2023-10-26

**Author:** 22000090/김선우

**Github:** 

**Demo Video:** [EC_LAB: Timer & PWM_김선우](https://www.youtube.com/watch?v=GSufnpuP0tk&t=8s)

### I. Introduction

 Write a program that drives the timer and PWM, and design the operation of outputting the sevo motor and DC motor according to the PWM input waveform.

#### Requirement

**Hardware**

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 3 LEDs and load resistance
  - RC Servo Motor (SG90)
  - DC motor (5V)
  - DC motor driver(LS9110s)
  - breadboard

**Software**

- Keil uVision, CMSIS, EC_HAL library



### II. Problem 1: RC servo motor

 An RC servo motor is a tiny and light weight motor with high output power. It is used to control rotation angles, approximately 180 degrees (90 degrees in each direction) and commonly applied in RC car, and Small-scaled robots. The angle of the motor can be controlled by the pulse width (duty ratio) of PWM signal. The PWM period should be set at **20ms or 50Hz**. Refer to the datasheet of the RC servo motor for detailed specifications.

![img](https://user-images.githubusercontent.com/38373000/195773601-f0f19e35-0a6f-49af-aa87-574c86bfec62.png)

#### 1-1. Create HAL library

1. Create the library files as

   - ecTIM.h, ecTIM.c

   - ecPWM.h, ecPWM.c

2. Update header files located in the directory `EC \lib\`.

##### ecTIM.h

```c
/* Timer Configuration */
void TIM_init(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_period(TIM_TypeDef* TIMx, uint32_t msec);

void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_UI_enable(TIM_TypeDef* TIMx);
void TIM_UI_disable(TIM_TypeDef* TIMx);

uint32_t is_UIF(TIM_TypeDef* TIMx);
void clear_UIF(TIM_TypeDef* TIMx);
```

##### ecPWM.h

```c
/* PWM Configuration using PinName_t Structure */

/* PWM initialization */
// Default: 84MHz PLL, 1MHz CK_CNT, 50% duty ratio, 1msec period
void PWM_init(PinName_t pinName);
void PWM_pinmap(PinName_t pinName, TIM_TypeDef** TIMx, int* chN);


/* PWM PERIOD SETUP */
// allowable range for msec:  1~2,000
void PWM_period(PinName_t pinName, uint32_t msec);
void PWM_period_ms(PinName_t pinName, uint32_t msec);	// same as PWM_period()
// allowable range for usec:  1~1,000
void PWM_period_us(PinName_t pinName, uint32_t usec);


/* DUTY RATIO SETUP */
// High Pulse width in msec
void PWM_pulsewidth(PinName_t pinName, uint32_t pulse_width_ms);
void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms);  // same as void PWM_pulsewidth
void PWM_pulsewidth_us(PinName_t pinName, uint32_t pulse_width_us);
// Duty ratio 0~1.0
void PWM_duty(PinName_t pinName, float duty);
```

#### 2. Procedure

 Make a simple program that changes the angle of the RC servo motor that rotates back and forth from 0 deg to 180 degree within a given period of time.

Reset to '0' degree by pressing the push button (PC13).

- Button input has to be an External Interrupt
- Use Port A Pin 1 as PWM output pin for TIM2_CH2.
- Use Timer interrupt of period 500msec.
- Angle of RC servo motor should rotate from 0° to 180° and back 0° at a step of 10° at the rate of 500msec.

- Check duty of PWM waveform

  - Duty = 0

  ![image-20231027180254579](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231027180254579.png)

  - Duty = 0.5

  ![image-20231027175944621](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231027175944621.png)

  - Duty = 1

  ![image-20231027180908890](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231027180908890.png)

1. Create a new project under the directory `\repos\EC\LAB\LAB_PWM`

- The project name is “**LAB_PWM”.**
- Create a new source file named as “**LAB_PWM_RCmotor.c”**

2. Include updated library in `\repos\EC\lib\` to project.

- **ecPinNames.h** **ecPinNames.c**
- **ecGPIO.h, ecGPIO.c**
- **ecRCC.h, ecRCC.c**
- **ecEXTI.h, ecEXTI.c**
- **ecTIM.h**, **ecTIM.c**
- **ecPWM.h** **ecPWM.h**

1. Connect the RC servo motor to MCU pin (PA1) , VCC and GND
2. Increase the angle of RC servo motor from 0° to 180° with a step of 10° every 500msec. After reaching 180°, decrease the angle back to 0°. Use timer interrupt IRQ.
3. When the button is pressed, it should reset to the angle 0° and start over. Use EXT interrupt.

#### 3. Configuration

| Type                | Port - Pin        | Configuration                                      |
| ------------------- | ----------------- | -------------------------------------------------- |
| **Button**          | Digital In (PC13) | Pull-Up                                            |
| **PWM Pin**         | AF (PA1)          | Push-Pull, Pull-Up, Fast                           |
| **PWM Timer**       | TIM2_CH2 (PA1)    | TIM2 (PWM) period: 20msec, Duty ratio: 0.5~2.5msec |
| **Timer Interrupt** | TIM3              | TIM3 Period: 1msec, Timer Interrupt of 500 msec    |

#### 4. Circuit Diagram

![image-20231027001303441](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231027001303441.png)

#### 5. Discussion

1. Derive a simple logic to calculate CRR and ARR values to generate x[Hz] and y[%] duty ratio of PWM. How can you read the values of input clock frequency and PSC?

> 1. PWM frequency
>
> - Calculate the counter clock frequency (F_ck_cnt) by dividing the input clock frequency (F_ck_psc) by (PSC + 1).
>
> - Then, calculate the desired PWM frequency (F_pwm) by dividing F_ck_cnt by (ARR + 1).
>
>   - Fck_cnt = Fck_psc / (PSC+1)
>
>   - Fpwm = Fck_cnt / (ARR + 1)
>
> 2. PWM duty ratio
>
> - CCR (Counter-Reload Register) is calculated using the ARR (Auto-Reload Register) value plus 1 and the desired duty ratio (DutyCycle).
>
>   - DutyCycle = CCR / (ARR +1)
>
>   - CCR = (ARR + 1) * dutycycle

2. What is the smallest and highest PWM frequency that can be generated for Q1?

> 1. Maximum F_pwm Calculation
>    - Therefore, F_ck_cnt = 84 MHz / 1 = 84 MHz 
>    - And, F_pwm = 84 MHz / 1 = 84 MHz
>
> 2. Minimum F_pwm Calculation
>    - Generally, if the number of bits in the PSC register is 16 bits, the PSC value can be set from 0 to 65535. ARR values have similar limitations.
>    - F_ck_cnt = F_ck_psc / (PSC + 1) = 84 MHz / (65535 + 1) ≈ 1281.74 Hz
>    - F_pwm = F_ck_cnt / (ARR + 1) = 1281.74 Hz / (65535 + 1) ≈ 0.0195 Hz
>    - However, it is possible in theory and it must be experimentally determined whether it can be implemented in practice.

#### 6. Code

1. main code

```c
#include "stm32f411xe.h"
#include "math.h"
#include "ecSTM32F411.h"

// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
#define PWM_PIN PA_1

uint32_t count = 0;
volatile static int i = 0;

void setup(void);
void EXTI15_10_IRQHandler(void);
void TIM3_IRQHandler(void);

int main(void) {
	// Initialization --------------------------------------------------
	setup();	
	
	// Infinite Loop ---------------------------------------------------
	while(1){
	}		
}

// Initialiization 
void setup(void) {	
	//clock
	RCC_PLL_init();
	SysTick_init();
	// Initialize Input Button
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	// Priority Highest(0) External Interrupt
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	// PWM of 20 msec:  TIM2_CH2 (PA_1 AFmode)
	PWM_init(PWM_PIN);	
	PWM_period(PWM_PIN, 20);   // 20 msec PWM period
	// TIM3 period 1 msec, interrupt of 500mec
	TIM_UI_init(TIM3, 1);
	TIM_UI_enable(TIM3);
}

void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		i = 1;
		PWM_duty(PWM_PIN, (float) ((0.5 * i) / 20));
		clear_pending_EXTI(BUTTON_PIN); 
	}
}

void TIM3_IRQHandler(void){
	if(is_UIF(TIM3)){ // update interrupt flag
		//Create the code to rotate angle of RC by 500ms
		if(count%500 == 0) { // 0.5sec : 500
			PWM_duty(PWM_PIN, (float) ((0.5 + (0.11 * i)) / 20) );
			i++;
			if(i>18) i = 0;
		}			
		count++;
		clear_UIF(TIM3);    // clear by writing 0
	}
}
```

1. setup
   - clock
   - Initialize Input Button
   - Priority Highest(0) External Interrupt
   - PWM of 20 msec:  TIM2_CH2 (PA_1 AFmode)
   - TIM3 period 1 msec, interrupt of 500mec
2. void EXTI15_10_IRQHandler(void) 
   - An interrupt occurs when a button is pressed
   - set i = 0, set to 0 degrees
3. void TIM3_IRQHandler(void)
   - Set the 500th timer interrupt to occur through a count for the timer interrupt that occurs at 1 msec.
   - It is 0.5ms based on a 20ms cycle at 0 degrees of the motor. And at 180 degrees, it is 2.5ms. If you move 10 degrees per 500msec, divide 2ms by 18 to increase by 0.11 each turn.

#### 7. Results

Results

| degree 0                                                     | degree 90                                                    | degree 180                                                   |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image-20231027121918239](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231027121918239.png) | ![image-20231027122032068](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231027122032068.png) | ![image-20231027122049646](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231027122049646.png) |

Demo vedio:  [EC_LAB: Timer & PWM_김선우](https://www.youtube.com/watch?v=GSufnpuP0tk&t=8s)

### Problem 2: DC motor

#### 1. Procedure

-  Make a simple program that rotates a DC motor that changes the duty ratio from 25% -->75%--> 25% --> and so on.

-  The rotating speed level changes every 2 seconds.

-  By pressing the push button (PC13), toggle from Running and stopping the DC motor

1. Use the same project.

- Create a new source file named “**LAB_PWM_DCmotor.c”**
- You need to eliminate the other source file that contains `main()` from the project
  - e.g. Eliminate "“**LAB_PWM_RCmotor.c”** from the project

2. Connect DC motor and DC motor driver.

- PA_0 for the DC motor PWM
- PC_2 for Direction Pin

3. Change DC motor from LOW Speed to HIGH Speed for every 2 seconds

- e.g. 25% -->75%--> 25% --> and so on.

4. When Button is pressed, it should PAUSE or CONTINUE motor run



#### 2. Configuration

| Function            | Port - Pin        | Configuration                                   |
| ------------------- | ----------------- | ----------------------------------------------- |
| **Button**          | Digital In (PC13) | Pull-Up                                         |
| **Direction Pin**   | Digital Out (PC2) | Push-Pull                                       |
| **PWM Pin**         | AF (PA0)          | Push-Pull, Pull-Up, Fast                        |
| **PWM Timer**       | TIM2_CH1 (PA0)    | TIM2 (PWM) period: **1msec (1kHz)**             |
| **Timer Interrupt** | TIM3              | TIM3 Period: 1msec, Timer Interrupt of 500 msec |



#### 3. Circuit Diagram![image-20231026203343480](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231026203343480.png)



#### 4. Code

1. main code

```c
#include "stm32f411xe.h"
#include "math.h"
#include "ecSTM32F411.h"

// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
#define DIR_PIN 2
#define PWM_PIN PA_0

#define S0 0 
#define S1 1 on/ 
#define S2 2

uint32_t count = 0;
uint32_t state = 0;
volatile static int i = 0;
volatile static float duty = 0;

void setup(void);
void EXTI15_10_IRQHandler(void);
void TIM3_IRQHandler(void);

int main(void) {
	// Initialization --------------------------------------------------
	setup();	
	
	// Infinite Loop ---------------------------------------------------
	while(1){
		GPIO_write(GPIOC, DIR_PIN, LOW);
		PWM_duty(PWM_PIN, duty);
	}		
}

// Initialiization 
void setup(void) {	
	//clock
	RCC_PLL_init();
	SysTick_init();
	// Initialize Input Button
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	// Initialize Direction pin
	GPIO_init(GPIOC, DIR_PIN, OUTPUT);  // calls RCC_GPIOC_enable()
	GPIO_otype(GPIOC, DIR_PIN, EC_OUT_PU);
	// Priority Highest(0) External Interrupt
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	// PWM of 20 msec:  TIM2_CH2 (PA_1 AFmode)
	PWM_init(PWM_PIN);	
	PWM_period(PWM_PIN, 1);   // 1 msec PWM period
	// TIM3 period 1 msec, interrupt of 500mec
	TIM_UI_init(TIM3, 1);
	TIM_UI_enable(TIM3);
}

void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		if(state == 0){
			state = 1; 
			i=0;
		}
		else					 state = 0; duty = 0;
		clear_pending_EXTI(BUTTON_PIN); 
	}
}

void TIM3_IRQHandler(void){
	if(is_UIF(TIM3)){ // update interrupt flag
		//Create the code to rotate angle of RC by 500ms
		if(count%500 == 0) { // 0.5sec : 500
			if(state == 1){
				if(i < 4) duty = 0.25;
				else if (i > 4) duty =0.75;
			}
			else duty = 0;
			i++;
			if(i>8) i = 0;
		}			
		count++;
		clear_UIF(TIM3);    // clear by writing 0
	}
}
```

1. setup

   - clock
   - Initialize Input Button
   - Initialize Direction pin
   - Priority Highest(0) External Interrupt
   - PWM of 20 msec:  TIM2_CH2 (PA_1 AFmode)
   - TIM3 period 1 msec, interrupt of 500mec

2. main

   - Direction pin write LOW

   - PWM generating

3. void EXTI15_10_IRQHandler(void)
   - When the button is pressed, the motor starts or stops
   - void TIM3_IRQHandler(void)
4. void TIM3_IRQHandler(void)
   -  Duty cycles from 0.25->0.75->0.25 every 500msec

#### 5. Results

Results

| State: off                                                   | State: on / duty = 0.25                                      | State: on / duty = 0.75                                      |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image-20231027182048026](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231027182048026.png) | ![image-20231027182037707](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231027182037707.png) | ![image-20231027182101348](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231027182101348.png) |

Demo vedio:  [EC_LAB: Timer & PWM_김선우](https://www.youtube.com/watch?v=GSufnpuP0tk&t=8s)

### Reference

gitbook: [LAB: Timer & PWM - EC (gitbook.io)](https://ykkim.gitbook.io/ec/ec-course/lab/lab-timer-and-pwm)



### Troubleshooting

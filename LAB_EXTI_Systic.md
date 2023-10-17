# LAB: EXTI & SysTick



**Date:** 2023-10-16

**Author:** 22000090_김선우

**Github:**

**Demo Video:** [EC_LAB: EXTI & SysTick_김선우 - YouTube](https://www.youtube.com/watch?v=If-uOYqmQK8&t=18s)

**PDF version:**  Acrobat 2017



## I. Introduction

 In this lab, primarily focuses on the exploration of EXTI (External Interrupt) and the SysTick timer's utilization to control a 7-segment display and examines their potential applications in the field of embedded systems. This research investigates how EXTI can be employed to detect and handle external events, while also highlighting the utility of the SysTick timer, which functions at regular time intervals. The principal objective of this experiment is to delve into the methods for updating a 7-segment display and harnessing these capabilities across diverse application domains.



## II. Requirement

#### Hardware

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 4 LEDs and load resistance
  - 7-segment display(5101ASR)
  - Array resistor (330 ohm)
  - breadboard

#### Software

- Keil uVision, CMSIS, EC_HAL library



## III. Problem 1: Counting numbers on 7-Segment using EXTI Button

### 1. Create HAL library

1. Create files as **ecEXTI.h, ecEXTI.c**

   - Author information at the top of the library code files.

   - Save these files in directory `ENBEDDED \lib\`.

2. Declare and define the following functions in your library : **ecEXTI.h**

### 2. Procedure

1. Create a new project under the directory `\ENBEDDED \LAB\LAB_EXTI`

2. Include updated library in `\ENBEDDED \lib\` to project. 

   - ecGPIO.h, ecGPIO.c

   - ecRCC.h, ecRCC.c

   - ecEXTI.h, ecEXTI.c

3. Use the decoder chip (**74LS47**). Connect it to the bread board and 7-segment display.

4. First, check if every number, 0 to 9, can be displayed properly on the 7-segment.

5. Then, create a code to display the number counting from 0 to 9 and repeats
   - by pressing the push button. (External Interrupt)

### 3. Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment decoder             |
| -------------------------- | --------------------------------------------- |
| Digital In                 | Digital Out                                   |
| PC13                       | PA7, PB6, PC7, PA9                            |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed |

###  . Circuit Diagram

![image-20231006123019396](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231006123019396.png)

### 5. Discussion

1. We can use two different methods to detect an external signal: polling and interrupt. What are the advantages and disadvantages of each approach?

>  Polling periodically checks external events at regular intervals, often implemented using constructs like `while` or `for` loops in C programming. It is a simple approach where event detection occurs predictably due to the periodic checks. However, continuous event polling consumes CPU resources and can lead to inefficiencies.
>
>  Interrupts, on the other hand, respond to external events by interrupting the execution of other tasks when an event occurs. It is an efficient method with low CPU resource consumption, as it only processes events when they happen. However, implementing interrupts can be relatively complex compared to polling, and predicting the timing of interrupt occurrences can be challenging. As a result, it may not be suitable for Real-Time Systems.

2. What would happen if the EXTI interrupt handler does not clear the interrupt pending flag? Check with your code

>  Failure to clear the interrupt pending flag may lead to unexpected behavior and continuous interrupt generation. If the pending flag is continuously set, the interrupt controller continues to generate interrupts until the event conditions are met. This causes system instability and unexpected results. You can check it with the code as follows:
>
> ```c
> EXTI15_10_IRQHandler(void) {
> 	if (is_pending_EXTI(BUTTON_PIN)) {
> 		if(cnt > 9) cnt = 0;
> 		sevensegment_display(cnt);
> 		cnt++;
> 		//clear_pending_EXTI(BUTTON_PIN); 
> 	}
> }
> ```
>
> If comment out the pending clear part and run the code, it is as follows.
>
> | initial state                                                | interrupt assignment                                         |
> | :----------------------------------------------------------- | ------------------------------------------------------------ |
> | <img src="C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017131911268.png" alt="image-20231017131911268" style="zoom:33%;" /> | <img src="C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017132021152.png" alt="image-20231017132021152" style="zoom: 33%;" /> |
>
>  Interrupts occur continuously and all 7 segment displays light up.

### 6. Code

##### 1. main code

```c
#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"
#include "ecSysTick.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

unsigned int cnt = 0;

void setup(void);
void EXTI15_10_IRQHandler(void);

int main(void) {
	// Initialiization 
	setup();
	
	// Inifinite Loop
	while (1) {
	
	}
}

//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		if(cnt > 9) cnt = 0;
		sevensegment_display(cnt);
		cnt++;
		clear_pending_EXTI(BUTTON_PIN); 
	}
}

void setup(void)
{
		RCC_PLL_init();												// System Clock = 84MHz
		// Initialize LED Output
		sevensegment_display_init();   				// calls RCC_GPIOA_enable()	
		// Initialize Input Button
		GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
		GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
		// Priority Highest(0) External Interrupt
		EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
		// sys init
		SysTick_init();
}
```

- set up
  - 7 segment display initialization
  - button pin initialization
  - Interrupt setting
  - System clock setting
- External interrupt code 
  - Each time an interrupt operates, the 7 segment state increases by 1.
  - If count exceeds 9, it is initialized to 0.

##### 2. function code

```c
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecEXTI.h"


void EXTI_init(GPIO_TypeDef* Port, int Pin, int trig_type, int priority) {

	// SYSCFG peripheral clock enable	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Connect External Line to the GPIO
	int EXTICR_port;
	if (Port == GPIOA) EXTICR_port = 0;
	else if (Port == GPIOB) EXTICR_port = 1;
	else if (Port == GPIOC) EXTICR_port = 2;
	else if (Port == GPIOD) EXTICR_port = 3;
	else 										EXTICR_port = 4;

	SYSCFG->EXTICR[Pin/4] &= ~0xFUL << (Pin%4) * 4;			// clear 4 bits
	SYSCFG->EXTICR[Pin/4] |= EXTICR_port << (Pin%4) * 4;			// set 4 bits

	// Configure Trigger edge
	if (trig_type == FALL) EXTI->FTSR |= 1UL << Pin;   // Falling trigger enable 
	else if (trig_type == RISE) EXTI->RTSR |= 1UL << Pin;   // Rising trigger enable 
	else if (trig_type == BOTH) {			// Both falling/rising trigger enable
		EXTI->RTSR |= 1UL << Pin;
		EXTI->FTSR |= 1UL << Pin;
	}

	// Configure Interrupt Mask (Interrupt enabled)
	EXTI->IMR |= 1UL << Pin;     // not masked

	// NVIC(IRQ) Setting
	int EXTI_IRQn = 0;

	if (Pin < 5) 	EXTI_IRQn = EXTI0_IRQn + Pin;
	else if (Pin < 10) 	EXTI_IRQn = EXTI9_5_IRQn;
	else 			EXTI_IRQn = EXTI15_10_IRQn;

	NVIC_SetPriority(EXTI_IRQn, priority);	// EXTI priority
	NVIC_EnableIRQ(EXTI_IRQn); 	// EXTI IRQ enable
}


void EXTI_enable(uint32_t pin) {
	EXTI->IMR |= 1UL << pin;     // not masked (i.e., Interrupt enabled)
}
void EXTI_disable(uint32_t pin) {
	EXTI->IMR |= 0UL << pin;     // masked (i.e., Interrupt disabled)
}

uint32_t is_pending_EXTI(uint32_t pin) {
	uint32_t EXTI_PRx = 1UL << pin;     	// check  EXTI pending 	
	return ((EXTI->PR & EXTI_PRx) == EXTI_PRx);
}


void clear_pending_EXTI(uint32_t pin) {
	EXTI->PR |= 1UL << pin;     // clear EXTI pending 
}
```

- External interrupt initalization code
  - SYSCFG peripheral clock enable	
  - EXTI Initialization & Connect Push-button to EXTI line
  - EXTI trigger decide
  - Decide whether to interrupt masking
  - NVIC(IRQ) Setting
- IMR enable code
- IMR disable code
- Pending Allocation Code
- Pending clear Code

### 7. Results

When the button is pressed, the 7-segment display changes from 0 to 9.

| ![image-20231017134249008](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017134249008.png) | ![image-20231017134810914](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017134810914.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image-20231017134333808](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017134333808.png) | ![image-20231017134352245](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017134352245.png) |
| ![image-20231017134749051](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017134749051.png) | ![image-20231017134731642](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017134731642.png) |
| ![image-20231017134500597](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017134500597.png) | ![image-20231017134631895](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017134631895.png) |
| ![image-20231017134648711](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017134648711.png) | ![image-20231017134525931](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017134525931.png) |

- Demo video link: [EC_LAB: EXTI & SysTick_김선우 - YouTube](https://www.youtube.com/watch?v=If-uOYqmQK8&t=18s)

## IV. Problem 2: Counting numbers on 7-Segment using SysTick

### 1. Create HAL library

1. Rename these files as **ecSysTick.h, ecSysTick.c**

- You MUST write your name and other information at the top of the library code files.
- Save these files in your directory `EC \lib\`.

2. Declare and define the following functions in your library : **ecSysTick.h**

### 2. Procedure

1. Create a new project under the directory`\EC\LAB\LAB_EXTI_SysTick`

- The project name is “**LAB_EXTI_SysTick”.**
- Create a new source file named as “**LAB_EXTI_SysTick.c”**

2. Include your updated library in `\EC\lib\` to your project.

- **ecGPIO.h, ecGPIO.c**
- **ecRCC.h, ecRCC.c**
- **ecEXTI.h, ecEXTI.c**
- **ecSysTick.h, ecSysTick.c**

2. Use the decoder chip (**74LS47**). Connect it to the bread board and 7-segment display.

3. First, check if every number, 0 to 9, can be displayed properly on the 7-segment.

4. Then, create a code to display the number counting from 0 to 9 and repeats at the rate of 1 second.
5. When the button is pressed, it should start from '0' again.

### 3. Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment decoder             |
| -------------------------- | --------------------------------------------- |
| Digital In                 | Digital Out                                   |
| PC13                       | PA7, PB6, PC7, PA9                            |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed |

### 4. Circuit Diagram

![image-20231006123019396](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231006123019396.png)

### 5. Code

1. main code

```c
#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"
#include "ecSysTick.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

unsigned int cnt = 0;

void setup(void);
void EXTI15_10_IRQHandler(void);

int main(void) {
	// Initialiization 
	setup();
	
	// Inifinite Loop
	while (1) {
		sevensegment_display(cnt);
		delay_ms(1000);
		cnt++;
		if (cnt > 9) cnt =0;
		SysTick_reset();	
	
	}
}

//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		cnt = 0;
		sevensegment_display(cnt);
		clear_pending_EXTI(BUTTON_PIN); 
	}
}

void setup(void)
{
		RCC_PLL_init();												// System Clock = 84MHz
		// Initialize LED Output
		sevensegment_display_init();   				// calls RCC_GPIOA_enable()	
		// Initialize Input Button
		GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
		GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
		// Priority Highest(0) External Interrupt
		EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
		// sys init
		SysTick_init();
}
```

- setting
  - System Clock = 84MHz
  - Initialize Input Button
  - Priority Highest(0) External Interrupt
  - sys init
- main code
  - While loop
    - 7-segment display operation 
    - 1 second delay 
    - State increase 
    - State initialization
- Interrupt code
  - Initialize to state 0

2. function code

```c
#include "ecSysTick.h"

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

volatile uint32_t msTicks = 0;

//EC_SYSTEM_CLK

void SysTick_init(void) {
	//  SysTick Control and Status Register
	SysTick->CTRL = 0;											// Disable SysTick IRQ and SysTick Counter

	// Select processor clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

	// uint32_t MCU_CLK=EC_SYSTEM_CLK
	// SysTick Reload Value Register
	SysTick->LOAD = MCU_CLK_PLL / 1000 - 1;						// 1ms, for HSI PLL = 84MHz.

	// SysTick Current Value Register
	SysTick->VAL = 0;

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

	NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC
}



void SysTick_Handler(void) {
	SysTick_counter();
}

void SysTick_counter() {
	msTicks++;
}


void delay_ms(uint32_t mesc) {
	uint32_t curTicks;

	curTicks = msTicks;
	while ((msTicks - curTicks) < mesc);

	msTicks = 0;
}

//void delay_ms(uint32_t msec){
//	uint32_t now=SysTick_val(); 
//	if (msec>5000) msec=5000;
//	if (msec<1) msec=1;
//	while ((now - SysTick_val()) < msec);
//}


void SysTick_reset(void)
{
	// SysTick Current Value Register
	SysTick->VAL = 0;
}

uint32_t SysTick_val(void) {
	return SysTick->VAL;
}

//void SysTick_counter(){
//	msTicks++;
//	if(msTicks%1000 == 0) count++;
//}	
```

- SysTick Control and Status Register
- Select processor clock
- SysTick Reload Value Register
- Enables SysTick exception request
- Enable SysTick IRQ and SysTick Timer
- NVIC setting

### 6. Results

- Every second, the 7-segment display increases by 1 from 0 to 9.
-  Initialized to 0 when an interrupt occurs

| ![image-20231017135811764](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017135811764.png) | ![image-20231017135821069](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017135821069.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image-20231017135859893](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017135859893.png) | ![image-20231017135911898](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017135911898.png) |

- Demo video link:[EC_LAB: EXTI & SysTick_김선우 - YouTube](https://www.youtube.com/watch?v=If-uOYqmQK8&t=18s)

## Ⅴ. Reference

Gitbook: [LAB: EXTI & SysTick - EC (gitbook.io)](https://ykkim.gitbook.io/ec/ec-course/lab/lab-exti-and-systick#introduction)

## Ⅵ. Troubleshooting

Interrupt bouncing

- When the button pin is pressed in an interrupt, bouncing occurs and two states are passed.
- As a solution, a method of adding a delay within the interrupt can be suggested.

![image-20231017135552320](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017135552320.png)

![image-20231017135604698](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231017135604698.png)

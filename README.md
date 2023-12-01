# Documentation

**Date**: 2023-12-01

**Author**: 22000090/김선우



## EC API Documentation

[See Class Github](https://github.com/ykkimhgu/EC-student/blob/main/docs/EC_HAL_Documentation.md) for the example documentation

[See Tutorial: Documentation with Markdown](https://ykkim.gitbook.io/ec/numerical-programming/ta-tutorial/tutorial-documentation-with-markdown#preparation)

You must submit your API documentation before the last day of class

### Embedded Controller - STM32F411 Driver Library

Written by: Your Name

Program: C/C++

IDE/Compiler: Keil uVision 5

OS: WIn10/11

MCU: STM32F411RE, Nucleo-64



### GPIO Digital InOut



##### Header File

```c
#include "ecGPIO.h"
```

```c
// MODER
#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

// IDR & ODR
#define HIGH 1
#define LOW  0

// OUTPUT Type
#define EC_OUT_PU 0 // Push-pull
#define EC_OUT_OP 1	// Open-Drain

// OSPEED
#define EC_LOW 0
#define EC_MEDIUM 1
#define EC_FAST 2
#define EC_HIGH 3

// PUDR
#define EC_NO 0 // No pull-up, pull-down
#define EC_PU 1 // Pull-up 	
#define EC_PD 2 // Pull-down	
#define EC_RE 3 // Reserved

void GPIO_init(GPIO_TypeDef *Port, int pin, int mode); //0
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode); //0
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output); //0
int  GPIO_read(GPIO_TypeDef *Port, int pin); //0 
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed); //0
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type); //0
void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd); //0
```



##### GPIO_init()

Initializes GPIO pins with default setting and Enables GPIO Clock. Mode: In/Out/AF/Analog

```c
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **mode**: INPUT(0), OUTPUT(1), AF(02), ANALOG (03)

**Example code**

```C
GPIO_init(GPIOA, 5, OUTPUT);
GPIO_init(GPIOC, 13, INPUT); //GPIO_init(GPIOC, 13, 0);
```



##### GPIO_mode()

Configures GPIO pin modes: In/Out/AF/Analog

```C
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **mode**: INPUT (0), OUTPUT (1), AF(02), ANALOG (03)

**Example code**

```C
GPIO_mode(GPIOA, 5, OUTPUT);
```



##### GPIO_write()

Write the data to GPIO pin: High, Low

```C
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output); //0
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **output**: LOW(0), HIGH(1)

**Example code**

```C
GPIO_write(GPIOA, 5, 1);  // 1: High
```



##### GPIO_read()

Read the data from GPIO pin

```C
int  GPIO_read(GPIO_TypeDef *Port, int pin);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15

**Example code**

```C
GPIO_read(GPIOC, 13);
```



##### GPIO_ospeed()

Configures output speed of GPIO pin : Low, Mid, Fast, High

 ```C
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
 ```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **speed**: LOW_SPEED(0), MID_SPEED(1), FAST_SPEED(2) , HIGH_SPEED(3)

**Example code**

```c
GPIO_ospeed(GPIOA, 5, 2);  // 2: FAST_SPEED
```



##### GPIO_otype()

Configures output type of GPIO pin: Push-Pull / Open-Drain

```c
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **type**: PUSH_PULL(0), OPEN_DRAIN(1)

**Example code**

```c
GPIO_otype(GPIOA, 5, 0);  // 0: Push-Pull
```



##### GPIO_pupd()

Configures Pull-up/Pull-down mode of GPIO pin: No Pull-up, Pull-down/ Pull-up/ Pull-down/ Reserved

```c
void GPIO_pupdr(GPIO_TypeDef* Port, int pin, int pupd);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **pupd**: NO_PUPD(0), PULL_UP(1), PULL_DOWN(2), RESERVED(3)

**Example code**

```c
GPIO_pupdr(GPIOA, 5, 0);  // 0: No Pull-up, Pull-down
```



### RCC

```c
#include "ecRCC.h"
```

```c
void RCC_HSI_init(void);
void RCC_PLL_init(void);
void RCC_GPIOA_enable(void);
void RCC_GPIOB_enable(void);
void RCC_GPIOC_enable(void);
void RCC_GPIOD_enable(void);
```



##### RCC_HSI_init()

Configure system clock 16MHz

**Example code**

```c
RCC_HSI_init();
```



##### RCC_PLL_init()

Configure system clock 84MHz

**Example code**

```c
RCC_PLL_init();
```



##### RCC_GPIOA_enable()

Configure enable GPIOA Clock

**Example code**

```c
RCC_GPIOA_enable();
```



##### RCC_GPIOB_enable()

Configure enable GPIOB Clock

**Example code**

```c
RCC_GPIOB_enable();
```



##### RCC_GPIOC_enable()

Configure enable GPIOC Clock

**Example code**

```c
RCC_GPIOC_enable();
```



##### RCC_GPIOD_enable()

Configure enable GPIOD Clock

**Example code**

```c
RCC_GPIOD_enable();
```



### EXTI



##### **Header file**

```c
#include "ecEXTI.h"
```

```c
#define FALL 0
#define RISE 1
#define BOTH 2

void EXTI_init(GPIO_TypeDef* Port, int pin, int trig, int priority);
void EXTI_enable(uint32_t pin);
void EXTI_disable(uint32_t pin);
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);
```



##### EXTI_init()

Initializer EXTI pins with default setting and Enables EXTI trig: Fall/Rise/Both , priority: 0~15

```c
void EXTI_init(GPIO_TypeDef* Port, int pin, int trig, int priority);
```

**Parameters**

- **Port:** Port Number, GPIOA~GPIOH
- **pin**: pin number (int) 0~15
- **trig**: FALL(0), RISE(1), BOTH(02)
- **priority**: 0~15 (0: most important priority, 15: least important priority)

**Example code**

```C
EXTI_init(GPIOC, BUTTON_PIN, FALL, 0); // BUTTON_PIN = 13
```



##### EXTI_enable()

Configures enables EXTI

```c
void EXTI_enable(uint32_t pin);
```

**Parameters**

- **pin**: pin number (int) 0~15

**Example code**

```c
EXTI_enable(BUTTON_PIN); // BUTTON_PIN = 13
```



##### EXTI_disable()

Configures disables EXTI

```c
void EXTI_disable(uint32_t pin);
```

**Parameters**

- **pin**: pin number (int) 0~15

**Example code**

```c
EXTI_disable(BUTTON_PIN); // BUTTON_PIN = 13
```



##### is_pending_EXTI()

Configures pending EXTI

```c
uint32_t is_pending_EXTI(uint32_t pin);
```

**Parameters**

- **pin**: pin number (int) 0~15

**Example code**

```c
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		if(cnt > 9) cnt = 0;
		sevensegment_display(cnt);
		cnt++;
		clear_pending_EXTI(BUTTON_PIN); 
	}
}
```



##### clear_pending_EXTI()

Configure clear pending EXTI

```c
void clear_pending_EXTI(uint32_t pin);
```

Configures pending clear EXTI

**Parameters**

- **pin**: pin number (int) 0~15

**Example code**

```c
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		if(cnt > 9) cnt = 0;
		sevensegment_display(cnt);
		cnt++;
		clear_pending_EXTI(BUTTON_PIN); 
	}
}
```



### SysTick



##### **Header file**

```c
#include "ecSysTick.h"
```

```c
extern volatile uint32_t msTicks;
void SysTick_init(void);
void SysTick_Handler(void);
void SysTick_counter();
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);
```



##### SysTick_init()

Initial system clock setting

```c
void SysTick_init(void);
```

**Example code**

```c
SysTick_init();
```



##### SysTick_Handler()

SysTick_counter() occurrence

```c
void SysTick_Handler(void);
```

**Example code**

```c
SysTick_Handler();
```



##### SysTick_counter()

Counting seconds in milliseconds

```c
void SysTick_counter();
```

**Example code**

```c
SysTick_counter();
```



##### delay_ms()

Delay occurs in milliseconds

```c
void delay_ms(uint32_t msec);
```

**Parameters**

- **msec**: delay value in milliseconds

**Example code**

```c
while(1) {
    LED_toggle();
    delay_ms(1000); // led toggle every 1 sec
}
```



##### SysTick_reset()

SysTick Current Value reset

```c
void SysTick_reset(void);
```

**Example code**

```c
while (1) {
	sevensegment_display(cnt);
	delay_ms(1000);
	cnt++;
	if (cnt > 9) cnt =0;
	SysTick_reset();	
}
```



##### SysTick_val()

SysTick Current Value

```c
uint32_t SysTick_val(void);
```

**Examplecode**

```c
read = SysTick_val(); // read current val
```



### TIM



##### **Header file**

```c
#include "ecTIM.h"
```

```c
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



##### TIM_init(TIM_TypeDef* TIMx, uint32_t msec)

Initial timer settings and timer period settings

```c
void TIM_init(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

- **TIMx:** TIM Number, TIM1~TIM5
- **msec**: TIM period (milliseconds)

**Example code**

```c
TIM_init(TIM3, 1); //TIM3 period 1 msec
```



##### TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec)

Timer period settings at microseconds

```c
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);
```

**Parameters**

- **TIMx:** TIM Number, TIM1~TIM5
- **usec**: TIM period (microseconds)

**Example code**

```c
TIM_period_us(TIM3, 1); //TIM3 period 1 usec
```



##### TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec)

Timer period settings at milliseconds

```c
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

- **TIMx:** TIM Number, TIM1~TIM5
- **msec**: TIM period (milliseconds)

**Example code**

```c
TIM_period_ms(TIM3, 1); //TIM3 period 1 msec
```



##### TIM_period(TIM_TypeDef* TIMx, uint32_t msec)

Timer period settings at milliseconds

```c
void TIM_period(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

- **TIMx:** TIM Number, TIM1~TIM5
- **msec**: TIM period (milliseconds)

**Example code**

```c
TIM_period(TIM3, 1); //TIM3 period 1 msec
```



##### TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec)

Initial timer interrupt settings and timer period settings

```c
void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

- **TIMx:** TIM Number, TIM1~TIM5
- **msec**: TIM period (milliseconds)

**Example code**

```c
TIM_UI_init(TIM3, 1); //TIM3 period 1 usec
```



##### TIM_UI_enable(TIM_TypeDef* TIMx)

Timer interrupt enable

```c
void TIM_UI_enable(TIM_TypeDef* TIMx);
```

**Parameters**

- **TIMx:** TIM Number, TIM1~TIM5

**Example code**

```c
TIM_UI_enable(TIM3);
```



##### TIM_UI_disable(TIM_TypeDef* TIMx)

Timer interrupt disable

```c
void TIM_UI_disable(TIM_TypeDef* TIMx);
```

**Parameters**

- **TIMx:** TIM Number, TIM1~TIM5

**Example code**

```c
TIM_UI_disable(TIM3);
```



##### is_UIF(TIM_TypeDef* TIMx)

pending Timer

```c
uint32_t is_UIF(TIM_TypeDef* TIMx);
```

**Parameters**

- **TIMx:** TIM Number, TIM1~TIM5

**Example code**

```c
if(is_UIF(TIM3)){ // update interrupt flag
	// executable code
	clear_UIF(TIM3);    // clear by writing 0
}
```



##### clear_UIF(TIM_TypeDef* TIMx)

clear pending

```c
void clear_UIF(TIM_TypeDef* TIMx);
```

**Parameters**

- **TIMx:** TIM Number, TIM1~TIM5

**Example code**

```c
if(is_UIF(TIM3)){ // update interrupt flag
	// executable code
	clear_UIF(TIM3);    // clear by writing 0
}
```



### PWM



##### **Header file**

```c
#include "ecPWM.h"
```

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



##### PWM_init()

```c
void PWM_init(PinName_t pinName);
```

**Parameter**

- **pinName**: Use the pin corresponding to the timer (PA_5 -> TIM_ch1)

**Example code**

```c
PWM_init(PA_5);
```



##### PWM_pinmap()

```c
void PWM_pinmap(PinName_t pinName, TIM_TypeDef** TIMx, int* chN);
```

**Parameter**

- **pinName**: Use the pin corresponding to the timer (PA_5 -> TIM_ch1)
- **chN**: TIM channel

**Example code**

```c
PWM_pinmap(PA_5, TIMx, chN) // TIMx & chN determined by pin
```



##### PWM_period()

```c
void PWM_period(PinName_t pinName, uint32_t msec);
```

**Parameter**

- **pinName**: Use the pin corresponding to the timer (PA_5 -> TIM_ch1)
- **msec**: pwm period at millisecond

**Example code**

```c
PWM_period(PA_5, 1);
```



##### PWM_period_ms()

```c
void PWM_period_ms(PinName_t pinName, uint32_t msec);	
```

**Parameter**

- **pinName**: Use the pin corresponding to the timer (PA_5 -> TIM_ch1)
- **msec**: pwm period at millisecond

**Example code**

```c
PWM_period_ms(PA_5, 1);
```



##### PWM_period_us()

```c
void PWM_period_us(PinName_t pinName, uint32_t usec);
```

**Parameter**

- **pinName**: Use the pin corresponding to the timer (PA_5 -> TIM_ch1)
- **usec**: pwm period at microsecond

**Example code**

```c
PWM_period_us(PA_5, 1);
```



##### PWM_pulsewidth()

```c
void PWM_pulsewidth(PinName_t pinName, uint32_t pulse_width_ms);
```

**Parameter**

- **pinName**: Use the pin corresponding to the timer (PA_5 -> TIM_ch1)
- **pulse_width_ms**: pwm pulse width period at millisecond

**Example code**

```c
PWM_pulsewidth(PA_5, 1);
```



##### PWM_pulsewidth_ms()

```c
void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms);
```

**Parameter**

- **pinName**: Use the pin corresponding to the timer (PA_5 -> TIM_ch1)
- **pulse_width_ms**: pwm pulse width period at millisecond

**Example code**

```c
PWM_pulsewidth_ms(PA_5, 1);
```



##### PWM_pulsewidth_us()

```c
void PWM_pulsewidth_us(PinName_t pinName, uint32_t pulse_width_us);
```

**Parameter**

- **pinName**: Use the pin corresponding to the timer (PA_5 -> TIM_ch1)
- **pulse_width_ms**: pwm pulse width period at microsecond

**Example code**

```c
PWM_pulsewidth_us(PA_5, 1);
```



##### PWM_duty()

```c
void PWM_duty(PinName_t pinName, float duty);
```

**Parameter**

- **pinName**: Use the pin corresponding to the timer (PA_5 -> TIM_ch1)
- **duty**: pwm duty (0~1)

**Example code**

```c
PWM_duty(PA_5, 0.5);
```



### Stepper



##### **Header file**

```c
#include "ecStepper.h"
```

```c
//State mode
#define HALF 0
#define FULL 1	 
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
	GPIO_TypeDef *port1;
	int pin1;
	GPIO_TypeDef *port2;
	int pin2;
	GPIO_TypeDef *port3;
	int pin3;
	GPIO_TypeDef *port4;
	int pin4;
	uint32_t _step_num;
} Stepper_t;

	 
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
void Stepper_setSpeed(long whatSpeed);
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 
void Stepper_stop(void);
void Stepper_pinOut (uint32_t state, uint32_t mode);
```



##### Stepper_init()

```c
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
```

**Parameter**

- port1~4: port GPIOA~GPIOH
- pin1~4: pin number 0~15  

**Example code**

```c
Stepper_init(GPIOB, 10, GPIOB, 4, GPIOB, 5, GPIOB, 3);
```



##### Stepper_setSpeed()

```c
void Stepper_setSpeed(long whatSpeed);
```

**Parameter**

- **whatSpeed**: 0~14 

**Example code**

```c
	Stepper_setSpeed(14);
```



##### Stepper_step()

```c
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 
```

**Parameter**

- **steps**: motor step
- **direction**: 0 or 1 (ccr or cr)
- **mode**: HALF: 0 / FULL: 1

**Example code**

```c
Stepper_step(2048, 1, HALF);
```



##### Stepper_stop()

```c
void Stepper_stop(void);
```

**Example code**

```c
Stepper_stop ();
```



##### Stepper_pinOut()

```c
void Stepper_pinOut (uint32_t state, uint32_t mode);
```

**Parameter**

- **state**: motor state
- **mode**: HALF: 0 / FULL: 1

**Example code**

```c
state = FSM_half[state].next[direction];
tepper_pinOut(state, mode);
```



### UART



##### **Header file**

```c
#include "ecUART.h"
```

```c
#define POL 0
#define INT 1

// You can modify this
#define BAUD_9600	9600
#define BAUD_19200	19200
#define BAUD_38400  	38400
#define BAUD_57600	57600
#define BAUD_115200 	115200
#define BAUD_921600 	921600


// ********************** USART 2 (USB) ***************************
// PA_2 = USART2_TX
// PA_3 = USART2_RX
// Alternate function(AF7), High Speed, Push pull, Pull up
// APB1
// **********************************************************

// ********************** USART 1 ***************************
// PA_9 = USART1_TX (default)	// PB_6  (option)
// PA_10 = USART1_RX (default)	// PB_3 (option)
// APB2
// **********************************************************

// ********************** USART 6 ***************************
// PA_11 = USART6_TX (default)	// PC_6  (option)
// PA_12 = USART6_RX (default)	// PC_7 (option)
// APB2
// **********************************************************

// Configuration UART 1, 2 using default pins 
void UART1_init(void);
void UART2_init(void);	
void UART1_baud(uint32_t baud);
void UART2_baud(uint32_t baud);

// USART write & read
void USART1_write(uint8_t* buffer, uint32_t nBytes);
void USART2_write(uint8_t* buffer, uint32_t nBytes);
uint8_t USART1_read(void);										
uint8_t USART2_read(void);	

// RX Inturrupt Flag USART1,2
uint32_t is_USART1_RXNE(void);
uint32_t is_USART2_RXNE(void);

// private functions
void USART_write(USART_TypeDef* USARTx, uint8_t* buffer, uint32_t nBytes);
void USART_init(USART_TypeDef* USARTx, uint32_t baud);  		
void UART_baud(USART_TypeDef* USARTx, uint32_t baud);											
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);
uint8_t USART_read(USART_TypeDef * USARTx);										
void USART_setting(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, uint32_t baud); 
void USART_delay(uint32_t us);  
```



##### UART1_init()

Initialization USART1 communication

```C
void UART1_init(void);
```

**Example code**

```c
UART1_init();
```



##### UART2_init()

Initialization USART2 communication

```c
void UART2_init(void);	
```

**Example code**

```c
UART2_init();
```



##### UART1_baud()

```c
void UART1_baud(uint32_t baud);
```

**Parameter**

- **baud**: Determined by bandwidth of communication device

**Example code**

```c
UART1_baud(BAUD_9600);
```



##### UART2_baud()

```c
void UART2_baud(uint32_t baud);
```

**Parameter**

- **baud**: Determined by bandwidth of communication device

**Example code**

```c
UART2_baud(BAUD_9600);
```



##### USART1_write()

```c
void USART1_write(uint8_t* buffer, uint32_t nBytes);
```

**Parameter**

- **buffer**:  letters to input
- **nBytes**: number of character bytes

**Example code**

```c
USART1_write((uint8_t*)"MOD: ", 5);
```



##### USART2_write()

```c
void USART2_write(uint8_t* buffer, uint32_t nBytes);
```

**Parameter**

- **buffer**:  letters to input
- **nBytes**: number of character bytes

**Example code**

```c
USART2_write((uint8_t*)"MOD: ", 5);
```



##### USART1_read()									

```c
uint8_t USART1_read(void);										
```

**Example code**

```c
BT_Data = USART1_read(USART1);
```



##### USART2_read()

```c
uint8_t USART2_read(void);
```

**Example code**

```c
BT_Data = USART2_read(USART1);
```



##### is_USART1_RXNE()

```c
uint32_t is_USART1_RXNE(void);
```

**Example code**

```c
is_USART1_RXNE();
```



##### is_USART2_RXNE()

```c
uint32_t is_USART2_RXNE(void);
```

**Example code**

```c
is_USART2_RXNE();
```



##### USART_write()

```c
void USART_write(USART_TypeDef* USARTx, uint8_t* buffer, uint32_t nBytes);
```

**Parameter**

- **USARTx**: USART1, USART2
- **buffer**:  letters to input
- **nBytes**: number of character bytes

**Example code**

```c
USART_write(USART1, (uint8_t*)"MOD: ", 5);
```



##### USART_init()	

```c
void USART_init(USART_TypeDef* USARTx, uint32_t baud);  		
```

**Parameter**

- **USARTx**: USART1, USART2
- **baud**: Determined by bandwidth of communication device

**Example code**

```c
USART_init(USART1, 9600);
```



##### UART_baud()					

```c
void UART_baud(USART_TypeDef* USARTx, uint32_t baud);									
```

**Parameter**

- **USARTx**: USART1, USART2
- **baud**: Determined by bandwidth of communication device

**Example code**

```c
UART_baud(USART1, 9600);
```



##### is_USART_RXNE()

```c
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);
```

**Parameter**

- **USARTx**: USART1, USART2

**Example code**

```c
is_USART_RXNE(USART1);
```



##### USART_read()								

```c
uint8_t USART_read(USART_TypeDef * USARTx);										
```

**Parameter**

- **USARTx**: USART1, USART2

**Example code**

```c
BT_Data = USART_read(USART1);
```



##### USART_setting()

```c
void USART_setting(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, uint32_t baud); 
```

**Parameter**

- **USARTx**: USART1, USART2
- **GPIO_TX**: GPIOA~GPIOH
- **pinTX**: pin number
- **GPIO_RX**: GPIOA~GPIOH
- **pinRX**: pin number
- **baud**: Determined by bandwidth of communication device

**Example code**

```c
USART_setting(USART1, GPIOA, 9, GPIOA, 10, 9600);
USART_setting(USART2, GPIOA, 2, GPIOA, 3, 9600);
```



##### USART_delay()

```c
void USART_delay(uint32_t us);  
```

**Parameter**

- us: microsecond delay

**Example code**

```c
USART_delay(300);
```



### ICAP



##### **Header file**

```c
#include "ecICAP.h"
```

```c
// ICn selection according to CHn
#define FIRST 1
#define SECOND 2

// Edge Type
#define IC_RISE 0
#define IC_FALL 1
#define IC_BOTH 2

// Input Capture Number
#define IC_1    1
#define IC_2    2
#define IC_3    3
#define IC_4    4

//Input Capture

void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);

void ICAP_init(PinName_t pinName);
void ICAP_setup(PinName_t pinName, int ICn, int edge_type);
void ICAP_counter_us(PinName_t pinName, int usec);
uint32_t ICAP_capture	(TIM_TypeDef* TIMx, uint32_t ICn);

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);  // CCnum= 1~4
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);
```



##### ICAP_pinmap()

```c
void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
```

**Parameter**

- **pinName**: ICAP setting pin (ex: PA_5, PC13)
- **TIMx**: setting timer by pin
- **chN**: setting timer channel by pin

**Example code**

```c
ICAP_pinmap(PA_5, &TIMx, &CHn); // TIMx & CHn determined by pin
```



##### ICAP_init()

```c
void ICAP_init(PinName_t pinName);
```

**Parameter**

- **pinName**: ICAP setting pin (ex: PA_5, PC13)

**Example code**

```c
ICAP_init(ECHO); // ECHO pin: PB_6
```



##### ICAP_setup()

```c
void ICAP_setup(PinName_t pinName, int ICn, int edge_type);
```

**Parameter**

- **pinName**: ICAP setting pin (ex: PA_5, PC13)
- **edge_type**: RISE 0 / FALL 1 / BOTH 2

**Example code**

```c
ICAP_setup(ECHO, IC_1, IC_RISE);
```



##### ICAP_counter_us()

```c
void ICAP_counter_us(PinName_t pinName, int usec);
```

**Parameter**

- **pinName**: ICAP setting pin (ex: PA_5, PC13)
- **usec**: ICAP period microsecond

**Example code**

```c
ICAP_counter_us(ECHO, 10);
```



##### ICAP_capture()

```c
uint32_t ICAP_capture	(TIM_TypeDef* TIMx, uint32_t ICn);
```

**Parameter**

- **TIMx**: TIM1~TIM5
- **ICn**: Inputcapture number IC_1~IC_4

**Example code**

```c
time1 = ICAP_capture(TIM4, IC_1);
```



##### is_CCIF()

```c
uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);  // CCnum= 1~4
```

**Parameter**

- **TIMx**: TIM1~TIM5
- **CCum**: flag setting

**Example code**

```c
if(is_CCIF(TIM4, IC_1)){ // TIM4_Ch1 (IC1) Capture 	  Flag. Rising Edge Detect
		time1 = ICAP_capture(TIM4, IC_1);		// Capture TimeStart
		clear_CCIF(TIM4, 1);                	// clear capture/compare interrupt flag 
}	
```



##### clear_CCIF()

```c
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);
```

**Parameter**

- **TIMx**: TIM1~TIM5
- **CCum**: flag setting

**Example code**

```c
if(is_CCIF(TIM4, IC_1)){ // TIM4_Ch1 (IC1) Capture 	  Flag. Rising Edge Detect
		time1 = ICAP_capture(TIM4, IC_1);		// Capture TimeStart
		clear_CCIF(TIM4, 1);                	// clear capture/compare interrupt flag 
}
```



### ADC



##### **Header file**

```c
#include "ecADC.h"
```

```c
// ADC init
// Default:  one-channel mode, continuous conversion
// Default: HW trigger - TIM3 counter, 1msec
void ADC_init(PinName_t pinName);

// Multi-Channel Scan Sequence 
void ADC_sequence(PinName_t *seqCHn, int seqCHnums); 

// ADC start
void ADC_start(void);

// flag for ADC interrupt
uint32_t is_ADC_EOC(void);
uint32_t is_ADC_OVR(void);
void clear_ADC_OVR(void);

// read ADC value
uint32_t ADC_read(void);

/////////////////////////////////////////////////////
// Advanced Setting
/////////////////////////////////////////////////////
// Conversion mode change: CONT, SINGLE / Operate both ADC,JADC
void ADC_conversion(int convMode); 					
void ADC_trigger(TIM_TypeDef* TIMx, int msec, int edge);

// Private Function
void ADC_pinmap(PinName_t pinName, uint32_t *chN);
```



##### ADC_init()

```c
void ADC_init(PinName_t pinName);
```

**Parameter**

- **pinName**: ADC pin setting (ex: PA_5, PB_0)

**Example code**

```c
ADC_init(PB_0);
```



##### ADC_sequence()

```c
void ADC_sequence(PinName_t *seqCHn, int seqCHnums); 
```

**Parameter**

- seqCHn: pin setting by squence
- seqCHnums: Number of ADCs used (1~4)

**Example code**

```c
PinName_t seqCHn[2] = {PB_0, PB_1};
ADC_sequence(seqCHn, 2);
```



##### ADC_start()

```c
void ADC_start(void);
```

**Example code**

```c
ADC_start();	
```



##### is_ADC_EOC()

```c
uint32_t is_ADC_EOC(void);
```

**Example code**

```c
	if(is_ADC_EOC()){		// after finishing sequence
		if (flag==0)
			value1 = ADC_read();  
		else if (flag==1)
			value2 = ADC_read();
		flag =! flag;		// flag toggle
	}
```



##### is_ADC_OVR()

```c
uint32_t is_ADC_OVR(void);
```

**Example code**

```c
if(is_ADC_OVR())
	clear_ADC_OVR();
```



##### clear_ADC_OVR()

```c
void clear_ADC_OVR(void);
```

**Example code**

```c
if(is_ADC_OVR())
	clear_ADC_OVR();
```



##### ADC_read()

```c
uint32_t ADC_read(void);
```

**Example code**

```c
value1 = ADC_read();
```



##### ADC_conversion()

```c
void ADC_conversion(int convMode); 			
```

**Parameter**

- **convMode**: continuous mode: 0, single mode: 1

**Example code**

```c
ADC_conversion(CON); // CON: 0
```



##### ADC_trigger()

```c
void ADC_trigger(TIM_TypeDef* TIMx, int msec, int edge);
```

**Parameter**

- **TIMx**: timer setting TIM1~5
- **msec**: ADC trig period millisecond
- **edge**: RISE: 1 / FALL: 2 / BOTH: 3

**Example code**

```c
ADC_trigger(TIM3, 1000, RISE);
```



##### ADC_pinmap()

```c
void ADC_pinmap(PinName_t pinName, uint32_t *chN);
```

**Parameter**

- **pinName**: ADC pin name (ex: PA_5, PB_6)
- **chN**: channel determined by pin

**Example code**

```c
ADC_pinmap(pinName, &chN); // pinName = PA_5, chN: determined pin
```





​                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          

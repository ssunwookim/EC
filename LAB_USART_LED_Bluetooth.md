# LAB: USART - LED, Bluetooth



**Date:** 2023-11-17

**Author:** 22000090 김선우

**Github:** 

**Demo Video:** [EC_LAB: USART - LED, Bluetooth_김선우](https://www.youtube.com/watch?v=HkN5WkfINg0)



## Introduction

In this lab, we will learn how to configure and use ‘USART(Universal synchronous asynchronous receiver transmitter)’ of MCU. Then, we will learn how to communicate between your PC and MCU and MCU to another MCU with wired serial communication.

- **Mission 1**: Control LED(LD2) of each other MCU.

- **Mission 2**: Run DC motors with Bluetooth

#### Requirement

##### Hardware

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - DC motor, DC motor driver(L9110s),
  - Bluetooth Module(HC-06),

##### Software

- Keil uVision, CMSIS, EC_HAL library



## Problem 1: EC HAL library

- Update header files located in the directory `EC \lib\ecUART.h`.

```c
#ifndef __EC_USART_H
#define __EC_USART_H

#include <stdio.h>
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"

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

#endif
```



## Problem 2: Communicate MCU1-MCU2 using RS-232

#### Procedure

1. Create a new project under the directory `\repos\EC\LAB\LAB_USART_LED`

   - The project name is “**LAB_USART_LED”.**

   - Create a new source files named as “**LAB_USART_LED.c”**

2. Include your updated library in `\repos\EC\lib\` to your project.

   - **ecGPIO.h, ecGPIO.c**

   - **ecRCC.h, ecRCC.c**
   - **ecUART.h, ecUART.c**
   - and other necessary header files

3. Connect each MCUs to each PC with **USART 2** via USB cable (ST-Link)

   - MCU1-PC1, MCU2-PC2

4. Connect MCU1 to MCU2 with **USART 1**

   - connect RX/TX pins externally as

     - MCU1_TX to MCU2_RXD

     - MCU1_RX - MCU2_TX

5. Send a message from PC_1 by typing keys on Teraterm. It should send that message from MCU_1 to MCU_2.

6. The received message by MCU_2 should be displayed on PC_2.

7. Turn other MCU's LED(LD2) On/OFF by sending text:

   - "**L**" for Turn OFF

   - "**H**" for Turn ON

#### Configuration

| Type                         | Port - Pin         | Configuration                                                |
| ---------------------------- | ------------------ | ------------------------------------------------------------ |
| System Clock                 |                    | PLL 84MHz                                                    |
| USART2 : USB cable (ST-Link) |                    | No Parity, 8-bit Data,                                                                            1-bit Stop bit, 38400 baud-rate |
| USART1 : MCU1 - MCU2         | TXD: PA9 RXD: PA10 | No Parity, 8-bit Data,                                                                             1-bit Stop bit, 38400 baud-rate |
| Digital Out: LD2             | PA5                |                                                              |



#### Code

- main code

```c
#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSysTick.h"


static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
uint8_t PC_string[]="Loop:\r\n";

void setup(void){
	RCC_PLL_init();
	SysTick_init();
	
	GPIO_init(GPIOA, 5, 1);
	// USART2: USB serial init
	UART2_init();
	UART2_baud(BAUD_9600);

	// USART1: BT serial init 
	UART1_init();
	UART1_baud(BAUD_9600);
}

int main(void){	
	setup();
	printf("MCU Initialized\r\n");	
	while(1){
		// USART Receive: Use Interrupt only
		// USART Transmit:  Interrupt or Polling
		USART2_write(PC_string, 7);
		
		if(BT_Data == 'H'){
			GPIO_write(GPIOA, 5, HIGH);
		}
		else if(BT_Data == 'L') GPIO_write(GPIOA, 5, LOW);
		
		delay_ms(2000);        
	}
}

void USART2_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART2_RXNE()){
		PC_Data = USART2_read();		// RX from UART2 (PC)
		USART2_write(&PC_Data,1);		// TX to USART2	 (PC)	 Echo of keyboard typing
		USART1_write(&PC_Data,1);
		
	}
}

void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART1_RXNE()){
		BT_Data = USART1_read();		// RX from UART1 (BT)		
		printf("RX: %c \r\n",BT_Data); // TX to USART2(PC)
		
	}
}
```

- setup
  - system clock setting
  - LED pin setting
  - USART1 & USART2 setting
- USART1 IRQHandler
  - btData = USART1 data read
- USART2 IRQHandler
  - pcData = USART2 data read
  - USART data write
- main
  - LED state
    - bt data = H -> LED high
    - bt data = L -> LED low

#### Result

| **State** | **User1-Input**                                              | **User2-Output**                                             |
| --------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| High      | ![image-20231117102922325](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117102922325.png) | ![image-20231117103008434](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117103008434.png) |
| Low       | ![image-20231117103052480](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117103052480.png) | ![image-20231117103112371](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117103112371.png) |
| **State** | **User2-Input**                                              | **User1-Output**                                             |
| High      | ![image-20231117103150683](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117103150683.png) | ![image-20231117103211020](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117103211020.png) |
| Low       | ![image-20231117103237186](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117103237186.png) | ![image-20231117103258680](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117103258680.png) |



- Demo vedio :  [EC_LAB: USART - LED, Bluetooth_김선우](https://www.youtube.com/watch?v=HkN5WkfINg0)



## Problem 3: Control DC Motor via Bluetooth



#### Require

1. Bluetooth (HC-06)	

![HC-06 Bluetooth Module: Pinout, Datasheet pdf and Arduino Connection](https://res.utmel.com/Images/UEditor/9d192091-981c-4930-b2d7-b5cf990950a4.jpg)

2. DC Motor Driver (L9110s)

![img](https://user-images.githubusercontent.com/38373000/200288512-5a3d57a2-6d98-410b-bf02-57646cde6fd4.png)

- A- IA: PWM pin (0~100% duty) for Motor A
- A- IB: Direction Pin (Digital Out H or L) for Motor B



#### Procedure

1. Create a new project under the directory `\repos\EC\LAB\LAB_USART_Bluetooth

   - The project name is “**LAB_USART_Bluetooth”.**

   - Create a new source files named as “**LAB_USART_Bluetooth.c”**

2. Include your updated library in `\repos\EC\lib\` to your project.
   - **ecGPIO.h, ecGPIO.c**
   - **ecRCC.h, ecRCC.c**
   - **ecUART.h, ecUART.c**
   - **ecTIM.h, ecTIM.c**

3. Connect the MCU to PC via Bluetooth. Use USART 1
   - connect RX/TX pins as
     - MCU TXD - BLUE RXD
     - MCU RXD - BLUE TXD

4. Check the Bluetooth connection by turning MCU's LED(LD2) On/OFF by sending text of "**0**" or "**1**" from PC.

5. Run 2 DC motors(Left-wheel, Right-wheel) to steer.

   - Turn Left: MotorA / MotorB = (50 / 80%) duty

   - Turn Right: MotorA / MotorB = (80 / 50%) duty

   - Go straight: MotorA / MotorB = (80 / 80 %) duty

   - STOP: MotorA / MotorB = (0 / 0 %) duty



#### Configuration

| Type                    | Port - Pin         | Configuration                                         |
| ----------------------- | ------------------ | ----------------------------------------------------- |
| System Clock            |                    | PLL 84MHz                                             |
|                         |                    |                                                       |
| USART1 : MCU -Bluetooth | TXD: PA9 RXD: PA10 | No Parity, 8-bit Data, 1-bit Stop bit, 9600 baud-rate |
| Digital Out: LD2        | PA5                |                                                       |
| PWM (Motor A)           | TIM2-Ch1           | PWM period (2kHz~10kHz)                               |
| PWM (Motor B)           | TIM2-Ch2           |                                                       |



#### Code

- main

```c
#include "stm32f4xx.h"
#include "ecSTM32F411.h"

#define M1 PA_0
#define M2 PA_1

static volatile uint8_t BT_Data = 0;

void setup(void){
	RCC_PLL_init();
	SysTick_init();
	
	UART2_init();
	UART2_baud(BAUD_38400);
	// USART1: BT serial init 
	UART1_init();
	UART1_baud(BAUD_9600);
	// PWM M1 of 20 msec:  TIM2_CH2 (PA_0 AFmode)
	PWM_init(M1);	
	PWM_period(M1, 1);   // 1 msec PWM period
	// PWM M1 of 20 msec:  TIM2_CH2 (PA_0 AFmode)
	PWM_init(M2);	
	PWM_period(M2, 1);   // 1 msec PWM period
	// Direction pin 1
	GPIO_init(GPIOC, 2, OUTPUT);
	// Direction pin 2
	GPIO_init(GPIOC, 3, OUTPUT);
    // LED pin
	GPIO_init(GPIOA, 5, OUTPUT);
}

int main(void){	
	
	setup();
	printf("MCU Initialized\r\n");	
	USART_write(USART1,(uint8_t*) "Hello bluetooth\r\n",17);
	
	GPIO_write(GPIOC, 2, HIGH);
	GPIO_write(GPIOC, 3, HIGH);
	
	while(1){
				
		// USART Receive: Use Interrupt only
		// USART Transmit:  Interrupt or Polling
				// straite
		if(BT_Data == 'W'){
			GPIO_write(GPIOA, 5, HIGH);
			PWM_duty(M1, 0.8);
			PWM_duty(M2, 0.8);
		}
		// right
		else if(BT_Data == 'D'){
			GPIO_write(GPIOA, 5, LOW);
			PWM_duty(M1, 0.8);
			PWM_duty(M2, 0.5);
		}			
		// left
		else if(BT_Data == 'A'){
			PWM_duty(M1, 0.5);
			PWM_duty(M2, 0.8);
		}
		//stop		
		else if(BT_Data == 'S'){
			PWM_duty(M1, 0);
			PWM_duty(M2, 0);
		}
		else if(BT_Data == '0'){
			GPIO_write(GPIOA, 5, LOW);
		}
		else if(BT_Data == '1'){
			GPIO_write(GPIOA, 5, HIGH);
		}
		
	}
}


void USART1_IRQHandler(){         //USART1 INT 
	if(is_USART_RXNE(USART1)){
		BT_Data = USART_read(USART1);
		USART_write(USART1,(uint8_t*) "BT sent : ", 10);
		USART_write(USART1, &BT_Data, 1);
		USART_write(USART1, "\r\n", 2);		
	}
}
```

- setup
  - sys clock setting
  - USART1 & USART2 init
  - PWM_M1 & PWM_M2 pin setting
  - direction pin setting
  - LED pin setting
- USART1 IRQHandler
  - BT data read
  - BT data write
- main
  - BT data 'w' -> straight
  - BT data 'A' -> left
  - BT data 'D' -> right
  - BT data 'S' -> stop
  - BT data '0' -> LED off
  - BT data '1' -> LED on

#### Result

- Result

| State    | Input                                                        | Output                                                       |
| -------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| straight | ![image-20231117125536970](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117125536970.png) | ![image-20231117113556832](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117113556832.png) |
| right    | ![image-20231117125604319](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117125604319.png) | ![image-20231117113623807](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117113623807.png) |
| left     | ![image-20231117125625655](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117125625655.png) | ![image-20231117113616974](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117113616974.png) |
| stop     | ![image-20231117125643142](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117125643142.png) | ![image-20231117113608363](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117113608363.png) |
| LED on   | ![image-20231117125752577](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117125752577.png) | ![image-20231117130149852](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117130149852.png) |
| LED off  | ![image-20231117125735323](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117125735323.png) | ![image-20231117130301790](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231117130301790.png) |

- Demo vedio :  [EC_LAB: USART - LED, Bluetooth_김선우](https://www.youtube.com/watch?v=HkN5WkfINg0)



## Reference

Gitbook: [LAB: USART - LED, Bluetooth - EC (gitbook.io)](https://ykkim.gitbook.io/ec/ec-course/lab/lab-usart-led-bluetooth)



## Troubleshooting

(Option) You can write Troubleshooting section

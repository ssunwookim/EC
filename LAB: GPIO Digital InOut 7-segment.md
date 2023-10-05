# LAB: GPIO Digital InOut 7-segment



**Date:** 2023-10-05

**Author:** 김선우

**Github:** repository link

**Demo Video:**  [EC_LAB: GPIO Digital InOut 7-segment_김선우](https://www.youtube.com/watch?v=RwqiFNbPHk0&t=13s)

**PDF version:**  Acrobat 2017



## I. Introduction

In this lab, you are required to create a simple program to control a 7-segment display to show a decimal number (0~9) that increases by pressing a push-button.



## II. Requirement

#### 1. Hardware 

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 7-segment display(5101ASR)
  - Array resistor (330 ohm)
  - decoder chip(74LS47)
  - breadboard

#### 2. Software

- Keil uVision, CMSIS, EC_HAL library



## III. Exercise

| **Port/Pin**   | **Description**              | **Register setting**          |
| -------------- | ---------------------------- | ----------------------------- |
| Port A Pin 5   | Clear Pin5 mode              | GPIOA->MODER &=~(3<<(5*2))    |
| Port A Pin 5   | Set Pin5 mode = Output       | GPIOA->MODER \|= 1<<(5*2)     |
| Port A Pin 6   | Clear Pin6 mode              | GPIOA->MODER &=~(3<<(6*2))    |
| Port A Pin 6   | Set Pin6 mode = Output       | GPIOA->MODER \|= 1<<(6*2)     |
| Port A Pin Y   | Clear PinY mode              | GPIOA->MODER &=~(3<<(Y*2))    |
| Port A Pin Y   | Set PinY mode = Output       | GPIOA->MODER \|= 1<<(Y*2)     |
| Port A Pin 5~9 | Clear Pin5~9 mode            | GPIOA->MODER &=~(1023<<(5*2)) |
|                | Set Pin5~9 mode = Output     | GPIOA->MODER \|= 341<<(5*2)   |
| Port X Pin Y   | Clear Pin Y mode             | GPIOX->MODER &=~(~(3<<(Y*2))) |
|                | Set Pin Y mode = Output      | GPIOX->MODER \|= 1<<(Y*2)     |
| Port A Pin5    | Set Pin5 otype=push-pull     | GPIOA->OTYPER = 0<<5          |
| Port A PinY    | Set PinY otype=push-pull     | GPIOA-> OTYPER = 0<<Y         |
| Port A Pin5    | Set Pin5 ospeed=Fast         | GPIOA->OSPEEDR = 2<<(5*2)     |
| Port A PinY    | Set PinY ospeed=Fast         | GPIOA-> OSPEEDR = 2<<(Y*2)    |
| Port A Pin 5   | Set Pin5 PUPD=no pullup/down | GPIOA->OTYPER = 0<<(5*2)      |
| Port A Pin Y   | Set PinY PUPD=no pullup/down | GPIOA-> OTYPER = 0<<(Y*2)     |



## IV. Problem 1: Connecting 7-Segment Display

### 1. Procedure

 Instead of using the decoder chip, we are going to make the 7-segment decoder with the MCU programming.

- Connect the common anode 7-segment with the given array resistors.
- Apply VCC and GND to the 7-segment display.
- Apply arduino pin to any 7-segment pin 'a'~'g' and observe if that LED is turned on or off

### 2. Connection Diagram

![image-20231006072835296](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231006072835296.png)

- Using Pin: PA5, PA6, PA7, PB6, PC7, PA9, PA8, PB10
- Each pin was sequentially connected to a, b, c, d, e, f, g, and dot of the 7 segments.

### 3. Discussion

Q1. Draw the truth table for the BCD 7-segment decoder with the 4-bit input.

![image-20231006073233636](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231006073233636.png)

Q2. What are the common cathode and common anode of 7-segment display?

- Common Cathode 
  - In a common cathode configuration of a 7-segment display, all the cathodes of the individual LEDs are interconnected and serve as a common connection. Each segment (a, b, c, d, e, f, g) possesses its dedicated anode. To activate a specific segment and illuminate it, a positive voltage is applied to the corresponding anode, while the common cathode is connected to ground. This structural arrangement provides precise control over which segments are activated, achieved by applying a high voltage selectively to the anodes of the desired segments.

- Common Anode
  - In a common anode configuration of a 7-segment display, all the anodes of the individual LEDs are interconnected and function as a shared common connection. Each segment (a, b, c, d, e, f, g) is equipped with its own cathode. To activate a specific segment and enable its illumination, a ground is applied to the respective cathode, while a positive voltage is supplied to the common anode. This arrangement is in contrast to the common cathode setup, as it necessitates the application of a low voltage to the cathodes of the segments one wishes to activate.

Q3. Does the LED of a 7-segment display (common anode) pin turn ON when 'HIGH' is given to the LED pin from the MCU?

- In a common anode 7-segment display, the LED segments do not turn on when 'HIGH' (high voltage) is applied to the MCU's LED pin. Instead, to turn on individual segments, you must apply 'LOW' (low voltage or ground, 0V) to the corresponding LED pin.



## Ⅴ. Problem 2: Display 0~9 with button press

### 1. Procedure

1. Create a new project under the directory `\repos\EC\LAB\LAB_GPIO_7segment`

2. Include updated library in `\repos\EC\lib\` to project.

   - ecGPIO.h, ecGPIO.c

   - ecRCC.h, ecRCC.c

3. Declare and Define the following functions in your library

   ```ㅊ
   void sevensegment_init(void); 
   void sevensegment_decoder(uint8_t  num);
   ```

4. Implement the following behavior

   - First, check if every number, 0 to 9, can be displayed properly
   - Then, create a code to display the number from 0 to 9 with each button press. After the number '9', it should start from '0' again.

### 2. Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment                                    |
| -------------------------- | ------------------------------------------------------------ |
| Digital In                 | Digital Out                                                  |
| PC13                       | PA5, PA6, PA7, PB6, PC7, PA9, PA8, PB10 ('a'~'h', respectively) |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed                |

### 3. Code

1. Main code

```c
#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	unsigned int cnt = 0;
	unsigned int lastState = 0;
									
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		sevensegment_decoder(cnt);
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
			if(lastState == 1) cnt++;
			lastState = 0;
		}
		else lastState = 1;
		
		if (cnt > 9) cnt = 0;
		
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	sevensegment_init(); // Definition of conditions for pins
	
}
```

- Load header file
- Definition of LED pins and button pins
- Initial value setting
  - Definition of detailed conditions of button pins and LED pins

- 'while' operation
  - sevensegment_decoder(cnt); Call the function to determine the seven-segment display operation.
  - When the button pin is pressed, it moves to the next number.
    - Define the last state so that the state changes only when the button pin changes from 1 to 0.
  -  If count exceeds 9, it is initialized to 0.

2. Function code

```c
void sevensegment_init(){
	
  GPIO_init(GPIOA, 5, OUTPUT);
	GPIO_otype(GPIOA, 5, 0);
	GPIO_pupd(GPIOA, 5, EC_NO);
	GPIO_ospeed(GPIOA,5, EC_MEDIUM);	
	
  GPIO_init(GPIOA, 6, OUTPUT); 
	GPIO_otype(GPIOA, 6, 0);
	GPIO_pupd(GPIOA, 6, EC_NO);
	GPIO_ospeed(GPIOA, 6, EC_MEDIUM);
	
  GPIO_init(GPIOA, 7, OUTPUT);       
	GPIO_otype(GPIOA, 7, 0);
	GPIO_pupd(GPIOA, 7, EC_NO);
	GPIO_ospeed(GPIOA, 7, EC_MEDIUM);	
  
  GPIO_init(GPIOB, 6, OUTPUT);
	GPIO_otype(GPIOB, 6, 0);
	GPIO_pupd(GPIOB, 6, EC_NO);
	GPIO_ospeed(GPIOB, 6, EC_MEDIUM);	
	
	GPIO_init(GPIOC, 7, OUTPUT);
	GPIO_otype(GPIOC, 7, 0);
	GPIO_pupd(GPIOC, 7, EC_NO);
	GPIO_ospeed(GPIOC, 7, EC_MEDIUM);	
	
	GPIO_init(GPIOA, 9, OUTPUT);       
	GPIO_otype(GPIOA, 9, 0);
	GPIO_pupd(GPIOA, 9, EC_NO);
	GPIO_ospeed(GPIOA, 9, EC_MEDIUM);	
	
	GPIO_init(GPIOA, 8, OUTPUT);       
	GPIO_otype(GPIOA, 8, 0);
	GPIO_pupd(GPIOA, 8, EC_NO);
	GPIO_ospeed(GPIOA, 8, EC_MEDIUM);	
	
	GPIO_init(GPIOB, 10, OUTPUT);
	GPIO_otype(GPIOB, 10, 0);
	GPIO_pupd(GPIOB, 10, EC_NO);
	GPIO_ospeed(GPIOB, 10, EC_MEDIUM);	
}

void sevensegment_decoder(uint8_t  num){
	 
	int ledState[10][8]={
	
	//LED              a b c d e f g dot
                    {0,0,0,0,0,0,1,0},          //zero
                    {1,0,0,1,1,1,1,0},          //one
                    {0,0,1,0,0,1,0,0},          //two
                    {0,0,0,0,1,1,0,0},          //three
                    {1,0,0,1,1,0,0,0},          //four
                    {0,1,0,0,1,0,0,0},          //five
                    {0,1,0,0,0,0,0,0},          //six
                    {0,0,0,1,1,0,1,0},          //seven
                    {0,0,0,0,0,0,0,0},          //eight
                    {0,0,0,0,1,0,0,0},          //nine
									};
		
		GPIO_write(GPIOA, 5, ledState[num][0]); 	//LED a
		GPIO_write(GPIOA, 6, ledState[num][1]); 	//LED b
		GPIO_write(GPIOA, 7, ledState[num][2]); 	//LED c
		GPIO_write(GPIOB, 6, ledState[num][3]); 	//LED d
		GPIO_write(GPIOC, 7, ledState[num][4]); 	//LED e
		GPIO_write(GPIOA, 9, ledState[num][5]); 	//LED f
		GPIO_write(GPIOA, 8, ledState[num][6]); 	//LED g
		GPIO_write(GPIOB, 10, ledState[num][7]);	//LED dot
}
```

- Function of ' void sevensegment_init(); '
  - Define detailed conditions for PA5, PA6, PA7, PB6, PC7, PA9, PA8, and PB10.
- Function of ' void sevensegment_decoder(uint8_t  num); '
  - Define LED state according to number.
  - An LED state is given to each pin according to its number.

### 4. Results

Demo video link: [EC_LAB: GPIO Digital InOut 7-segment_김선우](https://www.youtube.com/watch?v=RwqiFNbPHk0&t=13s)



## Ⅵ. Problem 3: Using both 7-Segment Decoder and 7-segment display

### 1. Procedure

- use the decoder chip (**74LS47**). Connect it to the bread board.
- Implement display operations from 0 to 9 with Only 4 Digital out pins of MCU

### 2. Connection Diagram

![image-20231006083348092](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231006083348092.png)

### 3. Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment                     |
| -------------------------- | --------------------------------------------- |
| Digital In                 | Digital Out                                   |
| PC13                       | PA7, PB6, PC7, PA9                            |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed |

### 4. Code

```c
void sevensegment_display_init(){
	
	GPIO_init(GPIOA, 7, OUTPUT);       
	GPIO_otype(GPIOA, 7, 0);
	GPIO_pupd(GPIOA, 7, EC_NO);
	GPIO_ospeed(GPIOA, 7, EC_MEDIUM);	
  
  GPIO_init(GPIOB, 6, OUTPUT);
	GPIO_otype(GPIOB, 6, 0);
	GPIO_pupd(GPIOB, 6, EC_NO);
	GPIO_ospeed(GPIOB, 6, EC_MEDIUM);	
	
	GPIO_init(GPIOC, 7, OUTPUT);
	GPIO_otype(GPIOC, 7, 0);
	GPIO_pupd(GPIOC, 7, EC_NO);
	GPIO_ospeed(GPIOC, 7, EC_MEDIUM);	
	
	GPIO_init(GPIOA, 9, OUTPUT);       
	GPIO_otype(GPIOA, 9, 0);
	GPIO_pupd(GPIOA, 9, EC_NO);
	GPIO_ospeed(GPIOA, 9, EC_MEDIUM);	

}

void sevensegment_display(uint8_t  num){
	int number[10][4] = {
												{0,0,0,0},          //zero
												{0,0,0,1},          //one
												{0,0,1,0},          //two
												{0,0,1,1},          //three
												{0,1,0,0},          //four
												{0,1,0,1},          //five
												{0,1,1,0},          //six
												{0,1,1,1},          //seven
												{1,0,0,0},          //eight
												{1,0,0,1},          //nine
											};
	
		GPIO_write(GPIOA, 7, number[num][0]); 	//binary 2^3
		GPIO_write(GPIOB, 6, number[num][1]); 	//binary 2^2
		GPIO_write(GPIOC, 7, number[num][2]); 	//binary 2^1
		GPIO_write(GPIOA, 9, number[num][3]); 	//binary 2^0
}
```

- Function of ' void sevensegment_display_init(); '
  - Define detailed conditions for  PA7, PB6, PC7, PA9
- Function of ' void sevensegment_display(uint8_t  num); '
  - Define number of decoder according to state of number.
  - An LED state is given to each pin according to its number.

## Ⅶ. Reference

Gitbook: [LAB: GPIO Digital InOut 7-segment - EC (gitbook.io)](https://ykkim.gitbook.io/ec/ec-course/lab/lab-gpio-digital-inout-7segment)

## Ⅷ. Troubleshooting

```c
#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int cnt = 0;
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		sevensegment_decode(cnt % 10);
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0) cnt++; 
		if (cnt > 9) cnt = 0;
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	sevensegment_init();
}
```

- The following code, when executed, caused the phenomenon of numbers incrementing unnaturally. Upon investigation, it was found that the while loop iteration was progressing faster than the button presses, leading to this occurrence.

```c
sevensegment_decoder(cnt);
if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
	if(lastState == 1) cnt++;
		lastState = 0;
	}
else lastState = 1;
if (cnt > 9) cnt = 0;
```

- To address this issue, a "last state" was defined to recognize the button pin only when it transitions from 1 to 0.

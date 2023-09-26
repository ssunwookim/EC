# LAB: GPIO Digital InOut

## LAB: Smart mini-fan with STM32-duino

**Date**: 2023/9/26

**Author**: 김선우

**Github**: [EC/LAB Smart mini-fan with STM32-duino.md at main · ssunwookim/EC (github.com)](https://github.com/ssunwookim/EC/blob/main/LAB Smart mini-fan with STM32-duino.md)

**Demo Video**:  [EC_LAB: GPIO Digital InOut_김선우 ](https://www.youtube.com/watch?v=RwqiFNbPHk0&t=7s)

**PDF version:** Acrobat 2017

## I. Introduction

 This experiment aims to execute a straightforward program for toggling several LEDs using push-button input. Furthermore, the experiment involves the creation of a HAL driver for GPIO digital input/output control and the establishment of an environment in which the library can be employed.



## II. Requirement

### 1. Hardware

- NUCLEO -F401RE
  - NUCLEO-F411RE
- Actuator/Sensor/Others
  - LEDs x 3
  - Resistor 330 ohm x 3, breadboard

### 2. Software

- Keil uVision, CMSIS, EC_HAL library



## III. Problem 1: Create EC_HAL library

### 1. Procedure

- Create the library directory

![image-20230926084852267](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230926084852267.png)

- Save header library files in this directory.

![image-20230926085205979](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230926085205979.png)

![image-20230926085304702](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230926085304702.png)

1) Create HAL group for header files

2) Specify an 'Include Path' for your project's header files

3. Add header files

- Create own library for Digital_In and Out

#### ecRCC.h

```c
void RCC_HSI_init(void);
void RCC_PLL_init(void);
void RCC_GPIOA_enable(void);
void RCC_GPIOB_enable(void);
void RCC_GPIOC_enable(void);
void RCC_GPIOD_enable(void);
```

#### ecGPIO.h

```c
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode); //0
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output); //0
int  GPIO_read(GPIO_TypeDef *Port, int pin); //0 
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode); //0
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed); //0
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type); //0
void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd); //0
```

- Example code

```c
/* ecGPIO.c  */

// write Output value
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output){
	Port->ODR &= ~(1UL<<pin);
	Port->ODR |= Output<<pin;
}
```



## IV. Problem 2: Toggle LED with Button

### 1. Procedure

1. Create a new project under the directory `\ENBEDDED\LAB\`

   - The project name is “**LAB_GPIO_DIO_LED”.**

   - Name the source file as “**LAB_GPIO_DIO_LED.c”**

2. Include your library **ecGPIO.h, ecGPIO.c** in `\ENBEDDED\lib\`.

3. Toggle the LED by pushing the button.
   - Push button (LED ON), Push Button (LED OFF) and repeat

### 2. Configuration

| Button (B1)   | LED                               |
| ------------- | --------------------------------- |
| Digital In    | Digital Out                       |
| GPIOC, Pin 13 | GPIOA, Pin 5                      |
| PULL-UP       | Open-Drain, Pull-up, Medium Speed |

### 3. Code

```c
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN 	  5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0)	GPIO_write(GPIOA, LED_PIN, HIGH);
		else 					GPIO_write(GPIOA, LED_PIN, LOW);
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_otype(GPIOA, LED_PIN, 1);
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);
	GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);
}
```

- LED PIN: 5, BUTTON PIN: 13
- Setup part
  - The button pin corresponds to GPIOC-13. Since the button is used as an input, the mode is set to input. The button pin environment is configured according to the configuration conditions. (condition: PULL UP)
  - The LED pin is associated with GPIOA-5. Since the LED uses an output value, the mode is set to output, and the LED pin environment is configured based on the configuration conditions. (condition: Open-Drain, Pull-up, Medium Speed)
- main
  - In the main code, toggling is carried out using a `while` loop. When the button is pressed and enters a state of 0, the LED pin is set to output HIGH. In the state when the button pin is not pressed and remains at 1, the LED pin is configured to output LOW.

### 4. Discussion

1. Find out a typical solution for software debouncing and hardware debouncing.

- "Bouncing phenomenon" refers to the occurrence of brief and rapid contact closures and openings of a switch's contacts when performing input and output operations with a switch. This results in momentary and irregular voltage fluctuations. To mitigate this bouncing phenomenon, both software-based and hardware-based debouncing methods can be employed.

  **Software-Based Debouncing:**

  - **Time Delay:** One approach to address bouncing is to introduce a time delay when the button is initially pressed. This delay allows control over the momentary bouncing that occurs.
  - **State Change Detection:** By continuously monitoring the state changes of the button, action is taken only when a change persists for a certain predefined period. Rapid state changes occurring within a short time frame are disregarded.
  - **Variable Threshold:** When the button is initially pressed, a variable is set, and subsequent state changes are ignored unless the button's value exceeds a predefined threshold. Typically, during bouncing, the voltage changes are most significant during the initial input, so this is used as a reference to filter out bouncing signals.

  **Hardware-Based Debouncing:**

  - **RC Filter:** An RC (Resistor-Capacitor) filter can be implemented by adding a capacitor to absorb and eliminate voltage fluctuations caused by bouncing. This results in smoothing the input signal.
  - **Flip-Flops or Counters:** Using flip-flops or counters, the state of the button, when pressed, can be stabilized for a specific clock cycle duration. This ensures that the button input remains in a stable state, mitigating bouncing effects.

2. What method of debouncing did this NUCLEO board use for the push-button(B1)?

-  The debounce mechanism was implemented using the 'GPIO_pupd' function, configuring it with Pull-Up settings to maintain the input pin at a high voltage state by default. When the button is pressed, the input pin undergoes a transition from a high voltage to a low voltage state. This transition from a high voltage to a low voltage state serves to alleviate the bouncing issue.

### 5. Results

Experiment result: [EC_LAB: GPIO Digital InOut_김선우 ](https://www.youtube.com/watch?v=RwqiFNbPHk0&t=7s)



## Ⅴ. Problem 3: Toggle LED with Button

### 1. Procedure

1. Create a new project under the directory `\ENBEDDED\LAB\`

   - The project name is “**LAB_GPIO_DIO_multiLED”.**

   - Name the source file as “**LAB_GPIO_DIO_multiLED.c”**

2. Include your library **ecGPIO.h, ecGPIO.c** in `\ENBEDDED\lib\`.

3. Connect 4 LEDs externally with necessary load resistors.
   - As Button B1 is Pressed, light one LED at a time, in sequence.
   - Example: LED0--> LED1-->LED2-->LED3-->LED0….

### 2. Configuration

| Button        | LED                              |
| ------------- | -------------------------------- |
| Digital In    | Digital Out                      |
| GPIOC, Pin 13 | PA5, PA6, PA7, PB6               |
| PULL-UP       | Push-Pull, Pull-up, Medium Speed |

### 3. Circuit Diagram

![img](https://424033796-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-MgmrEstOHxu62gXxq1t%2Fuploads%2Fgit-blob-7a04d146dc24c70b2ad3b5f4db67d96a4c252e9f%2Fimage.png?alt=media)

![image-20230926105126285](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230926105126285.png)

-  In the experiment, the pins GPIOA-5, GPIOA-6, GPIOA-7, and GPIOB-6 were used. The physical locations of these pins were determined based on the microcontroller's datasheet. Each of these pins was utilized as an input to control the respective LEDs. A common ground was shared among them.

### 4. Code

```c
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN     5
#define BUTTON_PIN  13

#define S0 0
#define S1 1
#define S2 2
#define S3 3

int State = S0;
int lastState = S3;
int ledState[4] = { 5, 6, 7, 6};
int bt = 0;

void setup(void);

int main(void) { 
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while (1) {
        if (State > S3) {
            State = S0;
        }
        
        if (GPIO_read(GPIOC, BUTTON_PIN) == 0) {
			if (State == S0) {
							GPIO_write(GPIOB, ledState[S3], LOW);
							GPIO_write(GPIOA, ledState[S2], LOW);
							GPIO_write(GPIOA, ledState[S1], LOW);
							GPIO_write(GPIOA, ledState[S0], HIGH);
							
			}
			else if(State == S1 ){
				GPIO_write(GPIOB, ledState[S3], LOW);
				GPIO_write(GPIOA, ledState[S2], LOW);
				GPIO_write(GPIOA, ledState[S1], HIGH);
				GPIO_write(GPIOA, ledState[S0], LOW);
							
			}
			else if(State == S2 ){
				GPIO_write(GPIOB, ledState[S3], LOW);
				GPIO_write(GPIOA, ledState[S2], HIGH);
				GPIO_write(GPIOA, ledState[S1], LOW);
				GPIO_write(GPIOA, ledState[S0], LOW);
							
			}
			else {
				GPIO_write(GPIOB, ledState[S3], HIGH);
				GPIO_write(GPIOA, ledState[S2], LOW);
				GPIO_write(GPIOA, ledState[S1], LOW);
				GPIO_write(GPIOA, ledState[S0], LOW);
			}
						
			if(bt == 1){
				State++;
			}
        }		
		bt = GPIO_read(GPIOC, BUTTON_PIN);		
    }
}

void setup(void) {
    RCC_HSI_init();
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);  
		GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	
    GPIO_init(GPIOA, LED_PIN, OUTPUT);
		GPIO_otype(GPIOA, LED_PIN, 0);
		GPIO_pupd(GPIOA, LED_PIN, EC_PU);
		GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);	
	
    GPIO_init(GPIOA, 6, OUTPUT); 
		GPIO_otype(GPIOA, 6, 0);
		GPIO_pupd(GPIOA, 6, EC_PU);
		GPIO_ospeed(GPIOA, 6, EC_MEDIUM);
	
    GPIO_init(GPIOA, 7, OUTPUT);       
		GPIO_otype(GPIOA, 7, 0);
		GPIO_pupd(GPIOA, 7, EC_PU);
		GPIO_ospeed(GPIOA, 7, EC_MEDIUM);	
  
    GPIO_init(GPIOB, 6, OUTPUT);
		GPIO_otype(GPIOB, 6, 0);
		GPIO_pupd(GPIOB, 6, EC_PU);
		GPIO_ospeed(GPIOB, 6, EC_MEDIUM);		
}
```

- LED PIN: 5, BUTTON PIN: 13
- define State: S0~3, ledState, button State(bt)
- Setup part
  - The button pin corresponds to GPIOC-13. Since the button is used as an input, the mode is set to input. The button pin environment is configured according to the configuration conditions. (condition: PULL UP)
  - The pins and conditions for four LEDs are configured as follows: Each of the four LEDs is connected to GPIOA-5, GPIOA-6, GPIOA-7, and GPIOB-6, respectively. The conditions include Push-Pull output mode, Pull-up configuration, and Medium Speed.
- main
  - Through an 'if' statement, it turns on the LED corresponding to each state, while turning off all LEDs associated with the remaining states.-
  - Using an 'if' statement, it updates the state if the previous button state was 1.
  - It stores the current button state in the button state variable.
  - This algorithm toggles the LED when the button state changes from 1 to 0.

### 5. Results

- Results image

##### State: S0

![image-20230926120038570](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230926120038570.png)

##### State: S2![image-20230926120112084](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230926120112084.png)

##### State: S1![image-20230926120143870](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230926120143870.png)

##### State: S3![image-20230926120209648](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230926120209648.png)

The figure above show the LED being toggled. Each time the button is pressed, the state changes and is toggled.

- Experiment result: [EC_LAB: GPIO Digital InOut_김선우 ](https://www.youtube.com/watch?v=RwqiFNbPHk0&t=7s)

### 6. Discussion

1. Find out a typical solution for software debouncing and hardware debouncing. What method of debouncing did this NUCLEO board use for the push-button(B1)?

   - "Bouncing phenomenon" refers to the occurrence of brief and rapid contact closures and openings of a switch's contacts when performing input and output operations with a switch. This results in momentary and irregular voltage fluctuations. To mitigate this bouncing phenomenon, both software-based and hardware-based debouncing methods can be employed.

     **Software-Based Debouncing:**

     - **Time Delay:** One approach to address bouncing is to introduce a time delay when the button is initially pressed. This delay allows control over the momentary bouncing that occurs.
     - **State Change Detection:** By continuously monitoring the state changes of the button, action is taken only when a change persists for a certain predefined period. Rapid state changes occurring within a short time frame are disregarded.
     - **Variable Threshold:** When the button is initially pressed, a variable is set, and subsequent state changes are ignored unless the button's value exceeds a predefined threshold. Typically, during bouncing, the voltage changes are most significant during the initial input, so this is used as a reference to filter out bouncing signals.

     **Hardware-Based Debouncing:**

     - **RC Filter:** An RC (Resistor-Capacitor) filter can be implemented by adding a capacitor to absorb and eliminate voltage fluctuations caused by bouncing. This results in smoothing the input signal.
     - **Flip-Flops or Counters:** Using flip-flops or counters, the state of the button, when pressed, can be stabilized for a specific clock cycle duration. This ensures that the button input remains in a stable state, mitigating bouncing effects.

   - What method of debouncing did this NUCLEO board use for the push-button(B1)?
     -  The debounce mechanism was implemented using the 'GPIO_pupd' function, configuring it with Pull-Up settings to maintain the input pin at a high voltage state by default. When the button is pressed, the input pin undergoes a transition from a high voltage to a low voltage state. This transition from a high voltage to a low voltage state serves to alleviate the bouncing issue.

## Ⅵ. Reference

- Gitbook: [LAB: GPIO Digital InOut - EC (gitbook.io)](https://ykkim.gitbook.io/ec/ec-course/lab/lab-gpio-digital-inout#procedure-1)

## Ⅶ. Troubleshooting

- Include problem

![image-20230926120638338](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230926120638338.png)

 During the experiments, a recurring issue was encountered due to a path configuration error. The root cause was identified as the failure to initiate the build phase by pressing the F7 key. Upon pressing the F7 key to complete the build phase and load the configuration, the experiments proceeded without any further issues.

- Button Recognition Issue

 During the experiments, it was frequently observed that pressing the button did not transition the state as expected. Debugging revealed instances where the button press was not being recognized. While one possible solution is to lower the button recognition voltage for improved sensitivity, this may lead to frequent bouncing issues. Hence, finding an appropriate compromise point in this regard is necessary.

# LAB: Stepper Motor



**Date:** 2023-11-27

**Author:** 22000090/김선우

**Github:** 

**Demo Video:**  [EC_LAB: Stepper Motor_김선우](https://www.youtube.com/watch?v=h36xFq5gyjU)



## Introduction

In this lab, we will learn how to drive a stepper motor with digital output of GPIOs of MCU. You will use a FSM to design the algorithm for stepper motor control.



### Requirement

#### Hardware

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 3Stepper Motor 28BYJ-48
  - Motor Driver ULN2003
  - breadboard

#### Software

- Keil uVision, CMSIS, EC_HAL library



## Problem 1: Stepper Motor



#### 1. Hardware Connection

![img](https://user-images.githubusercontent.com/91526930/197428440-9f4a9c8c-2d81-4d0e-a4e2-b4a4b9def44d.png)

![img](https://user-images.githubusercontent.com/91526930/197428469-a0d7a8fa-ba4c-482f-8688-ea87cfd9f4e0.png)



#### 2. Stepper Motor Sequence

##### Full-stepping sequence

![img](https://user-images.githubusercontent.com/91526930/197428513-f9a23147-3448-4bed-bda2-c90325b8c143.png)

![image-20231110101538166](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231110101538166.png)

##### Half-stepping sequence

![img](https://user-images.githubusercontent.com/91526930/197429006-d552ab16-0bbf-4c52-bdce-a0f2bfe5f0d8.png)

![image-20231110102926601](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231110102926601.png)



#### 3. Finite State Machine

- Full-Stepping Sequence

![image-20231110101624434](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231110101624434.png)

- Half-Stepping Sequence

![image-20231110101759587](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231110101759587.png)



## Problem 2: Firmware Programming



#### Create HAL library

- Create library file  `EC \lib\ecStepper.h`,  `EC \lib\ecStepper.c`.

##### ecStepper.h

```c
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



#### Procedure

1. Create a new project under the directory `\EC\LAB\LAB_Stepper_Motor`
   - The project name is “**LAB_Stepper_Motor”.**
   - Create a new source file named as “**LAB_Stepper_Motor.c”**
2. Include your updated library in `\repos\EC\lib\` to your project.
   - **ecGPIO.h, ecGPIO.c**
   - **ecRCC.h, ecRCC.c**
   - **ecEXTI.h, ecEXTI.c**
   - **ecSysTick.h**, **ecSysTick.c**
   - **ecStepper.h** **ecStepper.h**
3. Connect the MCU to the motor driver and the stepper motor.
4. Find out the number of steps required to rotate 1 revolution using Full-steppping.
5. Then, rotate the stepper motor 10 revolutions with 2 rpm. Measure if the motor rotates one revolution per second.
6. Repeat the above process in the opposite direction.
7. Increase and decrease the speed of the motor as fast as it can rotate to find the maximum and minimum speed of the motor.
8. Apply the half-stepping and repeat the above.



#### Configuration

| Digital Out                                                  | SysTick |
| ------------------------------------------------------------ | ------- |
| PB10, PB4, PB5, PB3                                                                                NO Pull-up Pull-down                                                                                  Push-Pull                                                                                                               Fast | delay() |



#### Discussion

1. Find out the trapezoid-shape velocity profile for a stepper motor. When is this profile necessary?

   > ![How to Calculate Velocity from Triangular and Trapezoidal Move Profiles](https://www.linearmotiontips.com/wp-content/uploads/2016/08/Trapezoidal-Move-Profile-740.jpg)
   >
   >  Trapezoid-shaped velocity profile is one of the common motion profiles used to control the movement of stepper motors. This profile is characterized by three distinct phases: acceleration, constant velocity, and deceleration. These three steps are respectively: The stepper motor accelerates from rest to the desired speed. Acceleration occurs at a constant rate, with speed increasing linearly with time. The goal here is to reach the desired speed as quickly as possible without overshooting or causing other problems. Once the desired speed is reached, the motor maintains a constant speed. This step is necessary in applications that require stable, predictable motor speeds. During the deceleration phase, the motor slows down to reach a stopping point. Similar to the acceleration phase, deceleration occurs at a constant rate. This trapezoid speed profile is designed to provide smooth, stable operation of the motor while still accurately reaching the target point.

2. How would you change the code more efficiently for micro-stepping control? You don’t have to code this but need to explain your strategy.

   >  Microstepping control can be achieved using an interpolation algorithm. An interpolation algorithm is a technique that generates intermediate microstep values between full steps. In a trapezoidal velocity profile, the movement between full steps is linear. Therefore, linear interpolation can be used to calculate the position between each full step. Linear interpolation is a method of accurately calculating intermediate positions by connecting a straight line between the current position and the target position. Linear interpolation is a simple yet effective method, but more sophisticated interpolation algorithms are also available. Smoother motions can be achieved through advanced interpolation techniques, for example cubic interpolation.



#### Code

1. main code

```c
#include "stm32f411xe.h"
#include "ecSTM32F411.h"


#define BUTTON_PIN 13

void setup(void);
volatile static int count = 0;
volatile static int revolutions = 1;
int main (void) {
	// initialization
	setup();
	
	while(1){
		if(count<revolutions){
			Stepper_step(2048, 1, FULL);
		}
		count++;
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
	
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);

	Stepper_init(GPIOB, 10, GPIOB, 4, GPIOB, 5, GPIOB, 3);
	Stepper_setSpeed(14);
}

void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		Stepper_stop ();
		clear_pending_EXTI(BUTTON_PIN); 
	}
}
```

- setup
  - clock set
  - Initialization input button pin
  - Interrupt set
  - Stepper input pin set
  - Stepper motor speed set
- main
  - while (1)
    - Determine the number of revolutions based on the count
- EXTI pending

### Results

1. Results image

| FULL mode                                                    | HALF mode                                                    |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image-20231110124327863](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231110124327863.png) | ![image-20231110124344550](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20231110124344550.png) |

- The experiment was conducted with the stepper motor in full step and half step mode. When operating 10 times at 2 rpm in full step mode, a total of 10 rotations were performed in 5 minutes, and when operating 10 times at 2 rpm in half step mode, a total of 5 rotations were confirmed in 5 minutes. In both modes, the smallest speed was 1rpm and the fastest speed was 14rpm.

Demo vedio link: [EC_LAB: Stepper Motor_김선우](https://www.youtube.com/watch?v=h36xFq5gyjU)

## Reference

gitbook: [LAB: Stepper Motor - EC (gitbook.io)](https://ykkim.gitbook.io/ec/ec-course/lab/lab-stepper-motor)

## Troubleshooting


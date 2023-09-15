# LAB: Smart mini-fan with STM32-duino

## LAB: Smart mini-fan with STM32-duino

**Date**: 2023/9/15

**Author**: 김선우

**Github**: [repository link](https://github.com/ssunwookim/EC/blob/main/LAB%20Smart%20mini-fan%20with%20STM32-duino.md)

**Demo Video**: [EC_LAB: Smart mini-fan with STM32-duino_김선우 - YouTube](https://www.youtube.com/watch?v=qafxAc4mhHY)



## I. Introduction

This experiment aims to design and control a Mini Fan, demonstrating the practical application of Finite State Machines (FSMs) in the context of an embedded digital system. The primary objective is to enhance comprehension of FSM principles and their implementation using Arduino, thereby realizing a simple embedded digital application.



## II. Requirement

### 1. Hardware

- NUCLEO -F401RE
- Ultrasonic distance sensor(HC-SR04)
- DC motor (RK-280RA)

### 2. Software

- Arduino IDE



## III. Problem

### 1. Procedure

The program should only activate the fan when the target is in close proximity and should provide two-speed control modes when the button is pressed.

1) As the button **B1** is pressed, change the fan velocity. The MODE(states)
   - MODE(state): **OFF(0%), MID(50%), HIGH(100%)**

2) When the object(face) is detected about 50 mm away, then it automatically pauses the fan temporarily.
   - Even the fan is temporarily paused, the MODE should be changed whenever the button **B1** is pressed

3) When the object(face) is detected within 50mm, then it automatically runs the fan
   - It must run at the speed of the current MODE
4) LED(**LED1**): Turned OFF when MODE=OFF. Otherwise, blink the LED with 1 sec period (1s ON, 1s OFF)

5. Print the distance and PWM duty ratio in Tera-Term console (every 1 sec).
6. Must use Mealy FSM to control the mini-fan
   - Draw a FSM(finite-state-machine) table and state diagram
   - Example Table. See below for example codes



### 2. configuration

**Ultrasonic distance sensor**

Trigger:

- Generate a trigger pulse as PWM to the sensor
- Pin: **D10** (TIM4 CH1)
- PWM out: 50ms period, 10us pulse-width

Echo:

- Receive echo pulses from the ultrasonic sensor
- Pin: **D7** (Timer1 CH1)
- Input Capture: Input mode
- Measure the distance by calculating pulse-width of the echo pulse.

**USART**

- Display measured distance in [cm] on serial monitor of Tera-Term.
- Baudrate 9600

**DC Motor**

- PWM: PWM1, set 10ms of period by default
- Pin: **D11** (Timer1 CH1N)

### 3. Circuit/Wiring Diagram

![image-20230915081004610](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230915081004610.png)



## IV. Algorithm

### 1. Overview

Mealy State Table

![image-20230915055305567](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230915055305567.png)

Mealy State Diagram

![image-20230915081652604](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230915081652604.png)

### 2. Description with Code

```c
// State definition
#define S0    0   // Fan OFF
#define S1    1   // Fan vel = 50%
#define S2    2   // Fan vel = 100%

// Address number of output in array
#define PWM 0
#define LED 1

// State table definition
typedef struct {
  uint32_t out[2][4];   // output = FSM[state].out[PWM or LED][PWM or LED State]
  uint32_t next[2][2];  // nextstate = FSM[state].next[input X][input Y]
} State_t;

State_t FSM[3] = {
  { {{0 , 0 , 0 , 50}  , {LOW , LOW , HIGH , HIGH}} , {{S0  , S0}, {S1  , S1}} },
  { {{0 ,50 , 0 ,100}  , {HIGH ,HIGH , HIGH ,HIGH}} , {{S1  , S1}, {S2  , S2}} },
  { {{0 ,100 , 0 , 0}  , {HIGH , HIGH , LOW , LOW}} , {{S2  , S2}, {S0  , S0}} },
};

// Pin setting
const int ledPin = 13;
const int pwmPin = 11;
const int btnPin = 3;
const int trigPin = 10;
const int echoPin = 7;

unsigned char state = S0;
unsigned char lastState = S0;
unsigned char input[2] = {0, 0};
unsigned char output[2][2] = {{0 , 1} , {2 , 3}};
unsigned char pwmOut = 0;
unsigned char ledOut = LOW;
unsigned char ledState = LOW;
unsigned long duration;
float distance;
int thresh = 5;

void setup() {  
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);

  // Initialize pwm pin as an output:
  pinMode(pwmPin, OUTPUT);
  
  // initialize the pushbutton pin as an interrupt input:
  pinMode(btnPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(btnPin), pressed, FALLING);

  // Initialize the trigger pin as an output
  pinMode(trigPin, OUTPUT);

  // Initialize the echo pin as an input
  pinMode(echoPin, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  // Generate pwm singal on the trigger pin.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);

  // Distance is calculated using how much time it takes.
  duration = pulseIn(echoPin, HIGH);
  distance = (float)duration / 58.0;

  // Output of states
  stateOutput();

  // Calculate next state. then update State
  nextState();

  analogWrite(pwmPin, pwmOut);
  digitalWrite(ledPin, ledOut);

  Serial.print("distance = ");
  Serial.print(distance);
  Serial.print(" [cm], ");

  Serial.print("input0 = ");
  Serial.print(input[0]);
  Serial.print(", input1 = ");
  Serial.print(input[1]);

  Serial.print(", state = ");
  Serial.println(state);

  delay(1000);
}

void pressed(){
  input[0] = 1;
  stateOutput();
  nextState();
  input[0] = 0;
}

void nextState(){
  if (distance < thresh)
    input[1] = 1;
  else
    input[1] = 0;
  
  // get nextState and Update last state
  lastState = state;
  state = FSM[state].next[input[0]][input[1]];
  if (lastState != state){
    ledState = 0;
  }
}

void stateOutput(){
  if (distance < thresh)
    input[1] = 1;
  else
    input[1] = 0;
    
  // get stateOutput
  pwmOut = FSM[state].out[PWM][output[input[0]][input[1]]];
  ledOut = FSM[state].out[LED][output[input[0]][input[1]]];

  //led blink
  if(ledOut==1){
    if(ledState == 1){
      ledOut = LOW;
      ledState = 0;
    }
    else{
      ledOut = HIGH;
      ledState = 1;
    }
  }
}

```



## Ⅴ. Results and Analysis

### 1. Results

![image-20230915084120320](C:\Users\ksw86\AppData\Roaming\Typora\typora-user-images\image-20230915084120320.png)

 The following is the distance to an object measured through an ultrasonic sensor and the change in state output through Tera Term. It was confirmed that the state change occurred correctly when the button was pressed, and the fact that the distance value was recognized as correct when the distance to the object was within 50mm can be seen through the input[1] value. As a result, it can be confirmed that all operations of the mini fan that were intended to be implemented in the experiment operate successfully.

### 2. Demo Video

link: [EC_LAB: Smart mini-fan with STM32-duino_김선우 - YouTube](https://www.youtube.com/watch?v=qafxAc4mhHY)

### 3. Analysis

 In this experiment, I successfully designed and implemented a Mini Fan using Arduino. The Mini Fan operates based on the proximity of a person's face, only turning on when the face is close and staying inactive when the face is far away. Additionally, the fan features a three-stage speed control mode activated by pressing a button.

 Prior to implementing the operation, the operation algorithm to be implemented above was first created through the Mealy FSM Table. And through programming, FSM was implemented as code, and when executed, it was confirmed that all operations worked excellently.

 Future improvements in this experiment are related to hardware issues. It was confirmed that the state was skipped twice during operation. We discovered a problem where the button was recognized twice and the states were not passed one by one, but one was skipped and operated. There are ways to estimate this, such as changing the recognition sensor or adjusting the recognition range of the button sensor in the program so that the button does not perform unnecessary recognition.

 In conclusion, through this experiment, we confirmed that FSM can be used in actual field through the experience of writing an FSM and actually implementing it. It is also significant in that it improved understanding of the theory through experience in handling a simple embedded controller.



## Reference

- Github: https://ykkim.gitbook.io/ec/ec-course/lab/lab-smart-mini-fan-with-stm32-duino

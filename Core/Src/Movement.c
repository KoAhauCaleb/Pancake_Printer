/*
 * Move.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: Caleb
 */

#include <stdbool.h>
#include "cmsis_os.h"
#include "main.h"
#include "Movement.h"



#define X_STEPS_PER_MM 20 //6.5mm - 27.6
#define Y_STEPS_PER_MM 20 //23mm - 43.5
#define Z_STEPS_PER_MM 50

#define ST 1500//step time ticks 1400-1600

extern TIM_HandleTypeDef htim1;

float current_x = 0;
float current_y = 0;


int crr1_1ms = 800;
int crr1_2ms = 1600;
int servo_on_duty = 15;
int servo_off_duty = 55;//30-50

float speed = 20.0; //mm/s
float extrude_speed = 2.0; //mm/s

bool extrusion_on = false;

void delayms(uint16_t ms){
  osDelay(ms);
  //delayus(ms * 1000);
}

void delayus(uint16_t us){
  __HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
  HAL_TIM_Base_Start(&htim1);
  while (__HAL_TIM_GET_COUNTER(&htim1) < us);
  HAL_TIM_Base_Stop(&htim1);// wait for the counter to reach the us input in the parameter
}

void motor_init(){
  //For some reason non-syncronus delay needed before synchronous delay will work.
  HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_SET);
  delayus(2);
  HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_RESET);
  delayus(1000000 / (Z_STEPS_PER_MM * extrude_speed));
  HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_SET);
  delayus(2);
  HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_RESET);
  delayus(1000000 / (Z_STEPS_PER_MM * extrude_speed));
  HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_SET);
  delayus(2);
  HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_RESET);
  delayus(1000000 / (Z_STEPS_PER_MM * extrude_speed));

  uint16_t ccr1 = (uint16_t) ((crr1_2ms - crr1_1ms) * servo_off_duty / 100) + crr1_1ms;
  TIM4->CCR1 = ccr1;
}

void set_speed(float new_speed){
  speed = new_speed;
}

void move(float x, float y){
  int x_steps = (x - current_x) * X_STEPS_PER_MM;
  int y_steps = (y - current_y) * Y_STEPS_PER_MM;
  HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_RESET);
  if(x_steps < 0){
    HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
    x_steps *= -1;
  }
  if(y_steps < 0){
    HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
    y_steps *= -1;
  }

  int max_steps = x_steps;
  if(y_steps > x_steps){
    max_steps = y_steps;
  }

  int x_step_ratio = max_steps / x_steps;
  int y_step_ratio = max_steps / y_steps;

  int x_taken = 0;
  int y_taken = 0;

  for(int i = 0; i < max_steps; i++){
    if(i % x_step_ratio == 0){
      HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_SET);
      x_taken++;
    }
    if(i % y_step_ratio == 0){
      HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_SET);
      y_taken++;
    }
    delayus(2);
    HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_RESET);
    delayms(1000 / (X_STEPS_PER_MM * speed)); // 1000000us/1s / (step/mm * mm/s)
  }

  while (x_steps > x_taken){
    HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_SET);
    delayus(2);
    HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_RESET);
    x_taken++;
  }
  while (y_steps > y_taken){
    HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_SET);
    delayus(2);
    HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_RESET);
    y_taken++;
  }
  current_x = x;
  current_y = y;
}

void wait(int ms){
  delayms(ms);
}

void home() {
  HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
  //move fast until limit switch hit
  while(!HAL_GPIO_ReadPin(X_STOP_GPIO_Port, X_STOP_Pin)){
    HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_SET);
    delayus(2);
    HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_RESET);
    delayms(1000 / (X_STEPS_PER_MM * speed));
  }
  while(!HAL_GPIO_ReadPin(Y_STOP_GPIO_Port, Y_STOP_Pin)){
    HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_SET);
    delayus(2);
    HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_RESET);
    delayms(1000 / (X_STEPS_PER_MM * speed));
  }

  current_x = 0;
  current_y = 0;

  //move away from limit switch by small distance
  move(10,10);

  //move slow towards limit switch
  HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
  //move fast until limit switch hit
  while(!HAL_GPIO_ReadPin(X_STOP_GPIO_Port, X_STOP_Pin)){
    HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_SET);
    delayus(2);
    HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_RESET);
    delayms(1000 / (X_STEPS_PER_MM * 10));
  }
  while(!HAL_GPIO_ReadPin(Y_STOP_GPIO_Port, Y_STOP_Pin)){
    HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_SET);
    delayus(2);
    HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_RESET);
    delayms(1000 / (X_STEPS_PER_MM * 10));
  }

  current_x = 0;
  current_y = 0;
}

void start_extrusion(){
  extrusion_on = true;

  uint16_t ccr1 = (uint16_t) (((crr1_2ms - crr1_1ms) * (servo_on_duty / 100.0)) + crr1_1ms);
  TIM4->CCR3 = ccr1;
}

void stop_extrusion(){
  extrusion_on = false;

  uint16_t ccr1 = (uint16_t) ((crr1_2ms - crr1_1ms) * (servo_off_duty / 100.0)) + crr1_1ms;
  TIM4->CCR3 = ccr1;
}

void extrude(){
    if(extrusion_on){
      HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_SET);
      delayus(2);
      HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_RESET);
      delayms(1000 / (Z_STEPS_PER_MM * extrude_speed));
    }
}

void reset_extruder(){

  HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_SET);

  while(!HAL_GPIO_ReadPin(X_STOP_GPIO_Port, X_STOP_Pin)){
    HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_SET);
    delayus(2);
    HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000 / (X_STEPS_PER_MM * 40));
  }

  HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_RESET);
}

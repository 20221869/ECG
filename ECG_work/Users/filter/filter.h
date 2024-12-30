//
// Created by Lyt on 2024/12/25.
//

#ifndef FINAL_TASK_FILTER_H
#define FINAL_TASK_FILTER_H

#include "stm32f4xx.h"
#include "arm_math.h"

//#define LENGTH_SAMPLES       10    //????
#define BLOCK_SIZE           1      //????dsp???????
#define NUM_TAPS             49       //?????
#define midfilt_num 125

void arm_fir_f32_lp_48(float32_t *Input_buffer,float32_t *Output_buffer);

void arm_fir_f32_lp_5(float32_t *Input_buffer,float32_t *Output_buffer);

void arm_fir48_init(void);

void arm_fir5_init(void);

float32_t midfilt1(float32_t *p_input,int size,int blocksize);

float32_t find_mid_val(float32_t *p_input,int size);

void maxim_sort_ascend(float32_t *pn_x,int32_t n_size);
#endif //FINAL_TASK_FILTER_H

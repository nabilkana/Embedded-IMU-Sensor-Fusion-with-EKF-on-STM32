/*
 * ekf.h
 *
 *  Created on: Jun 21, 2025
 *      Author: nabil
 */

#ifndef SRC_EKF_H_
#define SRC_EKF_H_


#include "stm32f1xx_hal.h"
#include <string.h>

#include <stdint.h>
#include <math.h>

#define STATE_DIM 2
#define INPUT_DIM 3
#define MEAS_DIM 2

typedef struct {
    float x[STATE_DIM];
    float P[STATE_DIM][STATE_DIM];
    float Q[STATE_DIM][STATE_DIM];
    float R[MEAS_DIM][MEAS_DIM];
} EKF_t;

void ekf_init(EKF_t *ekf);
void ekf_predict(EKF_t *ekf, float u[INPUT_DIM], float dt);
void ekf_update(EKF_t *ekf, float y[MEAS_DIM],UART_HandleTypeDef *huart);



#endif /* SRC_EKF_H_ */

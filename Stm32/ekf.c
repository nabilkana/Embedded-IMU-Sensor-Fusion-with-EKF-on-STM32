#include "ekf.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "stdio.h"
#include <math.h>

void ekf_init(EKF_t *ekf) {
    for (int i = 0; i < STATE_DIM; i++) {
        ekf->x[i] = 0.0f;
        for (int j = 0; j < STATE_DIM; j++) {
            ekf->P[i][j] = (i == j) ? 0.001f : 0.0f;
            ekf->Q[i][j] = 0.0f;
        }
    }
    ekf->Q[0][0] = 0.0151491f;
    ekf->Q[1][1] = 0.01843767f; // gy variance

    ekf->R[0][0] = 4.596579e-6f;  // Roll measurement variance
    ekf->R[1][1] = 4.306421e-6f;
    ekf->R[0][1] = 0.0f;
    ekf->R[1][0] = 0.0f;
}

void compute_jacobian(float A[STATE_DIM][STATE_DIM], float phi, float theta, float u[INPUT_DIM]) {
    float wx = u[0], wy = u[1], wz = u[2];
    float tan_theta = tanf(theta);
    float sec_theta2 = 1.0f / (cosf(theta) * cosf(theta));
    A[0][0] = wy * cosf(phi) * tan_theta - wz * sinf(phi) * tan_theta;
    A[0][1] = wy * sinf(phi) * sec_theta2 + wz * cosf(phi) * sec_theta2;
    A[1][0] = -wy * sinf(phi) - wz * cosf(phi);
    A[1][1] = 0.0f;
}

void matrix_add(float A[STATE_DIM][STATE_DIM], float B[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]) {
    for (int i = 0; i < STATE_DIM; i++)
        for (int j = 0; j < STATE_DIM; j++)
            result[i][j] = A[i][j] + B[i][j];
}

void matrix_mult_ATPA(float A[STATE_DIM][STATE_DIM], float P[STATE_DIM][STATE_DIM], float result[STATE_DIM][STATE_DIM]) {
    float temp[STATE_DIM][STATE_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++)
        for (int j = 0; j < STATE_DIM; j++)
            for (int k = 0; k < STATE_DIM; k++)
                temp[i][j] += A[i][k] * P[k][j];

    for (int i = 0; i < STATE_DIM; i++)
        for (int j = 0; j < STATE_DIM; j++) {
            result[i][j] = 0.0f;
            for (int k = 0; k < STATE_DIM; k++)
                result[i][j] += temp[i][k] * A[j][k];
        }
}

void ekf_predict(EKF_t *ekf, float u[INPUT_DIM], float dt) {
    float phi = ekf->x[0];
    float theta = ekf->x[1];
    float wx = u[0], wy = u[1], wz = u[2];

    float dphi = wx + wy * sinf(phi) * tanf(theta) + wz * cosf(phi) * tanf(theta);
    float dtheta = wy * cosf(phi) - wz * sinf(phi);

    ekf->x[0] += dphi * dt;
    ekf->x[1] += dtheta * dt;

    float A[STATE_DIM][STATE_DIM];
    compute_jacobian(A, phi, theta, u);

    float APAT[STATE_DIM][STATE_DIM];
    matrix_mult_ATPA(A, ekf->P, APAT);
    matrix_add(APAT, ekf->Q, ekf->P);
}

void ekf_update(EKF_t *ekf, float y[MEAS_DIM], UART_HandleTypeDef *huart) {
    char msg_local[128];

    float C[MEAS_DIM][STATE_DIM] = {
        {1.0f, 0.0f},
        {0.0f, 1.0f}
    };

    float y_hat[MEAS_DIM] = { ekf->x[0], ekf->x[1] };

    float y_err[MEAS_DIM] = { y[0] - y_hat[0], y[1] - y_hat[1] };



    for (int i = 0; i < MEAS_DIM; i++) {
        if (!isfinite(y_err[i])) return;
    }

    float CP[MEAS_DIM][STATE_DIM] = {0};
    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                CP[i][j] += C[i][k] * ekf->P[k][j];
            }
        }
    }

    float S[MEAS_DIM][MEAS_DIM] = {0};
    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                S[i][j] += CP[i][k] * C[j][k];
            }
            S[i][j] += ekf->R[i][j];
        }
    }

    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (fabsf(det) < 1e-8f || !isfinite(det)) return;

    float invS[2][2];
    invS[0][0] =  S[1][1] / det;
    invS[0][1] = -S[0][1] / det;
    invS[1][0] = -S[1][0] / det;
    invS[1][1] =  S[0][0] / det;

    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            if (!isfinite(invS[i][j])) return;
        }
    }

    float PCt[STATE_DIM][MEAS_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                PCt[i][j] += ekf->P[i][k] * C[j][k];
            }
        }
    }

    float K[STATE_DIM][MEAS_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            for (int k = 0; k < MEAS_DIM; k++) {
                K[i][j] += PCt[i][k] * invS[k][j];
            }
            if (!isfinite(K[i][j])) return;
        }
    }

    for (int i = 0; i < STATE_DIM; i++) {
        float update = 0.0f;
        for (int j = 0; j < MEAS_DIM; j++) {
            update += K[i][j] * y_err[j];
        }
        ekf->x[i] += update;
        if (!isfinite(ekf->x[i])) return;
    }

    float I_KC[STATE_DIM][STATE_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            I_KC[i][j] = (i == j) ? 1.0f : 0.0f;
            for (int k = 0; k < MEAS_DIM; k++) {
                I_KC[i][j] -= K[i][k] * C[k][j];
            }
        }
    }

    float newP[STATE_DIM][STATE_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                newP[i][j] += I_KC[i][k] * ekf->P[k][j];
            }
            if (!isfinite(newP[i][j])) {
                newP[i][j] = (i == j) ? 0.1f : 0.0f;
            }
        }
    }

    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            ekf->P[i][j] = newP[i][j];
        }
    }
}
